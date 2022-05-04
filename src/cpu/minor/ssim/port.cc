#include <bitset>
#include "./accel.hh"
#include "./ssim.hh"
#include "./stream.hh"
#include "./port.h"
#include "dsa/dfg/port.h"
#include "dsa/simulation/data.h"


namespace dsa {
namespace sim {

std::string IVPState::toString() {
  std::ostringstream oss;
  if (exec_repeat() != 1) {
    oss << "repeat=" << repeat;
  }
  if (stretch) {
    oss << ", stretch=" << stretch;
  }
  return oss.str().empty() ? "[default]" : oss.str();
}

bool PortExecState::tick() {
  if (exec_repeat()) {
    DSA_LOG(REPEAT) << "[" << port << "] ++: " << repeat_counter << "/" << exec_repeat();
    repeat_counter = (repeat_counter + 1) % exec_repeat();
  } else {
    // If it is zero, we should avoid dividen by zero,
    // and just pop the value.
    repeat_counter = 0;
  }
  if (repeat_counter == 0) {
    DSA_LOG(REPEAT) << "before stetching: " << exec_repeat();
    repeat += stretch;
    DSA_LOG(REPEAT) << "stetched: " << exec_repeat();
    return true;
  }
  return false;
}

std::string PortExecState::toString() {
  std::ostringstream oss;
  oss << "port=" << port;
  if (repeat != 1) {
    oss << ", repeat=" << repeat_counter << "/" << repeat
        << "(" << (double) repeat / (1 << DSA_REPEAT_DIGITAL_POINT) << ")";
  }
  if (stretch) {
    oss << ", stretch=" << stretch
        << "(" << (double) stretch / (1 << DSA_REPEAT_DIGITAL_POINT) << ")";
  }
  return oss.str();
}

void Port::bindStream(base_stream_t *s) {
  DSA_CHECK(!stream) << "Now serving stream " << s << ": " << s->toString();
  stream = s;
}

template<typename T>
void find_and_erase(std::vector<T> &a, T ky) {
  auto iter = std::find(a.begin(), a.end(), ky);
  DSA_CHECK(iter != a.end());
  a.erase(iter);
}

void Port::freeStream() {
  DSA_CHECK(stream) << "No stream to free!";
  DSA_CHECK(!stream->stream_active()) << "Stream is still ongoing!";
  struct PostProcessor : dsa::sim::stream::Functor {
    void Visit(LinearReadStream *lrs) {
      if (lrs->be) {
        find_and_erase<base_stream_t*>(lrs->be->referencer, lrs);
      }
    }
    void Visit(LinearWriteStream *lws) {
      if (lws->be) {
        find_and_erase<base_stream_t*>(lws->be->referencer, lws);
      }
    }
  };
  PostProcessor functor;
  DSA_LOG(COMMAND) << id() << " free " << stream->toString();
  stream->Accept(&functor);
  stream = nullptr;
}

int InPort::id() const {
  return pes.port;
}

void InPort::tick() {
  // TODO(@were): Implement this.
}

int Port::scalarSizeInBytes() const {
  DSA_CHECK(vp) << "scalarSizeInBytes() is only meaningful when configured.";
  return vp->bitwidth() / 8;
}

int Port::vectorBytes() const {
  return scalarSizeInBytes() * vectorLanes();
}

int InPort::vectorLanes() const {
  DSA_CHECK(vp) << "vectorLanes() is only meaningful when configured.";
  return ivp()->vectorLanes();
}

int OutPort::vectorLanes() const {
  DSA_CHECK(vp) << "vectorLanes() is only meaningful when configured.";
  return ovp()->vectorLanes();
}

void InPort::push(const std::vector<uint8_t> &raw, const stream::AffineStatus &as, bool imm) {
  int dbytes = scalarSizeInBytes();
  int lanes = vectorLanes();
  if (!imm) {
    ongoing -= raw.size();
    DSA_LOG(PORT) << "Deregister " << raw.size() << " bytes for port " << id();
  }
  DSA_CHECK(ongoing >= 0);
  // TODO(@were): This significantly hurt the performance.
  // DSA_CHECK(canPush(true) >= raw.size())
  //   << buffer_size << " - " << ongoing << " - " << buffer.size() << " * " << scalarSizeInBytes()
  //   << " = " << canPush(true) << " < " << raw.size();
  DSA_CHECK(raw.size() % dbytes == 0)
    << raw.size() << " % " << dbytes << " != 0, cannot gaurantee the alignment of predicate!";
  int residue = as.n % (lanes * dbytes);
  int padCount = residue ? (lanes * dbytes) - residue : 0;
  if (as.n == -1) {
    padCount = 0;
  } else {
    DSA_CHECK(padCount % dbytes == 0) << as.n << " % (" << lanes << " * " << dbytes << ")";
    padCount /= dbytes;
  }

  // TODO(@were): Make sure this is correct!
  // auto cpu_freq = parent->lsq()->get_cpu().clockDomain.clockPeriod();
  auto aa = parent->now();// + cpu_freq;
#define PADDING_IMPL(cond, zero_enum, predoff_enum)                \
  do {                                                             \
    if (cond) {                                                    \
      if (as.padding == zero_enum) { \
        DSA_LOG(PAD) << "Padding: " << as.n << " " << residue << " " << padCount; \
        SpatialPacket padValue(aa, 0, as.padding == zero_enum);    \
        for (int i = 0; i < padCount; ++i) {                       \
          buffer.emplace_back(padValue);                           \
        }                                                          \
      }                                                            \
    }                                                              \
  } while (false)

  int from = buffer.size();

  for (int i = 0; i < raw.size(); i += dbytes) {
    uint64_t data = 0;
    std::memcpy(&data, raw.data() + i, dbytes);
    buffer.emplace_back(SpatialPacket(aa, data, true));
  }
  PADDING_IMPL(as.dim_last, DP_PostStrideZero, DP_PostStridePredOff);
#undef PADDING_IMPL
  DSA_LOG(PORT)
    << id() << ": Buffered: " << buffer.size() * scalarSizeInBytes() << ", Ongoing: " << ongoing;

  if (as.penetrate_state.empty()) {
    for (int i = from; i < (int) buffer.size(); ++i) {
      buffer[i].stream_state = as.toTag(i == from, i == buffer.size() - 1);
    }
  } else {
    DSA_CHECK(as.penetrate_state.size() == (buffer.size() - from))
      << as.penetrate_state.size() << " != " << (buffer.size() - from);
    for (int i = from; i < (int) buffer.size(); ++i) {
      buffer[i].stream_state = as.penetrate_state[i - from];
    }
  }
  for (int i = from; i < (int) buffer.size(); ++i) {
    DSA_LOG(PORT)
      << "Push: " << buffer[i].value << "(" << buffer[i].valid << ", "
      << std::bitset<8>(buffer[i].stream_state).to_string() << ")";
  }
}

int InPort::canPush(bool ip) {
  int padding_buffer = ip * vectorLanes() * scalarSizeInBytes();
  return buffer_size + padding_buffer  - (ongoing + buffer.size() * scalarSizeInBytes());
}

bool InPort::lanesReady() {
  int n = vectorLanes();
  if (buffer.size() < n) {
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (parent->now() < buffer[i].available_at) {
      return false;
    }
  }
  if (pes.exec_repeat() == 0) {
    pop();
    return false;
  }
  return true;
}

void InPort::pop() {
  DSA_CHECK(lanesReady());
  if (pes.tick()) {
    int n = vectorLanes();
    if (ivp()->stationary_shift == -1) {
      buffer.erase(buffer.begin(), buffer.begin() + n);
    } else {
      // If we reach the end of stream, clear the shift state, and all the remaining data.
      if (buffer[n - 1].stream_state & 2) {
        ivp()->stationary_shift = -1;
        buffer.erase(buffer.begin(), buffer.begin() + n);
      } else {
        buffer.erase(buffer.begin(), buffer.begin() + ivp()->stationary_shift);
      }
    }
    DSA_LOG(PORT)
      << id() << ": Buffered: " << buffer.size() * scalarSizeInBytes() << ", Ongoing: " << ongoing;
  }
}

void OutPort::pop(int n) {
  DSA_CHECK(raw.size() >= n);
  raw.erase(raw.begin(), raw.begin() + n);
  int ops = ovp()->penetrated_state;
  if (ops != -1) {
    state.erase(state.begin());
  }
}

void InPort::reset() {
  buffer.clear();
}

bool InPort::empty() {
  return buffer.empty();
}

std::vector<PortPacket> InPort::poll() {
  int n = vectorLanes();
  DSA_CHECK(lanesReady());
  std::vector<PortPacket> res(buffer.begin(), buffer.begin() + n);
  return res;
}

void OutPort::push(const std::vector<uint8_t> &data) {
  raw.insert(raw.end(), data.begin(), data.end());
}

int OutPort::canPop(int n) {
  int ops = ovp()->penetrated_state;
  if (ops != -1) {
    // TODO(@were): Support vectorized!
    if (state.size() < 1) {
      return false;
    }
  }
  return raw.size() >= n;
}

std::vector<uint8_t> OutPort::poll(int n) {
  DSA_CHECK(canPop(n));
  std::vector<uint8_t> res;
  int ops = ovp()->penetrated_state;
  res.reserve(n + (ops != -1));
  for (int i = 0; i < n; ++i) {
    res.push_back(raw[i]);
  }
  if (ops != -1) {
    res.push_back(state[0]);
    DSA_LOG(PENE) << "Penetrate: " << (int) res.back() << " " << std::bitset<6>(res.back());
  }
  return res;
}

dfg::InputPort *InPort::ivp() const {
  auto *res = dynamic_cast<dfg::InputPort *>(vp);
  DSA_CHECK(res) << "The configured vp for input port should be an input, but get " << vp->name();
  return res;
}

dfg::OutputPort *OutPort::ovp() const {
  auto *res = dynamic_cast<dfg::OutputPort *>(vp);
  DSA_CHECK(res) << "The configured vp for output port should be an output, but get " << vp->name();
  return res;
}

int InPort::bytesBuffered() const {
  return buffer.size() * scalarSizeInBytes();
}

int OutPort::bytesBuffered() const {
  return raw.size();
}

}
}
