#include "./accel.hh"
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
    repeat_counter = (repeat_counter + 1) % exec_repeat();
  } else {
    // If it is zero, we should avoid dividen by zero,
    // and just pop the value.
    repeat_counter = 0;
  }
  if (repeat_counter == 0) {
    repeat += stretch;
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
  CHECK(!stream) << "Now serving stream " << s << ": " << s->toString();
  stream = s;
}

template<typename T>
void find_and_erase(std::vector<T> &a, T ky) {
  auto iter = std::find(a.begin(), a.end(), ky);
  CHECK(iter != a.end());
  a.erase(iter);
}

void Port::freeStream() {
  CHECK(stream) << "No stream to free!";
  CHECK(!stream->stream_active()) << "Stream is still ongoing!";
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
  CHECK(vp) << "scalarSizeInBytes() is only meaningful when configured.";
  return vp->bitwidth() / 8;
}

int InPort::vectorLanes() const {
  CHECK(vp) << "vectorLanes() is only meaningful when configured.";
  return vp->values.size();
}

int OutPort::vectorLanes() const {
  CHECK(vp) << "vectorLanes() is only meaningful when configured.";
  return vp->ops().size();
}

void InPort::push(const std::vector<uint8_t> &raw, const stream::AffineStatus &as, bool imm) {
  int dbytes = scalarSizeInBytes();
  int lanes = vectorLanes();
  if (!imm) {
    ongoing -= raw.size();
    DSA_LOG(PORT) << "Deregister " << raw.size() << " bytes for port " << id();
  }
  CHECK(ongoing >= 0);
  CHECK(canPush(true) >= raw.size())
    << buffer_size << " - " << ongoing << " - " << buffer.size() << " * " << scalarSizeInBytes()
    << " = " << canPush(true) << " < " << raw.size();
  CHECK(raw.size() % dbytes == 0)
    << raw.size() << " % " << dbytes << " != 0, cannot gaurantee the alignment of predicate!";
  int residue = as.n % (lanes * dbytes);
  int padCount = residue ? (lanes * dbytes) - residue : 0;
  if (as.n == -1) {
    padCount = 0;
  } else {
    CHECK(padCount % dbytes == 0) << as.n << " % (" << lanes << " * " << dbytes << ")";
    padCount /= dbytes;
  }

  // TODO(@were): Make sure this is correct!
  auto aa = _curEventQueue->nextTick();
#define PADDING_IMPL(cond, zero_enum, predoff_enum)                \
  do {                                                             \
    if (cond) {                                                    \
      if (as.padding == zero_enum || as.padding == predoff_enum) { \
        DSA_LOG(PAD) << "Padding: " << as.n << " " << residue << " " << padCount; \
        SpatialPacket padValue(aa, 0, as.padding == zero_enum);    \
        for (int i = 0; i < padCount; ++i) {                       \
          buffer.emplace_back(padValue);                           \
        }                                                          \
      }                                                            \
    }                                                              \
  } while (false)

  int from = buffer.size();

  PADDING_IMPL(as.dim_1st, DP_PreStrideZero, DP_PreStridePredOff);
  for (int i = 0; i < raw.size(); i += dbytes) {
    uint64_t data = 0;
    std::memcpy(&data, raw.data() + i, dbytes);
    buffer.emplace_back(SpatialPacket(aa, data, true));
  }
  PADDING_IMPL(as.dim_last, DP_PostStrideZero, DP_PostStridePredOff);
#undef PADDING_IMPL
  DSA_LOG(PORT)
    << id() << ": Buffered: " << buffer.size() * scalarSizeInBytes() << ", Ongoing: " << ongoing;

  for (int i = from; i < (int) buffer.size(); ++i) {
    buffer[i].tag = as.toTag(i == from, i == buffer.size() - 1);
    DSA_LOG(PORT) << "Push: " << buffer[i].value << "(" << buffer[i].valid << ")";
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
    if (curTick() < buffer[i].available_at) {
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
  CHECK(lanesReady());
  if (pes.tick()) {
    int n = vectorLanes();
    buffer.erase(buffer.begin(), buffer.begin() + n);
    DSA_LOG(PORT)
      << id() << ": Buffered: " << buffer.size() * scalarSizeInBytes() << ", Ongoing: " << ongoing;
  }
}

void OutPort::pop(int n) {
  CHECK(raw.size() >= n);
  raw.erase(raw.begin(), raw.begin() + n);
}

void InPort::reset() {
  buffer.clear();
}

bool InPort::empty() {
  return buffer.empty();
}

std::vector<PortPacket> InPort::poll() {
  int n = vectorLanes();
  CHECK(lanesReady());
  std::vector<PortPacket> res(buffer.begin(), buffer.begin() + n);
  return res;
}

void OutPort::push(const std::vector<uint8_t> &data) {
  raw.insert(raw.end(), data.begin(), data.end());
}

std::vector<uint8_t> OutPort::poll(int n) {
  CHECK(canPop(n));
  std::vector<uint8_t> res;
  res.reserve(n);
  for (int i = 0; i < n; ++i) {
    res.push_back(raw[i]);
  }
  return res;
}

dfg::InputPort *InPort::ivp() {
  auto *res = dynamic_cast<dfg::InputPort *>(vp);
  CHECK(res) << "The configured vp for input port should be an input, but get " << vp->name();
  return res;
}

dfg::OutputPort *OutPort::ovp() {
  auto *res = dynamic_cast<dfg::OutputPort *>(vp);
  CHECK(res) << "The configured vp for output port should be an output, but get " << vp->name();
  return res;
}

}
}
