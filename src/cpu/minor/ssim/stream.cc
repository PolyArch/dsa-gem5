#include <limits>
#include <numeric>
#include <functional>

#include "./ism.h"
#include "accel.hh"
#include "stream.hh"
#include "sim-debug.hh"

const char *LOC_NAME[] = {
#define MACRO(x) #x
#include "loc.def"
#undef MACRO
};

int base_stream_t::ID_SOURCE = 0;

#define CQ_PTR(x, delta)               \
  do {                                 \
    x += (delta);                      \
    if (x >= end) {                    \
      x = begin + (x) - end;           \
      DSA_CHECK(x >= begin && x <= end);   \
    }                                  \
  } while (false)

bool BuffetEntry::EnforceReadWrite(int64_t addr, int word) {
  if (mo == MemoryOperation::DMO_Read) {
    return InRange(addr);
  } else if (mo == MemoryOperation::DMO_Write) {
    return addr >= address && addr < address + Size();
  }
  DSA_CHECK(false) << "Not supported yet!";
  return false;
}

int64_t BuffetEntry::Translate(int64_t addr, MemoryOperation mo) {
  switch (mo) {
  case DMO_Write:
    DSA_CHECK(SpaceAvailable() && addr == address + occupied)
      << "For now only appended writing is supported!"
      << "request: " << addr << ", address: [" << address << ", " << address + occupied << ")"
      << ", Size: " << Size();
    break;
  case DMO_Read:
    DSA_CHECK(InRange(addr))
      << "Address out of range: " << addr << " not in ["
      << address << ", " << address + occupied << ")";
    break;
  default:
    DSA_CHECK(false) << "Not supported: " << mo;
  }
  CQ_PTR(addr, front - address);
  return addr;
}

void BuffetEntry::Append(int bytes) {
  DSA_CHECK(occupied + bytes <= Size())
    << "Buffet size overflow!" << occupied << " " << bytes << " " << Size();
  occupied += bytes;
  CQ_PTR(tail, bytes);
  if (bytes) {
    DSA_LOG(BUFFET) << "Append " << bytes << ", now: " << toString();
  }
}

void BuffetEntry::Shrink(int bytes) {
  DSA_CHECK(occupied - bytes >= 0)
    << "Buffet size underflow!" << occupied << " " << bytes << " " << Size();
  occupied -= bytes;
  address += bytes;
  CQ_PTR(front, bytes);
  if (bytes) {
    DSA_LOG(BUFFET) << "Pop " << bytes << ", now: " << toString();
  }
}

#undef CQ_PTR

int BuffetEntry::SpaceAvailable() {
  int64_t res = 0;
  if (front < tail) {
    res = tail - front;
  } else {
    res = (end - front) + (tail - begin);
  }
  if (res != occupied) {
    DSA_CHECK(occupied == 0 || occupied == end - begin)
      << occupied << " ? " << end - begin << " | "
      << front << ", " << tail;
  }
  return Size() - occupied;
}

void base_stream_t::set_mem_map_config() {
  if(_part_size==0) return;
  _part_bits = log2(_part_size);
  _core_bits = log2(_num_dist_cores);

}

std::string BuffetEntry::toString() {
  std::ostringstream oss;
  oss << "Alloc: [" << begin << ", " << end << "), Ptr: ["
      << front << ", " << tail << "), Buffered: [" << address
      << ", " << address + occupied << "), occupied: "
      << occupied << ", space: " << SpaceAvailable();
  return oss.str();
}

// based on memory mapping, extract these two information
uint64_t base_stream_t::get_core_id(addr_t logical_addr) {
  if(_part_size==0) return logical_addr/SCRATCH_SIZE;
  int core_id = (logical_addr >> _part_bits) & (_num_dist_cores-1);
  return _used_cores[core_id];
}

// PART_CORE_REST_BANK
addr_t base_stream_t::memory_map(addr_t logical_addr, addr_t cur_scr_offset) {
  if(_part_size==0) return logical_addr;

  logical_addr -= cur_scr_offset;

  int part_offset = logical_addr & (_part_size-1); // address inside a partition

  int part_id = logical_addr >> (_part_bits+_core_bits); // which partition index

  int mapped_local_scr_addr = cur_scr_offset + part_offset + (part_id*_part_size);

  int core_id = (logical_addr >> _part_bits) & (_num_dist_cores-1); // 0th core

  if(mapped_local_scr_addr==SCRATCH_SIZE) { // looping
    mapped_local_scr_addr=0;
    core_id = (core_id+1)%_num_dist_cores;
  }

  assert(mapped_local_scr_addr<SCRATCH_SIZE);

  
  // adding the importance of core
  mapped_local_scr_addr = SCRATCH_SIZE*_used_cores[core_id] + mapped_local_scr_addr;

  DSA_LOG(SHOW_CONFIG)
    << "original addr: " << logical_addr << " scr offset: " << cur_scr_offset
    << " extracted part_offset: " << part_offset << " part_id: " << part_id
    << " core_id: " << core_id << " mapped scr addr: " << mapped_local_scr_addr << "\n";

  return mapped_local_scr_addr;
}


std::string LinearReadStream::toString() {
  auto bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << "\t" << ls->toString();
  for (auto &elem : pes) {
    oss << " " << elem.toString();
    if (bsw) {
      oss << "(" << bsw->name(true, elem.port) << ")";
    }
  }
  oss << ", pad: " << padding;
  return oss.str();
}

template<typename T>
void replace_vec_elem(std::vector<T> &a, T tgt, T tbr) {
  auto iter = std::find(a.begin(), a.end(), tgt);
  DSA_CHECK(iter != a.end());
  *iter = tbr;
}

OPortStream *LinearWriteStream::clone(accel_t *accel) {
  auto res = new LinearWriteStream(*this);
  // After dispatching, the buffet entry should be updated accordingly.
  if (res->be) {
    replace_vec_elem<base_stream_t*>(res->be->referencer, this, res);
  }
  res->parent = accel;
  return res;
}

IPortStream *LinearReadStream::clone(accel_t *accel){
  auto res = new LinearReadStream(*this);
  if (res->be) {
    replace_vec_elem<base_stream_t*>(res->be->referencer, this, res);
  }
  res->parent = accel;
  return res;
}

std::string LinearWriteStream::toString() {
  auto bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << "\t" << ls->toString() << " out_port="
      << port();
  if (bsw) {
    oss << "(" << bsw->name(false, port()) << ")";
  }
  if (garbage()) {
    oss << " garbage";
  }
  return oss.str();
}

std::string ConstPortStream::toString() {
  auto bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << " dtype: " << dtype << " " << ls->toString();
  for (auto &elem : pes) {
    oss << " " << elem.toString();
    if (bsw) {
      oss << "(" << bsw->name(true, elem.port) << ")";
    }
  }
  return oss.str();
}

std::string IndirectReadStream::toString() {
  auto *bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << " " << fsm.toString(parent);
  for (auto &elem : pes) {
    oss << "ports: " << elem.toString();
    if (bsw) {
      oss << "(" << bsw->name(true, elem.port) << ")";
    }
  }
  return oss.str();
}

std::string IndirectGenerationStream::toString() {
  auto *bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << " " << fsm.toString(parent);
  for (auto &elem : pes) {
    oss << "ports: " << elem.toString();
    if (bsw) {
      oss << "(" << bsw->name(true, elem.port) << ")";
    }
  }
  return oss.str();
}

std::string IndirectAtomicStream::toString() {
  std::ostringstream oss;
  oss << short_name() << " " << fsm.toString(parent);
  return oss.str();
}

std::string RecurrentStream::toString() {
  auto bsw = parent ? &parent->bsw : nullptr;
  std::ostringstream oss;
  oss << short_name() << " " << oports[0];
  if (bsw) {
    oss << "(" << bsw->name(false, oports[0]) << ")";
  }
  oss << " ->";
  for (auto &elem : pes) {
    oss << elem.toString();
    if (bsw) {
      oss << "(" << bsw->name(true, elem.port) << ")";
    }
  }
  oss << " " << i << "/" << n;
  return oss.str();
}


namespace dsa {
namespace sim {
namespace stream {

int LinearStream::LineInfo::bytes_read() const {
  return std::accumulate(mask.begin(), mask.end(), 0, std::plus<int>());
}

LinearStream::LineInfo Linear1D::cacheline(int bandwidth, int at_most, BuffetEntry *be,
                                           MemoryOperation mo, LOC loc) {
  DSA_CHECK(hasNext());
  AffineStatus as;
  as.stream_1st = i == 0;
  as.dim_1st = i == 0 ? 1 : 0;
  std::vector<bool> mask(bandwidth, 0);
  DSA_CHECK(bandwidth == (bandwidth & -bandwidth));
  int64_t head = poll(false);
  if (be) {
    head = be->Translate(head, mo);
  }
  int64_t base = ~(bandwidth - 1) & head;
  DSA_LOG(LI) << "Head: " << head << ", Base: " << base;
  int64_t cnt = 0;
  int64_t current = -1;
  int64_t untranslated = -1;
  int64_t prev = -1;
  while (cnt < at_most && hasNext()) {
    prev = current;
    untranslated = current = poll(false);
    if (be && !be->EnforceReadWrite(current, word)) {
      DSA_LOG(LI) << "Cannot write to buffet!";
      break;
    }
    if (be) {
      current = be->Translate(current, mo);
      prev = current;
    }
    DSA_CHECK(current >= base);
    if (current < base + bandwidth) {
      DSA_LOG(LI) << "Address: " << current << " x " << word;
      if (mo != DMO_Read && loc == LOC::DMA && (prev != -1 && current - prev != word)) {
        DSA_LOG(LI) << "Break by incontinuous write!";
        break;
      }
      if (be) {
        if (be->mo == DMO_Write) {
          // Commit buffet append here.
          be->Append(word);
        }
      }
      for (int j = 0, n = abs(word); j < n; ++j) {
        DSA_CHECK(current - base + j >= 0 && current - base + j < mask.size())
          << current << " - " << base << " + " << j << " = "
          << current - base + j << " not in [0, " << mask.size() << ")";
        mask[(current - base) + j] = true;
        ++cnt;
      }
      poll(true); // pop the current one
    } else {
      DSA_LOG(LI) << "Break by bandwidth: " << current << " >= " << base << " + " << bandwidth;
      break;
    }
  }
  DSA_LOG(SHRINK) << "shrink: " << untranslated + word;
  as.dim_last = i == length ? 1 : 0;
  as.n = length * word;
  as.stream_last = !hasNext();
  return LineInfo(base, head, mask, untranslated + word, as);
}

void Functor::Visit(base_stream_t *) {}

void Functor::Visit(IPortStream *is) {
  Visit(static_cast<base_stream_t*>(is));
}

void Functor::Visit(OPortStream *os) {
  Visit(static_cast<base_stream_t*>(os));
}

void Functor::Visit(PortPortStream *os) {
  Visit(static_cast<base_stream_t*>(os));
}

void Functor::Visit(LinearReadStream *ars) {
  Visit(static_cast<IPortStream*>(ars));
}

void Functor::Visit(LinearWriteStream *aws) {
  Visit(static_cast<OPortStream*>(aws));
}

void Functor::Visit(ConstPortStream *cps) {
  Visit(static_cast<IPortStream*>(cps));
}

void Functor::Visit(Barrier *bar) {
  // I do not think stream barrier should comply to a stream base.
}

void Functor::Visit(IndirectReadStream *irs) {
  Visit(static_cast<PortPortStream*>(irs));
}

void Functor::Visit(IndirectGenerationStream *irs) {
  Visit(static_cast<PortPortStream*>(irs));
}

void Functor::Visit(RecurrentStream *rs) {
  Visit(static_cast<PortPortStream*>(rs));
}

void Functor::Visit(IndirectAtomicStream *rs) {
  Visit(static_cast<OPortStream*>(rs));
}


void Functor::Visit(SentinelStream *) {
  // Do nothing.
}

BarrierFlag Loc2BarrierFlag(LOC loc) {
  return loc == LOC::DMA ? DBF_DMAStreams : DBF_SPadStreams;
}

struct IFSMTicker : Functor {
#define SET_PTR(x, y) \
  do {                \
    x##_ = y;         \
    x = &x##_;        \
  } while (false)

  void Visit(RecurrentStream *rs) override {
    auto &ovp = accel->output_ports[rs->oports[0]];
    DSA_CHECK(rs->stream_active());
    if (ovp.canPop(rs->dtype)) {
      stream_first = (rs->i == 0);
      SET_PTR(i, rs->i);
      auto data = ovp.poll(rs->dtype);
      state = 0;
      if (data.size() == rs->dtype + 1) {
        SET_PTR(state, data.back());
        DSA_LOG(IFSM) << "Penetrated State: " << std::bitset<8>(state).to_string();
        data.back() = 0;
      }
      data.resize(8, 0);
      auto value = *reinterpret_cast<uint64_t*>(data.data());
      SET_PTR(res, value);
      if (pop) {
        ovp.pop(rs->dtype);
        ++rs->i;
      }
      stream_last = !rs->stream_active();
    }
  }

  void Visit(ConstPortStream *cps) override {
    DSA_CHECK(cps->ls->hasNext());
    auto l1d = dynamic_cast<Linear1D*>(cps->ls);
    DSA_CHECK(l1d);
    stream_first = (l1d->i == 0);
    // Set index before popping data, or the index would be contaminated.
    SET_PTR(i, l1d->i);
    SET_PTR(res, cps->ls->poll(pop));
    stream_last = !l1d->hasNext();
  }

  void Visit(base_stream_t *bs) override {
    DSA_CHECK(false) << bs->toString() << " unsupported!";
  }

  /*!
   * \brief If we want to pop element.
   */
  bool pop;
  /*!
   * \breif The accelerator it belongs to.
   */
  accel_t *accel;
  /*!
   * \brief The result of ticking the stream.
   */
  int64_t *res{nullptr};
  /*!
   * \brief The index of ticking the stream.
   */
  int64_t *i{nullptr};
  /*!
   * \brief The state of the stream recurrence.
   */
  int8_t *state{nullptr};

  bool stream_first{false};

  bool stream_last{false};

  IFSMTicker(bool p, accel_t *a) : pop(p), accel{a} {}

 private:
  int64_t res_{0};
  int64_t i_;
  int8_t state_{0};

#undef SET_PTR
};

int IndirectFSM::hasNext(accel_t *accel) {
  if (!accel) {
    return true;
  }
  auto hasNextIndex = [accel] (base_stream_t *index, base_stream_t *value) -> int {
    DSA_CHECK(index->stream_active() == value->stream_active())
      << "Two streams should be sync!";
    if (!index->stream_active()) {
      return 0;
    }
    IFSMTicker it(false, accel); // Index ticker.
    index->Accept(&it);
    IFSMTicker vt(false, accel); // Value ticker.
    value->Accept(&vt);
    if (!it.res) {
      DSA_LOG(IFSM) << "Index Starving: " << index->toString();
      return -1;
    }
    if (!vt.res) {
      DSA_LOG(IFSM) << "Operand Starving: " << value->toString();
      return -1;
    }
    return 1;
  };
  auto renewStream = [] (int port, int64_t n, int dtype) -> base_stream_t * {
    base_stream_t *res;
    if (port == -1) {
      auto l1d = new Linear1D(1, 0, 0, n, 0);
      res = new ConstPortStream(0, {}, l1d);
    } else {
      res = new RecurrentStream(0, port, {}, n);
    }
    res->dtype = dtype;
    return res;
  };
  if (!idx().stream->stream_active()) {
    // Refresh idx stream
    DSA_CHECK(length().stream && array().stream);
    DSA_CHECK(length().stream->stream_active() == array().stream->stream_active());
    if (!length().stream->stream_active()) {
      return 0;
    }
    IFSMTicker lt(false, accel); // Length ticker.
    IFSMTicker at(false, accel); // Pointer ticker.
    length().stream->Accept(&lt);
    array().stream->Accept(&at);
    if (!lt.res) {
      DSA_LOG(IFSM) << "L1D Starving: " << length().stream->toString();
      return -1;
    }
    if (!at.res) {
      DSA_LOG(IFSM) << "Array Ptr Starving: " << array().stream->toString();
      return -1;
    }
    // If associate, the inner dimension is infinite.
    // TODO(@were): Support multi dimension.
    int64_t inner_n = associate ? std::numeric_limits<int64_t>::max() : *lt.res;
    idx().stream = renewStream(idx().port, inner_n, idx().dtype);
    value().stream = renewStream(value().port, inner_n, value().dtype);
    lt.pop = at.pop = true;
    length().stream->Accept(&lt);
    length().value = *lt.res;
    length().i = *lt.i;
    DSA_CHECK(lt.res && lt.i);
    array().stream->Accept(&at);
    array().value = *at.res;
    array().i = *at.i;
    DSA_CHECK(at.res && at.i);
    int res = hasNextIndex(idx().stream, value().stream);
    if (!res) {
      DSA_CHECK(length().stream->stream_active() == array().stream->stream_active());
      if (length().stream->stream_active()) {
        DSA_LOG(IFSM) << "Dimension Starving: " << idx().stream->toString();
        return -1;
      }
    }
    return res;
  }
  return hasNextIndex(idx().stream, value().stream);
}

std::vector<int64_t> IndirectFSM::poll(accel_t *accel, bool pop, AffineStatus &as) {
  DSA_CHECK(hasNext(accel) == 1) << "No value to pop now!";
  DSA_CHECK(idx().stream->stream_active() && value().stream->stream_active());
  IFSMTicker it(pop, accel);
  IFSMTicker vt(pop, accel);
  idx().stream->Accept(&it);
  idx().i = *it.i;
  idx().value = *it.res;
  value().stream->Accept(&vt);
  value().i = *vt.i;
  value().value = *vt.res;
  DSA_CHECK(it.res && vt.res);
  if (pop && it.state && ((*it.state >> 1) & 1)) {
    value().stream = idx().stream = new SentinelStream(false);
    DSA_LOG(IFSM) << "Kill associated stream!";
  }
  int64_t addr = (idx().i * stride1d + idx().value + array().value) * value().dtype + start_offset;
  DSA_LOG(IND_ADDR)
    << "(" << idx().i << " * " << stride1d << " + " << idx().value << " + "
    << array().value << ") * " << value().dtype << " + " << start_offset << " = " << addr;
  std::vector<int64_t> res{addr, *vt.res};
  if (penetrate) {
    DSA_CHECK(it.state);
    res.push_back(*it.state);
    DSA_LOG(IND_ADDR) << "State: " << (int) *it.state;
    as.stream_last = !hasNext(accel);
  } else {
    // TODO(@were): The stream first is not correct now, but I guess it is OK?
    as.dim_1st = (it.stream_first ? 1 : 0);
    as.dim_last = (it.stream_last ? 1 : 0);
    if (!hasNext(accel)) {
      as.dim_last = dimension;
    }
    as.stream_last = !hasNext(accel);
  }
  return res;
}

std::string IndirectFSM::toString(accel_t *accel) {
  if (!accel) {
    return "[An Accelerator Context Required]";
  }
  std::ostringstream oss;
  if (hasNext(accel)) {
    oss << "Indirect Stream FSM:\n";
    oss << "index:  " << idx().stream->toString() << "\n";
    oss << "length: " << length().stream->toString() << "\n";
    oss << "array:  " << array().stream->toString() << "\n";
  } else {
    oss << " retired indirect stream";
  }
  return oss.str();
}

uint8_t AffineStatus::toTag(bool packet_1st, bool packet_last) const {
  int res = 0;
  int mask_1st = (1 << dim_1st) - 1;
  int mask_last = (1 << dim_last) - 1;
  res = (res << 1) | (((mask_last & 2) >> 1) && packet_last); // L2D end, MSB
  res = (res << 1) | (((mask_1st & 2) >> 1) && packet_1st);   // L2D 1st
  res = (res << 1) | ((mask_last & 1) && packet_last);        // L1D end
  res = (res << 1) | ((mask_1st & 1) && packet_1st);          // L1D 1st
  res = (res << 1) | (stream_last && packet_last);            // Stream end
  res = (res << 1) | (stream_1st && packet_1st);              // Stream 1st, LSB
  DSA_LOG(TAG) << res;
  return res;
}

std::string AffineStatus::toString() const {
  std::ostringstream oss;
  oss
    << "dim 1st: " << dim_1st << ", dim last: " << dim_last
    << ", stream 1st: " << stream_1st << ", stream last: " << stream_last
    << ", bytes: " << n;
  return oss.str();
}

} // namespace stream
} // namespace sim
} // namespace dsa
