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
      CHECK(x >= begin && x <= end);   \
    }                                  \
  } while (false)

bool BuffetEntry::EnforceReadWrite(int64_t addr, int word) {
  if (mo == MemoryOperation::DMO_Read) {
    return InRange(addr);
  } else if (mo == MemoryOperation::DMO_Write) {
    return addr >= address && addr < address + Size();
  }
  CHECK(false) << "Not supported yet!";
  return false;
}

int64_t BuffetEntry::Translate(int64_t addr, MemoryOperation mo) {
  switch (mo) {
  case DMO_Write:
    CHECK(SpaceAvailable() && addr == address + occupied)
      << "For now only appended writing is supported!"
      << "request: " << addr << ", address: [" << address << ", " << address + occupied << ")"
      << ", Size: " << Size();
    break;
  case DMO_Read:
    CHECK(InRange(addr))
      << "Address out of range: " << addr << " not in ["
      << address << ", " << address + occupied << ")";
    break;
  default:
    CHECK(false) << "Not supported: " << mo;
  }
  CQ_PTR(addr, front - address);
  return addr;
}

void BuffetEntry::Append(int bytes) {
  CHECK(occupied + bytes <= Size())
    << "Buffet size overflow!" << occupied << " " << bytes << " " << Size();
  occupied += bytes;
  DSA_LOG(BUFFET) << "Append " << bytes << ", now: " << toString();
  CQ_PTR(tail, bytes);
}

void BuffetEntry::Shrink(int bytes) {
  CHECK(occupied - bytes >= 0)
    << "Buffet size underflow!" << occupied << " " << bytes << " " << Size();
  occupied -= bytes;
  DSA_LOG(BUFFET) << "Pop " << bytes << ", now: " << toString();
  address += bytes;
  CQ_PTR(front, bytes);
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
    CHECK(occupied == 0 || occupied == end - begin)
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
  oss << "Alloc: [" << begin << ", " << end << "), Ptr: ("
      << front << ", " << tail << "), Buffered: [" << address
      << ", " << address + occupied << ")";
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
  CHECK(iter != a.end());
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
  CHECK(hasNext());
  AffineStatus as;
  as.stream_1st = i == 0;
  as.dim_1st = i == 0 ? 1 : -1;
  std::vector<bool> mask(bandwidth, 0);
  CHECK(bandwidth == (bandwidth & -bandwidth));
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
    CHECK(current >= base);
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
        CHECK(current - base + j >= 0 && current - base + j < mask.size())
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
  as.dim_last = i == length ? 1 : -1;
  as.n = length;
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
  x##_ = y;           \
  x = &x##_;

  void Visit(RecurrentStream *rs) override {
    auto &ovp = accel->output_ports[rs->oports[0]];
    CHECK(rs->stream_active());
    if (!ovp.raw.empty()) {
      SET_PTR(i, rs->i);
      auto data = ovp.poll(rs->dtype);
      data.resize(8, 0);
      auto value = *reinterpret_cast<uint64_t*>(data.data());
      SET_PTR(res, value);
      if (pop) {
        ovp.pop(rs->dtype);
        ++rs->i;
      }
    }
  }

  void Visit(ConstPortStream *cps) override {
    CHECK(cps->ls->hasNext());
    auto l1d = dynamic_cast<Linear1D*>(cps->ls);
    CHECK(l1d);
    // Set index before popping data, or the index would be contaminated.
    SET_PTR(i, l1d->i);
    SET_PTR(res, cps->ls->poll(pop));
  }

  bool pop;
  accel_t *accel;
  int64_t *res{nullptr};
  int64_t *i{nullptr};

  IFSMTicker(bool p, accel_t *a) : pop(p), accel{a} {}

 private:
  int64_t res_{0};
  int64_t i_;

#undef SET_PTR
};

int IndirectFSM::hasNext(accel_t *accel) {
  int ctx = 1 << accel->accel_index();
  auto hasNextIndex = [accel] (base_stream_t *index, base_stream_t *value) -> int {
    CHECK(index->stream_active() == value->stream_active())
      << "Two streams should be sync!";
    if (!index->stream_active()) {
      return 0;
    }
    IFSMTicker it(false, accel); // Index ticker.
    index->Accept(&it);
    IFSMTicker vt(false, accel); // Value ticker.
    value->Accept(&vt);
    if (!it.res || !vt.res) {
      return -1;
    }
    return 1;
  };
  auto renewStream = [ctx] (int port, int n, int dtype) -> base_stream_t * {
    base_stream_t *res;
    if (port == -1) {
      auto l1d = new Linear1D(dtype, 0, 1, n, 0);
      res = new ConstPortStream(ctx, {}, l1d);
    } else {
      res = new RecurrentStream(ctx, port, {}, n);
    }
    res->dtype = dtype;
    return res;
  };
  if (!idx().stream->stream_active()) {
    // Refresh idx stream
    CHECK(length().stream && array().stream);
    if (!length().stream->stream_active()) {
      return 0;
    }
    IFSMTicker lt(false, accel); // Length ticker.
    IFSMTicker at(false, accel); // Pointer ticker.
    length().stream->Accept(&lt);
    array().stream->Accept(&at);
    if (lt.res && at.res) {
      idx().stream = renewStream(idx().port, *lt.res, idx().dtype);
      value().stream = renewStream(value().port, *lt.res, value().dtype);
      lt.pop = at.pop = true;
      length().stream->Accept(&lt);
      length().value = *lt.res;
      length().i = *lt.i;
      CHECK(lt.res && lt.i);
      array().stream->Accept(&at);
      array().value = *at.res;
      array().i = *at.i;
      CHECK(at.res && at.i);
      return hasNextIndex(idx().stream, value().stream);
    }
    return -1;
  }
  return hasNextIndex(idx().stream, value().stream);
}

std::vector<int64_t> IndirectFSM::poll(accel_t *accel, bool pop) {
  CHECK(hasNext(accel) == 1) << "No value to pop now!";
  CHECK(idx().stream->stream_active() && value().stream->stream_active());
  IFSMTicker it(pop, accel);
  IFSMTicker vt(pop, accel);
  idx().stream->Accept(&it);
  value().stream->Accept(&vt);
  CHECK(it.res && vt.res);
  return {*it.res * stride1d + array().value + start_offset, *vt.res};
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

}
}
}
