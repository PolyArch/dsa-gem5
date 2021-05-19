#include "accel.hh"
#include "stream.hh"
#include "sim-debug.hh"

const char *LOC_NAME[] = {
#define MACRO(x) #x
#include "loc.def"
#undef MACRO
};

int base_stream_t::ID_SOURCE = 0;

void base_stream_t::set_empty(bool b) {
  assert(b && "only goes one way for now");
  assert(!_empty && "can only empty once");

  if(SS_DEBUG::COMMAND_O) {
    if(b == true) {
      // timestamp();
      std::cout << "COMPLETED: "; 
    } 
    print_status();
  }

  _empty=b;
}

int BuffetEntry::Translate(int64_t addr, bool linebase) {
  CHECK(linebase || InRange(addr))
    << "Address out of range: " << addr << " not in ["
    << address << ", " << addr + occupied << ")";
  addr -= address;
  addr += front;
  if (addr >= end) {
    addr = begin + addr - end;
  }
  return addr;
}

#define CQ_PTR(x, delta)               \
  do {                                 \
    x += (delta);                      \
    if (x >= end) {                    \
      x = begin + (x) - end;           \
      CHECK(x >= begin && x <= end);   \
    }                                  \
  } while (false)

void BuffetEntry::Append(int bytes) {
  CHECK(occupied + bytes <= Size())
    << "Buffet size overflow!" << occupied << " " << bytes << " " << Size();
  occupied += bytes;
  CQ_PTR(tail, bytes);
}

void BuffetEntry::Shrink(int bytes) {
  CHECK(occupied - bytes >= 0)
    << "Buffet size underflow!" << occupied << " " << bytes << " " << Size();
  occupied -= bytes;
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

void IPortStream::print_in_ports() {
  for(int i = 0; i < pes.size();++i) {
    std::cout << soft_port_name(pes[i].port, true) << " ";
  }
}

std::string BuffetEntry::toString() {
  std::ostringstream oss;
  oss << "Alloc: [" << begin << ", " << end << "), Buffered: ["
      << front << ", " << tail << "), Occupied: " << occupied
      << ", Address: " << address;
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

  if(SS_DEBUG::SHOW_CONFIG) {
    std::cout << "original addr: " << logical_addr << " scr offset: " << cur_scr_offset << " extracted part_offset: " << part_offset << " part_id: " << part_id << " core_id: " << core_id << " mapped scr addr: " << mapped_local_scr_addr << "\n";
  }

  // int core_id = logical_addr >> part_bits & (N_CORES-1) + start_core + core_dist;
  // int part_id = addr >> (part_bits+log(N_CORES);
  
  return mapped_local_scr_addr;
  /*if (map_type == 0) {
    int bank_id = local_scr_addr >> (part_bits-bank_bits) & (NUM_BANKS-1);
  } else {
    int bank_id = local_scr_addr & (NUM_BANKS-1);
  }*/

}

std::string base_stream_t::soft_port_name(int x, bool is_input) {
  std::ostringstream oss;
  oss << x;
  auto &vec = is_input ? _soft_config->in_ports_name : _soft_config->out_ports_name;
  if (_soft_config && x < vec.size()) {
    oss << "(" << vec[x] << ")";
  }
  return oss.str();
}

namespace dsa {
namespace sim {
namespace stream {

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

BarrierFlag Loc2BarrierFlag(LOC loc) {
  return loc == LOC::DMA ? DBF_DMAStreams : DBF_SPadStreams;
}

}
}
}