#include "accel.hh"
#include "stream.hh"
#include "sim-debug.hh"

int base_stream_t::ID_SOURCE=0;

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

void base_stream_t::print_in_ports() {
  for(int i = 0; i < _in_ports.size();++i) {
    std::cout << soft_port_name(_in_ports[i], true) << " ";
  }
}

// based on memory mapping, extract these two information
uint64_t base_stream_t::get_core_id(addr_t cur_addr) {
  int part_bits = log2(_part_size);
  int assoc_core = (cur_addr >> part_bits) & _num_dist_cores; // aware of hw param?
  return _used_cores[assoc_core];
}

// PART_CORE_REST_BANK
addr_t base_stream_t::memory_map(addr_t logical_addr) {
  if(_part_size==0) return logical_addr;

  // put offset logic here
  int part_bits = log2(_part_size); // 4 (16 bytes)
  int core_bits = log2(_num_dist_cores);

  int part_offset = logical_addr & (_part_size-1);


  int part_id = logical_addr >> (part_bits+core_bits);


  int mapped_local_scr_addr = part_offset + (part_id << part_bits);

  int core_id = (logical_addr >> part_bits) & (_num_dist_cores-1);
  
  // adding the importance of core
  mapped_local_scr_addr = SCRATCH_SIZE*core_id + mapped_local_scr_addr;

  if(SS_DEBUG::SHOW_CONFIG) {
    std::cout << "original addr: " << logical_addr << " extracted part_offset: " << part_offset << " part_id: " << part_id << " core_id: " << core_id << " mapped scr addr: " << mapped_local_scr_addr << "\n";
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
