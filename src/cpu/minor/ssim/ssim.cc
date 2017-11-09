#include <unordered_set>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
 
#include "ssim.hh"
#include "cpu/minor/cpu.hh"

extern "C" void libsbsim_is_present() {}

using namespace std;

// Vector-Stream Commands (all of these are context-dependent)

ssim_t::ssim_t(Minor::LSQ* lsq) : _lsq(lsq) {
  SB_DEBUG::check_env();

  for(int i = 0; i < NUM_ACCEL; ++i) {
    accel_arr[i] = new accel_t(lsq, i, this);
  }
}
void ssim_t::req_config(addr_t addr, int size) {
  set_in_use();

  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_context_bitmask & b) {
      accel_arr[i]->req_config(addr,size);
    }
  }
}


bool ssim_t::can_add_stream() {
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i,b<<=1) {
    if(_context_bitmask & b) {
      if(!accel_arr[i]->can_add_stream()) {
        return false;
      }
    }
  }
  return true;
}

bool ssim_t::done(bool show, int mask) {
  bool is_done=true;

  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_context_bitmask & b) {
      is_done &= accel_arr[i]->done(show,mask);
    }
  }
  return is_done;
}

bool ssim_t::is_in_config() {
  bool in_config=true;

  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_context_bitmask & b) {
      in_config &= accel_arr[i]->is_in_config();
    }
  }
  return in_config;
}

void ssim_t::step() {
  if(!_in_use) {
    return;
  }
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      accel_arr[i]->tick();     
    }
  }
}

void ssim_t::print_stats() {
   auto& out = cout;
   out.precision(4);
   out << dec;

   out << "\n*** ROI STATISTICS ***\n";
   out << "Simulator Time: " << ((double)elpased_time_in_roi())
                                    /1000000000 << " sec\n";

   out << "Cycles: " << roi_cycles() << "\n";
   out << "Control Insts Issued: " << control_core_insts() << "\n";
   out << "Config Stalls: " << ((double)config_waits()/roi_cycles()) << "\n";
   out << "Wait Stalls:   ";
   for(auto& i : _wait_map) {
     out << ((double)i.second/roi_cycles()) << " (";
     uint64_t mask = i.first;
     if(mask == 0) {
       out << "ALL";
     }
     if(mask & WAIT_SCR_RD) {
       out << "SCR_RD";
     }
     if(mask & WAIT_SCR_WR) {
       out << "SCR_WR";
     }
     out << ")  ";
   }
   out << "\n";

  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      accel_arr[i]->print_stats();     
    }
  }
}

uint64_t ssim_t::forward_progress_cycle() {
  uint64_t r = 0;
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_ever_used_bitmask & b) {
      r = std::max(accel_arr[i]->forward_progress_cycle(),r);
    }
  }
  return r;
}

void ssim_t::add_bitmask_stream(base_stream_t* s, uint64_t ctx) {
  //if(debug && (SB_DEBUG::SB_COMMAND)  ) {
  //  cout << "Sending to Cores: ";
  //}

  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(ctx & b) {
      //if(debug && (SB_DEBUG::SB_COMMAND)  ) {
      //  cout << i;
      //}
      accel_arr[i]->add_stream(s);
    }
  }

  //if(debug && (SB_DEBUG::SB_COMMAND)  ) {
  //  cout << "\n";
  //}
}

void ssim_t::add_bitmask_stream(base_stream_t* s) {
  add_bitmask_stream(s,_context_bitmask);
}


// ------------------------Time Stuffs-------------------------------------
void ssim_t::timestamp() {
  cout << std::dec << now() - _stat_start_cycle << "\t";
}

void ssim_t::timestamp_context() {
  timestamp();
  cout << "context:" << std::hex << _context_bitmask << "\t" << std::dec;
}


// ----------------------- STREAM COMMANDS ------------------------------------
void ssim_t::set_context(uint64_t context) {
  if(debug && (SB_DEBUG::SB_CONTEXT)  ) {
    cout << "Context: " << std::hex << context << std::dec << "\n";
  }

  _context_bitmask = context;
  _ever_used_bitmask |= context;
}

void ssim_t::load_dma_to_scratch(addr_t mem_addr, 
    uint64_t stride, uint64_t access_size, uint64_t num_strides,
    addr_t scratch_addr) {
  dma_scr_stream_t* s = new dma_scr_stream_t();
  s->_mem_addr=mem_addr;
  s->_scratch_addr=scratch_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->set_orig();

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    delete s;
    return;
  }

  //WAIT_FOR(can_add_dma_scr_stream,1500);
  //add_dma_scr_stream(s);
  add_bitmask_stream(s);
}

void ssim_t::write_dma_from_scratch(addr_t scratch_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t mem_addr) {
  scr_dma_stream_t* s = new scr_dma_stream_t();
  s->_mem_addr=scratch_addr; //don't worry this is correct
  s->_dest_addr=mem_addr;    //... this too
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->set_orig();

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    delete s;
    return;
  }

  add_bitmask_stream(s);
}

void ssim_t::load_dma_to_port(addr_t mem_addr,
     uint64_t stride, uint64_t access_size, uint64_t num_strides,
     int in_port, int repeat) {
  dma_port_stream_t* s = new dma_port_stream_t();
  s->_mem_addr=mem_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_in_port=in_port;
  s->_repeat_in=repeat;
  s->set_orig();

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0 || repeat == 0) {
    delete s;
    return;
  }

  add_bitmask_stream(s);
}

void ssim_t::write_dma(uint64_t garb_elem, int out_port,
    uint64_t stride, uint64_t access_size, uint64_t num_strides,
    addr_t mem_addr, int shift_bytes, int garbage) {
  port_dma_stream_t* s = new port_dma_stream_t();
  s->_mem_addr=mem_addr;
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_out_port=out_port;
  s->_shift_bytes=shift_bytes;
  s->_garbage=garbage;
  s->set_orig();

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0) {
    delete s;
    return;
  }

  add_bitmask_stream(s);
}

void ssim_t::load_scratch_to_port(addr_t scratch_addr,
  uint64_t stride, uint64_t access_size, uint64_t num_strides,
  int in_port, int repeat) {
  scr_port_stream_t* s = new scr_port_stream_t();
  s->_mem_addr=scratch_addr; //NOTE: Here _mem_addr *is* the scratchpad address
  s->_num_strides=num_strides;
  s->_stride=stride;
  s->_access_size=access_size;
  s->_in_port=in_port;
  s->_repeat_in=repeat;
  s->set_orig();

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_strides==0 || access_size==0 || repeat == 0) {
    delete s;
    return;
  }

  //WAIT_FOR(can_add_scr_port_stream,1500);

  //_outstanding_scr_read_streams++;

  //add_scr_port_stream(s);
  add_bitmask_stream(s);
}

void ssim_t::write_scratchpad(int out_port, 
    addr_t scratch_addr, uint64_t num_bytes, uint64_t shift_bytes) {
  
  port_scr_stream_t* s = new port_scr_stream_t();
  s->_out_port=out_port;
  s->_scratch_addr=scratch_addr;
  s->_num_bytes=num_bytes;
  s->_shift_bytes=shift_bytes;
  s->set_orig();

  if(debug && (SB_DEBUG::SB_COMMAND || SB_DEBUG::SCR_BARRIER)  ) {
    timestamp_context();
    s->print_status(); 
  }

  if(num_bytes==0) {
    delete s;
    return;
  }

  //WAIT_FOR(can_add_port_scr_stream,1500);

  //add_port_scr_stream(s);
  add_bitmask_stream(s);

}

//The reroute function handles either local recurrence, or remote data transfer
void ssim_t::reroute(int out_port, int in_port, uint64_t num_elem, int repeat, 
                     uint64_t flags) {
  base_stream_t* s=NULL, *r=NULL;


  int core_d = (flags==1) ? -1 : 1;


  if(flags == 0) {
    s = new port_port_stream_t(out_port,in_port,num_elem,repeat);
  } else {
    auto s1 = new remote_port_stream_t(out_port,in_port,num_elem,repeat,core_d,true);
    auto r1 = new remote_port_stream_t(out_port,in_port,num_elem,repeat,core_d,false);
    s1->_remote_stream=r1; // tie together <3
    r1->_remote_stream=s1;
    s=s1;
    r=r1;
  }

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status();
  }

  if(num_elem==0 || repeat==0) {
    delete s;
    if(r) delete r;
    return;
  }

  add_bitmask_stream(s);
  if(r) {
    uint64_t new_ctx=0;
    if(core_d==1) {
      new_ctx = (_context_bitmask<<1) & ACCEL_MASK;
    } else if(core_d==-1) {
      new_ctx = (_context_bitmask>>1) & ACCEL_MASK;
    } else {
      assert(0 && "how do i do this?");
    }
    add_bitmask_stream(r, new_ctx);  
  }


}

//Configure an indirect stream with params
void ssim_t::indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
    uint64_t num_elem, int repeat) {
  indirect_stream_t* s = new indirect_stream_t();
  s->_ind_port=ind_port;
  s->_type=ind_type;
  s->_in_port=in_port;
  s->_index_addr=index_addr;
  s->_index_in_word=0;
  s->_num_elements=num_elem;
  s->_repeat_in=repeat;
  s->set_orig();

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status();
  }

  if(num_elem==0 || repeat == 0) {
    delete s;
    return;
  }

  add_bitmask_stream(s);
}

//Configure an indirect stream with params
void ssim_t::indirect_write(int ind_port, int ind_type, int out_port, 
    addr_t index_addr, uint64_t num_elem) {
  indirect_wr_stream_t* s = new indirect_wr_stream_t();
  s->_ind_port=ind_port;
  s->_type=ind_type;
  s->_out_port=out_port; //comes from sb_dma_addr
  s->_index_addr=index_addr;
  s->_index_in_word=0;
  s->_num_elements=num_elem;
  s->set_orig();

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status();
  }

  add_bitmask_stream(s);
}

//These two functions just return the first core from 0
bool ssim_t::can_receive(int out_port) {
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_context_bitmask & b) {
      return accel_arr[i]->can_receive(out_port); 
    }
  }
  return false;
  assert(0 && "has to be an accel active");
}

uint64_t ssim_t::receive(int out_port) {
  for(uint64_t i=0,b=1; i < NUM_ACCEL; ++i, b<<=1) {
    if(_context_bitmask & b) {
      return accel_arr[i]->receive(out_port); 
    }
  }
  assert(0 && "has to be an accel active");
  return 0;
}

void ssim_t::write_constant(int num_strides, int in_port, 
                    SBDT constant, uint64_t num_elem, 
                    SBDT constant2, uint64_t num_elem2, 
                    uint64_t flags) { //new

  const_port_stream_t* s = new const_port_stream_t();
  s->_in_port=in_port;

  if((flags&1)==0) {
    s->_constant=0;
    s->_num_elements=0;
    s->_num_iters=1;
  } else {
    s->_constant=constant2;
    s->_num_elements=num_elem2;
    s->_num_iters=num_strides;
  }

  s->_constant2=constant;
  s->_num_elements2=num_elem;

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status();
  }


  s->set_orig();

  if(debug && SB_DEBUG::SB_COMMAND) {
    timestamp_context();
    s->print_status();
  }

  if(num_elem==0) {
    delete s;
    return;
  }

  //WAIT_FOR(can_add_const_port_stream,1500);

  //add_const_port_stream(s);
  add_bitmask_stream(s);

}

uint64_t ssim_t::now() {
  return _lsq->get_cpu().curCycle();
}

// ------------------------- TIMING ---------------------------------
void ssim_t::roi_entry(bool enter) {
  if(enter) {
    _stat_start_cycle=now();
    clock_gettime(CLOCK_REALTIME,&_start_ts);
    _in_roi=true;
    _times_roi_entered+=1;
  } else {
    _stat_stop_cycle=now();
    clock_gettime(CLOCK_REALTIME,&_stop_ts);
    _elapsed_time_in_roi += 1000000000 * (_stop_ts.tv_sec - _start_ts.tv_sec) +
                                          _stop_ts.tv_nsec - _start_ts.tv_nsec;

    _roi_cycles += _stat_stop_cycle - _stat_start_cycle;
    _in_roi=false;
  }
}


