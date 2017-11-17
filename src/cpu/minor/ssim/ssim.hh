#ifndef __SOFTSIM_H__
#define __SOFTSIM_H__

#include <time.h>
#include <cstdint>
#include <iostream>
#include "accel.hh"

//Some utilities:
template<typename T, typename S, typename R> 
static void rolling_inc(T& which, S max, R reset) {
  which = ((which+1)==max) ? reset : which + 1; //rolling increment
}


class ssim_t 
{
  friend class ticker_t;
  friend class scratch_read_controller_t;
  friend class scratch_write_controller_t;
  friend class dma_controller_t;
  friend class port_port_controller_t;

public:
  //Simulator Interface
  ssim_t(Minor::LSQ* lsq);

  // Interface from instructions to streams
  // IF SB_TIMING, these just send the commands to the respective controllers
  // ELSE, they carry out all operations that are possible at that point
  void set_context(uint64_t context);
  void req_config(addr_t addr, int size);
  void load_dma_to_scratch(addr_t mem_addr, uint64_t stride, uint64_t acc_size,
      int stretch, uint64_t num_strides, addr_t scratch_addr, uint64_t flags);
  void write_dma_from_scratch(addr_t scratch_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t mem_addr, uint64_t flags); 
  void load_dma_to_port(addr_t mem_addr, uint64_t stride, 
      uint64_t access_size, int stretch, uint64_t num_strides, int port, int repeat_in);
  void load_scratch_to_port(addr_t scratch_addr, uint64_t stride, 
                            uint64_t access_size, int stretch, uint64_t num_strides, 
                            int in_port, int repeat_in); 
  void write_scratchpad(int out_port, addr_t scratch_addr, 
                        uint64_t num_bytes, uint64_t shift_bytes); 
  void write_dma(uint64_t garb_elem, 
      int out_port, uint64_t stride, uint64_t access_size, 
      uint64_t num_strides, addr_t mem_addr, int shift_bytes, int garbage);
  void reroute(int out_port, int in_port, uint64_t num_elem, 
               int repeat, uint64_t flags);
  void indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
    uint64_t num_elem, int repeat);
  void indirect_write(int ind_port, int ind_type, int out_port, 
    addr_t index_addr, uint64_t num_elem);
  bool can_receive(int out_port);
  uint64_t receive(int out_port);
  void write_constant(int num_strides, int in_port, 
                      SBDT constant, uint64_t num_elem, 
                      SBDT constant2, uint64_t num_elem2, 
                      uint64_t flags); 
  void insert_barrier(uint64_t mask);
  
  void print_stats();
  uint64_t forward_progress_cycle();
  bool can_add_stream();
  void add_bitmask_stream(base_stream_t* s);
  void add_bitmask_stream(base_stream_t* s, uint64_t context);
  bool done(bool show, int mask);
  bool is_in_config();

  void set_in_use() {_in_use=true;}
  void set_not_in_use() {_in_use=false;}
  bool in_use() {return _in_use;}

  uint64_t now();

  uint64_t elpased_time_in_roi() {return _elapsed_time_in_roi;}

  bool in_roi() {return _in_roi;}
  void roi_entry(bool enter);

  void timestamp(); //print timestamp
  void timestamp_index(int i); //print timestamp
  void timestamp_context(); //print timestamp

  void step();
  void cycle_shared_busses();


  Minor::MinorDynInstPtr cur_minst() {return _cur_minst;}

  void set_cur_minst(Minor::MinorDynInstPtr m) {
    assert(m);
    _cur_minst=m;
  }

  void issued_inst() {
    if(in_roi()) {
      _control_core_insts++;
    }
  }
  void wait_inst(uint64_t mask) {
    if(in_roi()) {
      _wait_map[mask]++;
    }
  }
  void wait_config() {
    if(in_roi()) {
      _config_waits++;
    }
  }

  static bool stall_core(uint64_t mask) {
    return (mask==0) || (mask&WAIT_CMP);
  }

  uint64_t roi_cycles() {return _roi_cycles;}
  uint64_t control_core_insts() {return _control_core_insts;}
  uint64_t config_waits() {return _config_waits;}

  accel_t* get_acc(int i) {
    assert(i>=0 && i<NUM_ACCEL_TOTAL);
    return accel_arr[i];
  }

  accel_t* shared_acc() { return accel_arr[SHARED_SP]; } 

  bool can_push_shs_buf(int size, uint64_t addr, uint64_t bitmask) {  
    for(uint64_t i=0,b=1; i < NUM_ACCEL_TOTAL; ++i, b<<=1) {
      if(bitmask & b) {
        if(!accel_arr[i]->scr_w_c()->_buf_shs_write.can_push_addr(size,addr)) {
          return false;
        }
      }
    }
    return true;
  }

private:

  unsigned _which_shr=0;

  Minor::MinorDynInstPtr _cur_minst;
  uint64_t _context_bitmask=1; //core 1 active
  uint64_t _ever_used_bitmask=1; //bitmask if core ever used

  Minor::LSQ* _lsq;

  accel_t* accel_arr[NUM_ACCEL+1]; //LAST ONE IS SHARED SCRATCH

  bool _prev_done = true;

  bool _in_use=false;

  bool debug=true;

  //Statistics Members
  bool _in_roi = false;
  timespec _start_ts, _stop_ts;
  uint64_t _elapsed_time_in_roi=0;
  uint64_t _times_roi_entered=0;
  uint64_t _orig_stat_start_cycle = 0;
  uint64_t _stat_start_cycle = 0;
  uint64_t _stat_stop_cycle = 0;
  uint64_t _roi_cycles=0;

  uint64_t _control_core_insts=0;
  uint64_t _config_waits=0;
  std::unordered_map<uint64_t,uint64_t> _wait_map;

};


#endif