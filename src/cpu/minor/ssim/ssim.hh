#ifndef __SOFTSIM_H__
#define __SOFTSIM_H__

#include <time.h>
#include <cstdint>
#include <iostream>
#include "accel.hh"

#define NUM_ACCEL 4

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
  void load_dma_to_scratch(addr_t mem_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t scratch_addr);
  void write_dma_from_scratch(addr_t scratch_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t mem_addr); //new
  void load_dma_to_port(addr_t mem_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, int port, int repeat_in);
  void load_scratch_to_port(addr_t scratch_addr, uint64_t stride, 
                            uint64_t access_size, uint64_t num_strides, 
                            int in_port, int repeat_in); //*
  void write_scratchpad(int out_port, addr_t scratch_addr, 
                        uint64_t num_bytes, uint64_t shift_bytes); //new
  void write_dma(uint64_t garb_elem, //new
      int out_port, uint64_t stride, uint64_t access_size, 
      uint64_t num_strides, addr_t mem_addr, int shift_bytes, int garbage);
  void reroute(int out_port, int in_port, uint64_t num_elem, int repeat);
  void indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
    uint64_t num_elem, int repeat);
  void indirect_write(int ind_port, int ind_type, int out_port, 
    addr_t index_addr, uint64_t num_elem);
  bool can_receive(int out_port);
  uint64_t receive(int out_port);
  void write_constant(int num_strides, int in_port, 
                      SBDT constant, uint64_t num_elem, 
                      SBDT constant2, uint64_t num_elem2, 
                      uint64_t flags); //new

  void print_stats();
  uint64_t forward_progress_cycle();
  bool can_add_stream();
  void add_bitmask_stream(base_stream_t* s);
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
  void timestamp_context(); //print timestamp

  void step();

  Minor::MinorDynInstPtr cur_minst() {return _cur_minst;}

  void set_cur_minst(Minor::MinorDynInstPtr m) {
    assert(m);
    _cur_minst=m;
  }

  uint64_t roi_cycles() {return _roi_cycles;}

private:

  Minor::MinorDynInstPtr _cur_minst;
  uint64_t _context_bitmask=1; //core 1 active
  uint64_t _ever_used_bitmask=1; //bitmask if core ever used

  Minor::LSQ* _lsq;

  accel_t* accel_arr[NUM_ACCEL];

  bool _prev_done = true;

  bool _in_use=false;

  bool debug=true;

  //Statistics Members
  bool _in_roi = true;
  timespec _start_ts, _stop_ts;
  uint64_t _elapsed_time_in_roi=0;
  uint64_t _times_roi_entered=0;
  uint64_t _stat_start_cycle = 0;
  uint64_t _stat_stop_cycle = 0;
  uint64_t _roi_cycles=0;

};


#endif
