#ifndef __ACCEL_H__
#define __ACCEL_H__

#include <model.h>
#include <schedule.h>
#include <vector>
#include <deque>
#include <map>
#include <unordered_map>

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <algorithm>
#include <fstream>
#include <queue>
#include <list>
#include <sstream>
#include <utility>
#include <iomanip>
#include <memory>

#include "sim-debug.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/lsq.hh"
#include "stream.hh"
#include "consts.hh"

using namespace SS_CONFIG;

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return (h1 ^ h2) + h1;  
    }
};


//forward decls
class ssim_t;
class accel_t;

//configuration
class soft_config_t {
public:
  uint64_t cur_config_addr;

  std::vector<std::string> in_ports_name;
  std::vector<std::string> out_ports_name;

  std::vector<int> in_ports_active;
  std::vector<int> in_ports_active_backcgra; // which inputs triggered by backcgra

  std::vector<int> out_ports_active;

  //In ports active by group
  std::vector<std::vector<int>> in_ports_active_group;
  std::vector<std::vector<int>> out_ports_active_group;

  std::vector<std::pair<int,int>> group_thr;

  std::vector<int> in_ports_active_plus; //includes indirect ports as well
  std::vector<int> out_ports_active_plus;

  std::vector<int> in_port_delay; //delay ports -- no longer used

  std::vector<int> out_ports_lat;

  std::vector<bool> cgra_in_ports_active;

  //input dfg nodes for [group][vec][port]
  std::vector<std::vector<std::vector<SSDfgInput*>>>  input_dfg_node;
  std::vector<std::vector<std::vector<SSDfgOutput*>>> output_dfg_node;

  std::map<SS_CONFIG::ss_inst_t,int> inst_histo;

  void reset();
};

// -------------- Vector Ports ------------------------

// "Wide" or interface port (AKA Vector Ports)
class port_data_t {
public:
  enum class STATUS {FREE, COMPLETE, BUSY};

  void initialize(SSModel* ssconfig, int port, bool isInput);
  void reset(); //reset if configuration happens
  unsigned port_cgra_elem() {
    int pm_size = _port_map.size();
    return std::max(pm_size,1);
  } //num of pairs in mapping 
  
  unsigned port_vec_elem(); //total size elements of the std::vector port 
  unsigned port_depth() { return port_vec_elem() / port_cgra_elem();} //depth of queue
  unsigned cgra_port_for_index(unsigned i) { return _port_map[i].first;}

  //reset all data
  void reset_data() {
    _mem_data.clear();
    _status = STATUS::FREE;
    _repeat=1, _repeat_stretch=0;
    _cur_repeat_lim=1;
    _num_times_repeated=0;
    _outstanding=0;
    _loc=LOC::NONE;
    for(int i = 0; i < _cgra_data.size(); ++i) {
      _cgra_data[i].clear();
      _cgra_valid[i].clear();
    }
    _num_in_flight=0; 
    _num_ready=0;
    _total_pushed=0;
  }

  float instances_ready() {
    return _num_ready + _mem_data.size() / (float) port_cgra_elem();
  }

  bool can_push_vp(int size) {
    return size <= num_can_push();
  }

  int num_can_push() {
    if(_mem_data.size() < VP_LEN) {
      return VP_LEN-_mem_data.size();
    } else {
      return 0;
    }
  }

  // Fill in remaining values in a vector port, if mode is appropriate
  // This should only be called at the begining/end of a stream's data
  void fill(uint32_t mode) {
    if(mode == NO_FILL) {
      return; //do nothing
    } else { //zero fill to width
      bool valid_flag = (mode!=STRIDE_DISCARD_FILL);

      unsigned width = port_cgra_elem();
      unsigned remainder = _mem_data.size() % width;
      if(remainder != 0) {
        unsigned extra = width - remainder;
        for(int i = 0; i < extra; ++i) {
          push_data(0,valid_flag);
        }
      }
    }
  }

  void push_mem_data(SBDT data) {
    _mem_data.push_back(data);
  }

  uint8_t _incomplete_word[8]={0};
  int _bytes_in_word=0;
  //We need an adaptor so that we can push sub-word size data to the ports
  //Ideally we'd change all the datastructures so they could work properly
  //for now I'm implementing a hack
  
  void push_data_byte(uint8_t b) {
     _incomplete_word[_bytes_in_word++]=b;
     if(_bytes_in_word==8) {
       _bytes_in_word=0;
       push_data( *((uint64_t*)_incomplete_word),true);
      for(int i = 0; i <8; ++i) _incomplete_word[i]=0; 
     }
  }

  void push_data(SBDT data, bool valid=true) {
    if(_bytes_in_word!=0) {
      std::cout << "It's not cool to leave random incomplete words in the port, "
           << "please be more tidy next time\n";
      assert(0);
    }
    _mem_data.push_back(data);
    _valid_data.push_back(valid); 
    _total_pushed+=valid;
  }


  void push_data(std::vector<SBDT> data, bool valid=true) { 
    for(auto i : data) push_data(i,valid); 
  }

  void reformat_in();  //rearrange all data for CGRA
  void reformat_in_one_vec();
  void reformat_in_work();

  void set_in_flight() {
    assert(_num_ready>0);
    _num_ready-=1;
    _num_in_flight+=1;
  }

  void set_out_complete() {
    assert(_num_in_flight>0);
    _num_in_flight-=1;
    _num_ready+=1;
  }

  bool can_output() {return _num_ready >= port_depth();}
  void reformat_out(); //rearrange all data from CGRA
  void reformat_out_one_vec();
  void reformat_out_work();

  int port() {return _port;}

  //Push one val into port
  void push_cgra_port(unsigned cgra_port, SBDT val, bool valid) {
    _cgra_data[cgra_port].push_back(val);
    _cgra_valid[cgra_port].push_back(valid);
  }

  void inc_ready(unsigned instances) {_num_ready+=instances;}

  //get the value of an instance in cgra port
  SBDT value_of(unsigned port_idx, unsigned instance) {
    return _cgra_data[port_idx][instance];
  }
  bool valid_of(unsigned port_idx, unsigned instance) {
    return _cgra_valid[port_idx][instance];
  }

  SBDT pop_in_data(); // pop one data from mem

  SBDT pop_out_data(); // pop one data from mem
  SBDT peek_out_data(); // peek one data from mem
  SBDT peek_out_data(int i); // peek one data from mem

  bool any_data() {
    if(_isInput) {
      return num_ready();
    } else {
      return mem_size();
    }
  }
  unsigned mem_size() {
      // std::cout << "came to calculate mem_size\n";
      return _mem_data.size();
  }
  unsigned num_ready() {return _num_ready;}         //Num of ready instances
  unsigned num_in_flight() {return _num_in_flight;}  //outputs in flight

  std::string status_string() {
    if(_status==STATUS::FREE) return "free";
    std::string ret;
    if(_loc==LOC::SCR)  ret = " (SCR)";
    if(_loc==LOC::PORT) ret = " (PORT)";
    if(_loc==LOC::DMA)  ret = " (DMA)";
    if(_status==STATUS::BUSY)     return "BUSY " + ret;
    if(_status==STATUS::COMPLETE) return "COMP " + ret;
    assert(0);
  }

  void set_status(STATUS status, LOC loc=LOC::NONE, uint32_t fill_mode=0) {
    if(SS_DEBUG::VP_SCORE) {
      std::cout << (_isInput ? "ip" : "op") << std::dec << _port;
      std::cout << " " << status_string();
      std::cout.flush();
    }

    if(_status == STATUS::BUSY) {
      assert(status != STATUS::BUSY && "can't set busy if already busy\n");
    }
    if(_status == STATUS::FREE) {
      if(status == STATUS::FREE  || status == STATUS::COMPLETE) {
        // print_status();
        assert(0 && "can't free if free or complete\n");
      }
    }
    if(status == STATUS::BUSY) {
      assert((_loc==loc || _status==STATUS::FREE) && 
          "can only assign to a port with eqiv. loc, or if the status is FREE");
    }
    
    if(status == STATUS::FREE) {
      //We are really just freeing one stream! So...
      //only enter truely free status if no outstanding streams
      _outstanding--;
      if(_outstanding==0){
        _status=STATUS::FREE;
      } 

      fill(fill_mode); //call fill 

    } else {
       _status=status; // no need 
      if(status == STATUS::BUSY) {
        _outstanding++;
        _loc=loc;
      }
    }

    if(SS_DEBUG::VP_SCORE) {
      std::cout << " -> " << status_string() << "\n";
    }
  }

  bool can_take(LOC loc, int repeat=1, int repeat_stretch=0) {
    //This makes sure the input port is fully drained before sending the next 
    //stream. This causes ~1-2 cycles of pipeline bubble + memory access time 
    //as well -- NOT GOOD.   Don't switch often the repeat size, especially
    //streams comming from memory!  (if this is required, add new h/w mechanism)
    if(_repeat != repeat || _repeat_stretch != repeat_stretch) { 
      return !in_use() && mem_size()==0 && (_cgra_data.size()==0 || 
                                            _cgra_data[0].size()==0);
    }
    return (_status == STATUS::FREE) || 
           (_status == STATUS::COMPLETE && _loc == loc);
  }
  bool in_use() {
    return !(_status == STATUS::FREE);
  }
  bool completed() {
    return _status == STATUS::COMPLETE;
  }
  LOC loc() {return _loc;}
  STATUS status() {return _status;}

  void pop(unsigned instances);  //Throw away data in CGRA input ports

  //NOTE:TODO:FIXME: Right now we only support wide maps, so pm is not really
  //necessary -- we construct our own pm from the mask -- maybe fix this later
  //when we are more ambitious
  void set_port_map(std::vector<int> pm, 
                    std::vector<bool> mask, int temporal) {
    assert(mask.size() == pm.size()); //masks should be the same!
    assert(mask.size() != 0);
    int total=0;
    for(unsigned i = 0; i < mask.size(); ++i) {
      if(mask[i]) {
        if(temporal) {
           for(unsigned i = 0; i < temporal; ++i) {
             std::vector<int> v;
            v.push_back(total);
             ++total;
            _port_map.push_back(std::make_pair(pm[i],v));
           }
        } else {
          std::vector<int> v;
          v.push_back(total);
          _port_map.push_back(std::make_pair(pm[i],v));
           ++total;
        }
      }
    }
    _cgra_data.resize(port_cgra_elem());
    _cgra_valid.resize(port_cgra_elem());
    assert(_cgra_data.size() > 0);
  }

  int repeat() {return _repeat;}

  void set_repeat(int r, int rs);
  
  // Increase the times of data repeated.
  // Return true if the value should be popped.
  // Or more accurately, repeat time runs out.
  bool inc_repeated();

  uint64_t total_pushed() { return _total_pushed; }

private:
  //Programmable Repeat:
  uint64_t _repeat=1, _repeat_stretch=0;
  int64_t _cur_repeat_lim=1;
  int64_t _num_times_repeated=0;

  // cgra_port: vec_loc(offset)
  // 23: 1, 2   24: 3, 4
  bool _isInput;
  int _port=-1;
  int _outstanding=0;
  STATUS _status=STATUS::FREE;
  LOC _loc=LOC::NONE;
  std::vector<std::pair<int, std::vector<int> > > _port_map;    //loc_map
  std::deque<SBDT> _mem_data;
  std::deque<bool> _valid_data;
  std::vector<std::deque<SBDT>> _cgra_data; //data per port
  std::vector<std::deque<bool>> _cgra_valid; //data per port
  unsigned _num_in_flight=0; 
  unsigned _num_ready=0;

  uint64_t _total_pushed=0;
};

//Entire Port interface with each being port_data_t
class port_interf_t {
public:

  void initialize(SSModel* ssconfig);
  void push_data(std::vector<SBDT>& data, int port) {
    for(SBDT i : data) {
      _in_port_data[port].push_data(i);
    }
  }

  void push_data(SBDT data, int port) {
    _in_port_data[port].push_data(data);
  }
  void reformat_in(int port) {
    _in_port_data[port].reformat_in();
  }

  port_data_t& in_port(int i) {return _in_port_data[i];}
  port_data_t& out_port(int i) {return _out_port_data[i];}

  void reset() {
    for(unsigned i = 0; i < _in_port_data.size(); ++i) {
      _in_port_data[i].reset();
    }
    for(unsigned i = 0; i < _out_port_data.size(); ++i) {
      _out_port_data[i].reset();
    }

  }

private:

  std::vector<port_data_t> _in_port_data;
  std::vector<port_data_t> _out_port_data;
};


//.............. Streams and Controllers .................

// Each controller forwards up to one "block" of data per cycle.
class data_controller_t {
  public:
  data_controller_t(accel_t* host) {
    _accel=host;
  }

protected:
  accel_t* _accel;
  ssim_t* get_ssim();
  bool is_shared();
  void timestamp();
  void add_bw(LOC l1, LOC l2, uint64_t times, uint64_t bytes);
};

class scratch_write_controller_t;
class scratch_read_controller_t;


//Limitations: 1 simultaneously active scratch stream
class dma_controller_t : public data_controller_t {
  friend class scratch_write_controller_t;
  friend class scratch_read_controller_t;

  public:
  static const int data_width=64; //data width in bytes
  //static const int data_ssdts=data_width/SBDT; //data width in bytes
  std::vector<bool> mask;

  dma_controller_t(accel_t* host,
      scratch_read_controller_t* scr_r_c,
      scratch_write_controller_t* scr_w_c) : 
    data_controller_t(host), _scr_r_c(scr_r_c), _scr_w_c(scr_w_c) {
    
    _prev_port_cycle.resize(64); //resize to maximum conceivable ports
    mask.resize(MEM_WIDTH/DATA_WIDTH);
  }


  void reset_stream_engines() {
    _read_streams.clear();
    _write_streams.clear();
    _dma_port_streams.clear();
    _indirect_streams.clear();
    _port_dma_streams.clear();
    _indirect_wr_streams.clear();
  }

  void reset_data() {
    reset_stream_engines();
    _mem_read_reqs=0;
    _mem_write_reqs=0;
    _fake_scratch_reqs=0;
  }

  void cycle();
  void finish_cycle();
  bool done(bool show, int mask);

  int req_read(affine_read_stream_t& stream);
  void req_write(affine_write_stream_t& stream, port_data_t& vp);

  void ind_read_req(indirect_stream_t& stream);
  void ind_write_req(indirect_wr_stream_t& stream);

  void make_read_request();
  void make_write_request();

  void print_status();
  void cycle_status();

  bool mem_reads_outstanding()  {return _mem_read_reqs;}
  bool mem_writes_outstanding() {return _mem_write_reqs;}
  bool scr_reqs_outstanding()  {return _fake_scratch_reqs;}

  bool dma_port_streams_active();
  bool port_dma_streams_active();
  bool indirect_streams_active();
  bool indirect_wr_streams_active();

  bool schedule_dma_port(affine_read_stream_t& s);
  bool schedule_port_dma(affine_write_stream_t& s);
  bool schedule_indirect(indirect_stream_t&s);
  bool schedule_indirect_wr(indirect_wr_stream_t&s);

  int mem_reqs() {return _mem_read_reqs + _mem_write_reqs;}

  scratch_read_controller_t*  scr_r_c() {return _scr_r_c;}
  scratch_write_controller_t* scr_w_c() {return _scr_w_c;}

  private:

  scratch_read_controller_t* _scr_r_c;
  scratch_write_controller_t* _scr_w_c;

  void port_resp(unsigned i);

  unsigned _which_rd=0, _which_wr=0;

  std::vector<base_stream_t*> _read_streams;
  std::vector<base_stream_t*> _write_streams;

  //This ordering defines convention of checking
  std::vector<affine_read_stream_t*> _dma_port_streams;  //reads
  std::vector<indirect_stream_t*> _indirect_streams; //indirect reads
  std::vector<affine_write_stream_t*> _port_dma_streams; //writes
  std::vector<indirect_wr_stream_t*> _indirect_wr_streams; //indirect writes

  void delete_stream(int i, affine_read_stream_t* s);
  void delete_stream(int i, indirect_stream_t* s);
  void delete_stream(int i, affine_write_stream_t* s);  
  void delete_stream(int i, indirect_wr_stream_t* s);
 
  //address to stream -> [stream_index, data]
  uint64_t _mem_read_reqs=0, _mem_write_reqs=0;
  std::vector<uint64_t> _prev_port_cycle;
  uint64_t _prev_scr_cycle=0;
  int _fake_scratch_reqs=0;

  //std::unordered_map<uint64_t, uint64_t> port_youngest_data;
};

class scratch_read_controller_t : public data_controller_t {
  public:
  std::vector <bool> mask;

  scratch_read_controller_t(accel_t* host, dma_controller_t* d) 
    : data_controller_t(host) {
    _dma_c=d; //save this for later

    mask.resize(SCR_WIDTH/DATA_WIDTH);

    //if(is_shared()) {
    //  _scr_scr_streams.resize(NUM_ACCEL);
    //} else {
    //  _scr_scr_streams.resize(1); 
    //}
    _indirect_scr_read_requests.resize(NUM_SCRATCH_BANKS);

    reset_stream_engines();
  }

  void reset_stream_engines() {
    _ind_port_streams.clear();
    _scr_port_streams.clear();
  }

  void reset_data() {
    reset_stream_engines();
  }

  std::vector<SBDT> read_scratch(affine_read_stream_t& stream);
  void read_scratch_ind(indirect_stream_t& stream, uint64_t scr_addr);

  float calc_min_port_ready();
  // void cycle();
  void cycle(bool &performed_read);
  // banked scratchpad read buffers
  bool indirect_scr_read_requests_active();

  int cycle_read_queue();

  void finish_cycle();
  bool done(bool,int);

  bool schedule_scr_port(affine_read_stream_t& s);
  bool schedule_indirect(indirect_stream_t& s);

  void print_status();
  void cycle_status();

  bool scr_port_streams_active();

  private:
  int _which_rd=0;

  struct ind_reorder_entry_t {
    uint8_t data[64]; //64 bytes per request
    int size; //number of writes that should be completed
    int completed=0;
    int data_bytes;
    base_stream_t* stream; 
    bool last = false;
  };

  struct indirect_scr_read_req{
    void *ptr;
    uint64_t addr;
    size_t bytes;
    ind_reorder_entry_t* reorder_entry=NULL;
  };

  std::vector<base_stream_t*> _read_streams;

  void delete_stream(int i, affine_read_stream_t* s);
  void delete_stream(int i, indirect_stream_t* s);

  std::vector<affine_read_stream_t*> _scr_port_streams;
  //std::vector<scr_scr_stream_t*> _scr_scr_streams;
  std::vector<indirect_stream_t*> _ind_port_streams;

  std::vector<std::queue<indirect_scr_read_req>> _indirect_scr_read_requests;
  std::queue<ind_reorder_entry_t*> _ind_ROB;

  dma_controller_t* _dma_c;  
};


class scratch_write_controller_t : public data_controller_t {
  public:
  std::vector<bool> mask;

  scratch_write_controller_t(accel_t* host, dma_controller_t* d) 
    : data_controller_t(host) {
    _dma_c=d;
    mask.resize(SCR_WIDTH/DATA_WIDTH);

    _atomic_scr_issued_requests.resize(NUM_SCRATCH_BANKS);

    //if(is_shared()) {
    //  _scr_scr_streams.resize(NUM_ACCEL); 
    //} else {
    //  _scr_scr_streams.resize(1); 
    //}
  }

  void write_scratch_ind(indirect_wr_stream_t& stream);

  void reset_stream_engines() {
    _port_scr_streams.clear();
    _const_scr_streams.clear();
    _atomic_scr_streams.clear();
    _ind_wr_streams.clear();
    _write_streams.clear();
  }

  void reset_data() {
    reset_stream_engines();
  }

  bool crosssar_backpressureOn();

  // void cycle();
  void cycle(bool can_perform_atomic_scr, bool &performed_atomic_scr);
  void finish_cycle();
  bool done(bool,int);

  bool schedule_atomic_scr_op(atomic_scr_stream_t& s);
  bool schedule_indirect_wr(indirect_wr_stream_t& s);

  void print_status();
  void cycle_status();

  // for atomic_stream
  bool atomic_scr_streams_active();
  // streams issued but not executed because of bank conflicts
  bool atomic_scr_issued_requests_active();

  bool port_scr_streams_active();
  bool const_scr_streams_active();

  bool schedule_port_scr(affine_write_stream_t& s);
  bool schedule_const_scr(const_scr_stream_t& s);

  private:
  int _which_wr=0;

  struct atomic_scr_op_req{
    addr_t _scr_addr;
    SBDT _inc;
    int _opcode;
    int _value_bytes;
  };

  void delete_stream(int i, affine_write_stream_t* s);
  void delete_stream(int i, const_scr_stream_t* s);
  void delete_stream(int i, atomic_scr_stream_t* s);
  void delete_stream(int i, indirect_wr_stream_t* s);

  std::vector<base_stream_t*> _write_streams;

  std::vector<affine_write_stream_t*> _port_scr_streams; 
  std::vector<const_scr_stream_t*> _const_scr_streams;  
  std::vector<atomic_scr_stream_t*> _atomic_scr_streams; 
  std::vector<indirect_wr_stream_t*> _ind_wr_streams; 

  // TODO: add request queue max size
  std::vector<std::queue<atomic_scr_op_req>> _atomic_scr_issued_requests;
  dma_controller_t* _dma_c;
};


class port_controller_t : public data_controller_t {
  public:
  port_controller_t(accel_t* host) : data_controller_t(host) {
    _port_port_streams.resize(8);  //IS THIS ENOUGH?
    _const_port_streams.resize(8);  //IS THIS ENOUGH?
    _remote_port_streams.resize(8);  //IS THIS ENOUGH?
    for(auto& i : _port_port_streams) {i.reset();}
    for(auto& i : _const_port_streams) {i.reset();}
    for(auto& i : _remote_port_streams) {i.reset();}
  }

  void cycle();
  void finish_cycle();
 
  //bool any_stream_active() {
  //  return comm_streams_active() || read_streams_active() 
  //    || write_streams_active();
  //}
  //bool comm_streams_active() {
  //  return false;
  //}
  //bool read_streams_active() {
  //  return port_port_streams_active() || const_port_streams_active();
  //}
  //bool write_streams_active() {
  //  return port_port_streams_active();
  //}

  bool port_port_streams_active();
  bool const_port_streams_active();
  bool done(bool,int);

  bool schedule_port_port(port_port_stream_t& s);
  bool schedule_const_port(const_port_stream_t& s);
  bool schedule_remote_port(remote_port_stream_t& s);

  void print_status();
  void cycle_status();

  private:
  unsigned _which_pp=0;
  unsigned _which_cp=0;
  unsigned _which_rp=0;

  std::vector<port_port_stream_t>   _port_port_streams;
  std::vector<const_port_stream_t>  _const_port_streams;
  std::vector<remote_port_stream_t> _remote_port_streams;
};




struct stream_stats_histo_t {
  uint64_t vol_by_type[(int)STR_PAT::LEN];
  uint64_t vol_by_len[64];
  std::unordered_map<uint64_t,uint64_t> vol_by_len_map;
  std::unordered_map<std::pair<int,int>,uint64_t,pair_hash> vol_by_source;
  uint64_t total_vol=0;
  uint64_t total=0;

  stream_stats_histo_t() {
    for(int i = 0; i < (int)STR_PAT::LEN; ++i) {vol_by_type[i]=0;}
    for(int i = 0; i < 64; ++i)     {vol_by_len[i]=0;}
  }

  void add(STR_PAT t, LOC src, LOC dest, uint64_t vol) {
    vol_by_source[std::make_pair((int)src,(int)dest)] += vol;
    vol_by_type[(int)t] += vol;
    vol_by_len[ilog2(vol)+1]+=vol;
    vol_by_len_map[vol]+=vol;
    total_vol+=vol;
    total++;
  }

  std::string name_of(STR_PAT t) {
    switch(t) {
      case STR_PAT::PURE_CONTIG: return "PURE_CONTIG";
      case STR_PAT::SIMPLE_REPEATED: return "REPEATED";
      case STR_PAT::SIMPLE_STRIDE: return "STRIDE";
      case STR_PAT::OVERLAP: return "OVERLAP";
      case STR_PAT::CONST: return "CONST";
      case STR_PAT::REC: return "REC";
      case STR_PAT::IND: return "IND";
      case STR_PAT::NONE: return "NONE";
      case STR_PAT::OTHER: return "NONE";
      default: return "UNDEF";
    }
    return "XXXXX";
  }

  void print(std::ostream& out) {
    out << std::setprecision(2);
    out << " by orig->dest:\n";
    for(auto i : vol_by_source) {
      out << base_stream_t::loc_name((LOC)(i.first.first)) << "->"
          << base_stream_t::loc_name((LOC)(i.first.second)) << ": ";
      out << ((double)i.second)/total_vol << "\n";
    }

    out << "   by pattern type:\n";
    for(int i = 0; i < (int)STR_PAT::LEN; ++i) {
      out << name_of((STR_PAT)i) << ": " 
          << ((double)vol_by_type[i])/total_vol << "\n";
    }
    int lowest=64, highest=0;
    for(int i = 0; i < 64; ++i) {
      if(vol_by_len[i]) {
        if(i < lowest) lowest=i;
        if(i > highest) highest=i;
      }
    }
    out << "    by len (log2 bins):\n";
    for(int i = 1,x=2; i < highest; ++i,x*=2) {
      out << i << ": " << vol_by_len[i] << "(" << x << " to " << x*2-1 << ")\n";
    }
  }

  

};

struct stream_stats_t { 
  stream_stats_histo_t reqs_histo;
  stream_stats_histo_t vol_histo;

  void add(STR_PAT t, LOC src, LOC dest, uint64_t vol, uint64_t reqs) {
    vol_histo.add(t,src,dest,vol);
    reqs_histo.add(t,src,dest,vol);
  }

  void print(std::ostream& out) {
    out << "Volume ";
    vol_histo.print(out);
  } 

};


struct pipeline_stats_t {
  enum PIPE_STATUS {CONFIG, ISSUED, ISSUED_MULTI, TEMPORAL_ONLY,
                    CONST_FILL, SCR_FILL, DMA_FILL, REC_WAIT,
                    CORE_WAIT, SCR_BAR_WAIT, DMA_WRITE, CMD_QUEUE, CGRA_BACK,  
                    DRAIN, NOT_IN_USE, LAST};

  static std::string name_of(PIPE_STATUS value) {
      const char* s = 0;
      #define PROCESS_VAL(p) case(p): s = #p; break;
      switch(value){
        PROCESS_VAL(CONFIG);
        PROCESS_VAL(ISSUED);
        PROCESS_VAL(ISSUED_MULTI);
        PROCESS_VAL(TEMPORAL_ONLY);
        PROCESS_VAL(CONST_FILL);
        PROCESS_VAL(SCR_FILL);
        PROCESS_VAL(DMA_FILL);
        PROCESS_VAL(REC_WAIT);
        PROCESS_VAL(CORE_WAIT);
        PROCESS_VAL(SCR_BAR_WAIT);
        PROCESS_VAL(DMA_WRITE);
        PROCESS_VAL(CMD_QUEUE);
        PROCESS_VAL(CGRA_BACK);
        PROCESS_VAL(DRAIN);
        PROCESS_VAL(NOT_IN_USE);
        case LAST: assert(0);
      }
      #undef PROCESS_VAL
      return std::string(s);
  }

  double pipe_stats[PIPE_STATUS::LAST] = { 0 };

  void pipe_inc(PIPE_STATUS p, double amount) {
    pipe_stats[p]+=amount;
  }

  void print_histo(std::ostream& out, uint64_t roi_cycles) {
    out.precision(4);
    out << std::fixed;

    uint64_t total=0;
    for(int i=0; i < PIPE_STATUS::LAST; ++i) {
      total+=pipe_stats[i];
    } 

    if(roi_cycles < total) {
      total=roi_cycles;
    }

    pipe_stats[NOT_IN_USE]=roi_cycles - total;

    for(int i=0; i < PIPE_STATUS::LAST; ++i) {
      out << name_of( (PIPE_STATUS)i) << ":" << 
        pipe_stats[i] / (double)roi_cycles << " ";
    }

    out << std::defaultfloat;
  }

};



class accel_t
{
  friend class ssim_t;
  friend class scratch_read_controller_t;
  friend class scratch_write_controller_t;
  friend class dma_controller_t;
  friend class port_port_controller_t;
 
public:

  accel_t(Minor::LSQ* lsq, int i, ssim_t* ssim);

  bool in_use();
  void timestamp(); //print timestamp
  uint64_t now(); //return's sim's current time

  //Stats Interface
  void print_stats();
  void pedantic_statistics(std::ostream&);
  void print_statistics(std::ostream&);
//  void reset_statistics();
  void print_status();

  void cycle_status();
  void clear_cycle();

  void request_reset_data();

  port_interf_t& port_interf() {
    return _port_interf;
  }


  bool done(bool show = false, int mask = 0);

  bool set_in_config() {return _in_config = true;}
  bool is_in_config() {return _in_config;}
  bool can_add_stream();

  void add_stream(std::shared_ptr<base_stream_t> s) {

    /*if (_ssconfig->dispatch_inorder()) {
      add_port_based_stream(s);
      return;
    }
    if(dma_scr_stream_t* s_d = dynamic_cast<dma_scr_stream_t*>(s)) {
      add_dma_scr_stream(s_d);
      return;
    }
    if(scr_dma_stream_t* s_d = dynamic_cast<scr_dma_stream_t*>(s)) {
      add_scr_dma_stream(s_d);
      return;
    }*/
    add_port_based_stream(s);
  }

  uint64_t forward_progress_cycle() { return _forward_progress_cycle; }

  Minor::MinorDynInstPtr cur_minst();

  void process_stream_stats(base_stream_t& s) {
    uint64_t    vol  = s.data_volume();
    uint64_t    reqs = s.requests();
    STR_PAT     t  = s.stream_pattern();
    _stream_stats.add(t,s.src(),s.dest(),vol,reqs);
  }

  void configure(addr_t addr, int size, uint64_t* bits);

  pipeline_stats_t::PIPE_STATUS whos_to_blame(int group);
  void whos_to_blame(std::vector<pipeline_stats_t::PIPE_STATUS>& blame_vec,
                     std::vector<pipeline_stats_t::PIPE_STATUS>& group_vec);
  void tick(); //Tick one time

  uint64_t roi_cycles();

  //New Stats
  void add_bw(LOC l1,LOC l2,uint64_t times, uint64_t bytes) {
    auto& p = _bw_map[std::make_pair(l1,l2)];
    p.first += times;
    p.second += bytes;

    //inneficient (can be done at the end), but improve later : )
    auto& ps = _bw_map[std::make_pair(l1,LOC::TOTAL)];
    ps.first += times;
    ps.second += bytes;

    auto& pr = _bw_map[std::make_pair(LOC::TOTAL,l2)];
    pr.first += times;
    pr.second += bytes;

    //TODO: FIXME: need to check this logic
    if(l2 == LOC::PORT && (l1 == LOC::CONST || l1 == LOC::PORT)) {
      auto& pr = _bw_map[std::make_pair(l1,LOC::REC_BUS)];
      pr.first += times;
      pr.second += bytes;

      auto& pt = _bw_map[std::make_pair(LOC::TOTAL,LOC::REC_BUS)];
      pt.first += times;
      pt.second += bytes;

    }
  }

  ssim_t* get_ssim() {return _ssim;}
  bool is_shared() {return _accel_index==SHARED_SP;}
  int accel_index() {return _accel_index;}

  scratch_read_controller_t*  scr_r_c() {return &_scr_r_c;}
  scratch_write_controller_t* scr_w_c() {return &_scr_w_c;}

private:
  ssim_t* _ssim;
  Minor::LSQ* _lsq;
  std::ofstream in_port_verif, out_port_verif, scr_wr_verif, scr_rd_verif, cmd_verif;
  std::ofstream cgra_multi_verif;
  bool _cleanup_mode=false;

  std::ostream* _cgra_dbg_stream=NULL;

  //softsim_interf_t* _sim_interf;
  //SBDT ld_mem8(addr_t addr,uint64_t& cycle, Minor::MinorDynInstPtr m) 
  //  {return _sim_interf->ld_mem8(addr,cycle,m);}
  //SBDT ld_mem(addr_t addr, uint64_t& cycle, Minor::MinorDynInstPtr m)  
  //  {return _sim_interf->ld_mem(addr,cycle,m);}
  //void st_mem(addr_t addr, SBDT val, uint64_t& cycle, Minor::MinorDynInstPtr m) 
  //  {_sim_interf->st_mem(addr,val,cycle,m);}
  //void st_mem16(addr_t addr, SBDT val, uint64_t& cycle, Minor::MinorDynInstPtr m) 

  //***timing-related code***
  bool done_internal(bool show, int mask);
  bool done_concurrent(bool show, int mask);

  void cycle_cgra();   //Tick on each cycle
  void cycle_cgra_backpressure();  
  void cycle_cgra_fixedtiming();   

  void reset_data(); //carry out the work 

  void cycle_in_interf();
  void cycle_out_interf();
  void cycle_indirect_interf(); //forward from indirect inputs to indirect outputs
  void schedule_streams();

  bool cgra_done(bool, int mask);
  bool cgra_input_active() {
    for(unsigned i = 0; i < _soft_config.in_ports_active_plus.size(); ++i) {
      int cur_port = _soft_config.in_ports_active_plus[i];
      auto& in_vp = _port_interf.in_port(cur_port);
      if(in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size()) { 
        return true;
      }
    } 
    return false;
  }

  bool cgra_compute_active() {
    for(unsigned i = 0; i < _soft_config.out_ports_active.size(); ++i) {
      int cur_port = _soft_config.out_ports_active[i];
      auto& out_vp = _port_interf.out_port(cur_port);
        if(out_vp.num_in_flight()) {
          return true;
        }
    }
    return false;
  }

  bool cgra_output_active() {
    for(unsigned i = 0; i < _soft_config.out_ports_active_plus.size(); ++i) {
      int cur_port = _soft_config.out_ports_active_plus[i];
      auto& out_vp = _port_interf.out_port(cur_port);
      if(out_vp.in_use() || out_vp.num_ready() || out_vp.mem_size()) { 
        return true; 
      }
    }
    return false;
  }

  bool in_roi();

  bool can_receive(int out_port);
  uint64_t receive(int out_port);

  void verif_cmd(base_stream_t* s) {
    if(SS_DEBUG::VERIF_CMD) {
      cmd_verif << s->short_name();
      cmd_verif << s->mem_addr()     << " ";   
      cmd_verif << s->access_size()  << " ";   
      cmd_verif << s->stride()       << " ";   
      cmd_verif << s->scratch_addr() << " ";   
      cmd_verif << s->num_strides()  << " ";   
      cmd_verif << s->num_bytes()    << " ";   
      cmd_verif << s->constant()     << " ";   
      cmd_verif << s->first_in_port()      << " ";   
      cmd_verif << s->out_port()     << " ";   
      cmd_verif << s->wait_mask()    << " ";   
      cmd_verif << s->shift_bytes()  << "\n";
    }
  }

  void req_config(addr_t addr, int size, uint64_t context);

  void sanity_check_stream(base_stream_t* s);

  void add_port_based_stream(std::shared_ptr<base_stream_t> s) {
    sanity_check_stream(s.get());
    assert(cur_minst());
    s->set_soft_config(&_soft_config);
    s->set_minst(cur_minst());
    _cmd_queue.push_back(s);
    forward_progress();
    verif_cmd(s.get());
  }

  void do_cgra();
  void execute_dfg(unsigned instance, int group);

  void forward_progress() {
    _waiting_cycles=0; 
    _forward_progress_cycle=now();
  }


  void read_scratchpad(void* dest, uint64_t scr_addr, 
      std::size_t count, int id) {
    assert(scr_addr < SCRATCH_SIZE);

    std::memcpy(dest, &scratchpad[scr_addr], count);
    if(SS_DEBUG::CHECK_SCR_ALIAS) { //TODO: make this check work for unaligned
      for(int i = 0; i < count; i+=sizeof(SBDT)) {
        uint64_t running_addr=  scr_addr+i;
        if(scratchpad_writers[running_addr]) {
          std::cout << "WARNING: scr_addr: " << running_addr
                    << " constistency is potentially violated; "
                    << " writer_id: " << scratchpad_writers[running_addr]
                    << " reader_id: " << id << "\n";
        }
        scratchpad_readers[running_addr]=id;
      }
    }
  }

  void write_scratchpad(uint64_t scr_addr, const void* src, 
      std::size_t count, int id) {
    std::memcpy(&scratchpad[scr_addr], src, count);
    if(SS_DEBUG::CHECK_SCR_ALIAS) {
      for(int i = 0; i < count; i+=sizeof(SBDT)) {
        uint64_t running_addr = scr_addr+i;
        if(scratchpad_writers[running_addr]) {
          std::cout << "WARNING: scr_addr: " << running_addr
                    << " constistency is potentially violated; "
                    << " writer_id: " << scratchpad_writers[running_addr] 
                    << " writer_id: " << id << "\n";
        }
        if(scratchpad_readers[running_addr]) {
          std::cout << "WARNING: scr_addr: " << running_addr
                    << " constistency is potentially violated; "
                    << " reader_id: " << scratchpad_readers[running_addr]
                    << " writer_id: " << id << "\n";
        }
        scratchpad_writers[running_addr]=id;
      }
    }
  }


  //members------------------------
  soft_config_t _soft_config;
  port_interf_t _port_interf;

  bool _in_config=false;

  SSModel* _ssconfig = NULL;
  Schedule* _sched   = NULL;
  SSDfg*    _dfg     = NULL;

  int _fu_fifo_len=DEFAULT_FIFO_LEN;
  int _ind_rob_size=DEFAULT_IND_ROB_SIZE;


  std::vector<uint8_t> scratchpad;    
  std::bitset<SCRATCH_SIZE/SCR_WIDTH> scratch_ready; //TODO: use this

  unsigned scratch_line_size = 16;                //16B line 
  unsigned fifo_depth = 32;  
  bool debug;
  unsigned _queue_size=16;

  uint64_t _accel_index = 0;
  uint64_t _accel_mask = 0;

  // Controllers
  dma_controller_t _dma_c;
  scratch_read_controller_t _scr_r_c;
  scratch_write_controller_t _scr_w_c;
  port_controller_t _port_c;

  std::list<std::shared_ptr<base_stream_t>> _cmd_queue;

  std::map<uint64_t,std::vector<int>> _cgra_output_ready;

  //Stuff for tracking stats
  uint64_t _waiting_cycles=0;
  uint64_t _forward_progress_cycle=0;

  public:
  //* running variables
  bool _cgra_issued_group[NUM_GROUPS];
  int _cgra_issued;
  int _dedicated_cgra_issued;
  int _backcgra_issued;
  int _scr_ctrl_turn = 0;

  std::vector<bool> _cgra_prev_issued_group[NUM_GROUPS];
  //uint64_t _delay_group_until[NUM_GROUPS]={0,0,0,0,0,0};
    
  //* Stats
  uint64_t _stat_comp_instances = 0;
  uint64_t _stat_cgra_busy_cycles = 0;
  uint64_t _stat_scratch_read_bytes = 0;
  uint64_t _stat_scratch_write_bytes = 0;
  uint64_t _stat_scratch_reads = 0;
  uint64_t _stat_scratch_writes = 0;
  uint64_t _stat_scratch_bank_requests_pushed = 0;
  uint64_t _stat_scratch_bank_requests_executed = 0;
  double _stat_bank_conflicts=0.0;
  uint64_t _stat_cycles_atomic_scr_pushed=0;
  uint64_t _stat_cycles_atomic_scr_executed=0;

  uint64_t _stat_commands_issued = 0;

  uint64_t _stat_tot_mem_fetched=0;
  uint64_t _stat_tot_mem_stored=0;

  uint64_t _stat_tot_loads=0;
  uint64_t _stat_tot_stores=0;
  uint64_t _stat_tot_mem_store_acc=0;
  uint64_t _stat_tot_mem_load_acc=0;
 
  //Cycle stats
  std::map<int,int> _stat_ivp_put;
  std::map<int,int> _stat_ivp_get;
  std::map<int,int> _stat_ovp_put;
  std::map<int,int> _stat_ovp_get;
  int _stat_mem_bytes_wr=0;
  int _stat_mem_bytes_rd=0;
  int _stat_scr_bytes_wr=0;
  int _stat_scr_bytes_rd=0;
  int _stat_mem_bytes_wr_sat=0;
  int _stat_mem_bytes_rd_sat=0;
  int _stat_cmds_issued=0;
  int _stat_cmds_complete=0;
  int _stat_ss_insts=0;
  // for backcgra
  double _stat_ss_dfg_util=0.0;
  double _stat_ss_data_avail_ratio=0.0;
  int _slot_avail[NUM_IN_PORTS] = {0};
  int _could_not_serve[NUM_IN_PORTS] = {0};

  bool _back_cgra=false;

  // newly added
  const char* _banked_spad_mapping_strategy = "";

  std::map<SS_CONFIG::ss_inst_t,int> _total_histo;
  std::map<int,int> _vport_histo;

  stream_stats_t _stream_stats;  
  pipeline_stats_t _pipe_stats;  
  uint64_t _prev_roi_clock;

  std::map<std::pair<LOC,LOC>, std::pair<uint64_t,uint64_t>> _bw_map;

  //Checking data structures
  std::vector<int> scratchpad_writers;
  std::vector<int> scratchpad_readers;

};

#endif

