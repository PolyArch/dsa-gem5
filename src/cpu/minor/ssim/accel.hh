#pragma once

#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <unordered_map>

#include <cstdio>
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

#include "dsa/debug.h"
#include "dsa/arch/model.h"
#include "dsa/mapper/schedule.h"

#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/lsq.hh"

#include "sim-debug.hh"
#include "stream.hh"
#include "consts.hh"
#include "statistics.h"
#include "./spad.h"

namespace dsa {
namespace sim {

struct StreamArbiter;

}
}

using namespace dsa;

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

// -------------- Vector Ports ------------------------


// "Wide" or interface port (AKA Vector Ports)

// "Wide" or interface port (AKA Vector Ports)
class port_data_t {
public:

  /*!
   * \brief The size of the buffer.
   */
  int buffer_size;

  /*!
   * \brief The size of the data buffer.
   */
  std::queue<uint8_t> buffer;

  /*!
   * \brief The data in the buffer of the crossbar is ready to fire to the spatial arch.
   */
  std::vector<uint64_t> xbar;

  /*!
   * \brief The stream this port currently executes.
   */
  base_stream_t *stream{nullptr};

  /*!
   * \brief A stream can be broadcast to multiple ports.
   *        This stores port repeat information of "this" port.
   */
  dsa::sim::rt::PortExecState pes;

  port_data_t(int size) : buffer_size(size), pes(dsa::sim::IVPState(), -1) {}

  port_data_t() : pes(dsa::sim::IVPState(), -1) {}

  /*!
   * \brief Move on the state this port.
   */
  void tick();

  /*!
   * \brief If we still have space in the FIFO.
   */
  bool bufferAvailable();

  /*!
   * \brief The accelerator this port belongs to.
   */
  accel_t *parent;


  enum class STATUS {FREE, COMPLETE, BUSY};

  void initialize(SSModel* ssconfig, int port, bool isInput);
  void reset(); //reset if configuration happens

  //This function is similar to port_vec_elem, but...
  //For now, the *only* cases where we don't return vector length is
  //1. this port is not assigned a vector, or
  //2. temporal vector: all elements are mapped to the same cgra port
  // TODO: rename it to logical length and cgra port length
  // FIXME: dgra logical ports doesn't work with the temporal region
  unsigned port_cgra_elem() {
    if(_dfg_vec) {
      return _dfg_vec->logical_len();
    }
    return 1;
  } 

  unsigned port_vec_elem(); //total size elements of the std::vector port
  unsigned port_depth() { return port_vec_elem() / port_cgra_elem();} //depth of queue
  // unsigned cgra_port_for_index(unsigned i) { return i%port_cgra_elem();}
  // it should return same output for a range of i (might be an issue for dgra
  // temporal)
  unsigned cgra_port_for_index(unsigned i) { 
    int x = i*8/_port_width;
    return x%port_cgra_elem();
  }

  //reset all data
  void reset_data() {
    stream = nullptr;
    _mem_data.clear();
    _valid_data.clear(); // FIXME:CHECKME: check if this is true!
    _outstanding=0;
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

  bool can_push_bytes_vp(int num_bytes) {
    int num_elem = num_bytes/_port_width;
    // std::cout << "num elem: " << num_elem << " num can push: " << num_can_push() << "\n";
    return num_elem <= num_can_push();
  }

  int num_can_push() {
    // effective buffer size should be more here
    if(_mem_data.size() < VP_LEN*8/_port_width) {
      // return VP_LEN-_mem_data.size();
      return VP_LEN*8/_port_width-_mem_data.size();
    } else {
      return 0;
    }
  }

  //utility functions
  template <typename T>
  std::vector<uint8_t> get_byte_vector(T val, int len){
    // std::cout << "value inside get_byte_vector is: " << val << " and length should be: " << len << "\n";
    std::vector<uint8_t> v;
    for(int i=0; i<len; i++){
      v.push_back((val >> (i*8)) & 255);
    }
    return v;
  }

  // FIXME: hack for now--create a map of data_width and datatype
  template <typename T>
  T get_custom_val(std::vector<uint8_t> v, int len){
    switch (len) {
    case 1:
      return *reinterpret_cast<uint8_t*>(&v[0]);
    case 2:
      return *reinterpret_cast<uint16_t*>(&v[0]);
    case 4:
      return *reinterpret_cast<uint32_t*>(&v[0]);
    case 8:
      return *reinterpret_cast<uint64_t*>(&v[0]);
    default:
      assert(0);
    }
  }

  template <typename T>
  void push_mem_data(T data) {
    int data_size = sizeof(T);
    assert(data_size == _port_width && "data size doesn't match the port width in dfg");
    push_data(data);
  }

  void push_data_byte(uint8_t b) {
     leftover.emplace_back(b, true);
     if(leftover.size() == 8) {
       push_data({},true);
     }
  }

  // not used for only read (should be just for port_resp from dma)
  // associated with each port
  // push mem_data=1 ie. port_width amount of data
  std::vector<std::pair<uint8_t, bool>> leftover;
  void push_data(std::vector<uint8_t> data, bool valid=true) {
    for(uint8_t item : data) {
      leftover.emplace_back(item, valid);
    }
    size_t i;
    for (i = 0; i + _port_width <= leftover.size(); i += _port_width) {
      std::vector<uint8_t> to_append;
      for (int j = 0; j < _port_width; ++j) {
        to_append.push_back(leftover[i + j].first);
        assert(leftover[i + j].second == leftover[i].second);
      }
      // std::cout << "pushed to mem data\n";
      _mem_data.push_back(to_append);
      _valid_data.push_back(leftover[i].second);
      _total_pushed += valid;
    }

    leftover.erase(leftover.begin(), leftover.begin() + i);
  }

  template <typename T>
  void push_data(T data, bool valid=true) {
    int data_size = sizeof(T);
    assert(data_size == _port_width && "data size doesn't match the port width in dfg");
    if (!leftover.empty()) {
      std::cout << "It's not cool to leave random incomplete words in the port, "
           << "please be more tidy next time\n";
      assert(0);
    }
    std::vector<uint8_t> vec(sizeof(T));
    *reinterpret_cast<T*>(&vec[0]) = data;
    push_data(vec, valid);
  }

  void PadData(bool stride_first, bool stride_last, Padding policy) {
    auto f = [this] (bool valid) {
      int width = port_cgra_elem();
      int residue = _mem_data.size() % width;
      if (residue == 0) {
        return;
      }
      std::vector<uint8_t> dummy_val(_port_width, 0);
      int extra = width - residue;
      for(int i = 0; i < extra; ++i) {
        push_data(dummy_val, valid);
      }
    };
    if (stride_first) {
      if (policy == DP_PreStridePredOff) {
        f(false);
      } else if (policy == DP_PreStrideZero) {
        f(true);
      }
    }
    if (stride_last) {
      if (policy == DP_PostStridePredOff) {
        f(false);
      } else if (policy == DP_PostStrideZero) {
        f(true);
      }
    }
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


  void push_cgra_port(unsigned cgra_port, SBDT val, bool valid) {
    std::vector<uint8_t> v(sizeof(SBDT));
    *reinterpret_cast<SBDT*>(&v[0]) = val;
    _cgra_data[cgra_port].push_back(v);
    _cgra_valid[cgra_port].push_back(valid);
  }

  //get the value of an instance in cgra port
  SBDT value_of(unsigned port_idx, unsigned instance) {
    auto &vec = _cgra_data[port_idx][instance];
    switch (vec.size()) {
    case 1:
      return *reinterpret_cast<uint8_t*>(&vec[0]);
    case 2:
      return *reinterpret_cast<uint16_t*>(&vec[0]);
    case 4:
      return *reinterpret_cast<uint32_t*>(&vec[0]);
    case 8:
      return *reinterpret_cast<uint64_t*>(&vec[0]);
    default:
      assert(0);
    }
  }

  bool valid_of(unsigned port_idx, unsigned instance) {
    return _cgra_valid[port_idx][instance];
  }

  SBDT pop_in_data(); // pop one data from mem
  template <typename T>
  T pop_in_custom_data(); // pop one data from mem

  SBDT pop_out_data(int bytes = -1); // pop one data from mem
  template <typename T>
  T pop_out_custom_data(); // pop one data from mem
  SBDT peek_out_data(int i = 0, int bytes = -1); // peek one data from mem

  bool any_data() {
    if(_isInput) {
      return num_ready();
    } else {
      return mem_size();
    }
  }

  unsigned mem_size() {
    return _mem_data.size(); // size of the deque (_mem_data.size()*_port_width)
  }

  unsigned num_ready() { return _num_ready; }         //Num of ready instances

  unsigned num_in_flight() {return _num_in_flight;}  //outputs in flight

  std::string status_string() {
    if(status()==STATUS::FREE) return "free";
    std::string ret;
    if(loc()==LOC::SCR)  ret = " (SCR)";
    if(loc()==LOC::PORT) ret = " (PORT)";
    if(loc()==LOC::DMA)  ret = " (DMA)";
    if(status()==STATUS::BUSY)     return "BUSY " + ret;
    if(status()==STATUS::COMPLETE) return "COMP " + ret;
    assert(0);
  }

  void bindStream(base_stream_t *exec_stream);

  void freeStream();

  bool in_use() {
    return status() != STATUS::FREE;
  }

  bool completed() {
    return status() == STATUS::COMPLETE;
  }

  LOC loc() {
    if (stream) {
      return stream->src();
    }
    return LOC::NONE;
  }

  STATUS status() {
    if (stream) {
      return stream->stream_active() ? STATUS::BUSY : STATUS::COMPLETE;
    }
    return STATUS::FREE;
  }

  void pop(unsigned instances);  //Throw away data in CGRA input ports

  uint64_t total_pushed() { return _total_pushed; }

  void set_port_width(int num_bits){
    assert(num_bits % 8 == 0);
    _port_width = num_bits / 8;
  }

  // in bytes
  int get_port_width() {
    CHECK(_dfg_vec) << port();
    return _dfg_vec->bitwidth() / 8;
  }

  void set_dfg_vec(dsa::dfg::VectorPort* dfg_vec) {
    _dfg_vec = dfg_vec;
    // std::cout << "CGRA ELEM SIZE: " << port_cgra_elem() << "\n";
    _cgra_data.resize(port_cgra_elem());
    _cgra_valid.resize(port_cgra_elem());
    assert(_cgra_data.size() > 0);
  }

  // stats info (see if I need it)
  void inc_rem_wr(int x) { _num_rem_wr+=x; }
  unsigned get_rem_wr() { return _num_rem_wr; }

  void set_is_bytes_waiting_final() {
    _is_bytes_waiting_final=true;
  }


  void reset_is_bytes_waiting_final() {
    _is_bytes_waiting_final=false;
  }

  void inc_bytes_waiting(int x) {
    _bytes_waiting += x;
    // std::cout << "New bytes waiting: " << _bytes_waiting << " and inc: " << x << "\n";
  }

  bool is_bytes_waiting_zero() {
    return _bytes_waiting==0;
  }

  bool get_is_bytes_waiting_final() {
    return _is_bytes_waiting_final;
  }

  int get_bytes_waiting() {
    return _bytes_waiting;
  }

private:

  dsa::dfg::VectorPort *_dfg_vec{nullptr}; // null by default

  // cgra_port: vec_loc(offset)
  // 23: 1, 2   24: 3, 4
  bool _isInput;
  int _port=-1;
  int _port_width=-1; // 8; // take 8 bytes by default
  int _outstanding=0;
  // std::deque<SBDT> _mem_data;
  std::deque<std::vector<uint8_t>> _mem_data;
  std::deque<bool> _valid_data;
  // std::vector<std::deque<SBDT>> _cgra_data; //data per port
  // outermost for number of scalar ports, and innermost is for datatype of the
  // port
  std::vector<std::deque<std::vector<uint8_t>>> _cgra_data; //data per scalar port
  std::vector<std::deque<bool>> _cgra_valid; //data per port
  unsigned _num_in_flight=0;
  unsigned _num_ready=0;

  unsigned _num_rem_wr=0;

  uint64_t _total_pushed=0;
  int _bytes_waiting=0;
  bool _is_bytes_waiting_final=false;
};

//Entire Port interface with each being port_data_t
class port_interf_t {
public:

  void initialize(SSModel* ssconfig);
  // void initialize(SSModel* ssconfig, Schedule* sched);
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

  port_data_t& in_port(int i) { return _in_port_data[i]; }
  port_data_t& out_port(int i) { return _out_port_data[i]; }
  std::vector<port_data_t> &ports(bool is_input) { return is_input ? _in_port_data : _out_port_data; }

  void reset() {
    for(unsigned i = 0; i < _in_port_data.size(); ++i) {
      if(i == NET_VAL_PORT || i == NET_ADDR_PORT){
      } else {
        _in_port_data[i].reset();
      }
    }
    for(unsigned i = 0; i < _out_port_data.size(); ++i) {
      // TODO: check if this was useful!
      _out_port_data[i].reset();
      /*
      if(_out_port_data[i].port() == MEM_SCR_PORT || _out_port_data[i].port() == SCR_MEM_PORT){
        // printf("Detected a network port\n");
      } else {
      // printf("Index of out_port is: %d\n",i);
        _out_port_data[i].reset();
      }
      */
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
class network_controller_t;


//Limitations: 1 simultaneously active scratch stream
class dma_controller_t : public data_controller_t {
  friend class scratch_write_controller_t;
  friend class scratch_read_controller_t;
  friend class network_controller_t;

 public:
  static const int data_width=64; //data width in bytes
  // static const int data_ssdts=data_width/SBDT; //data width in bytes

  dma_controller_t(accel_t* host,
      scratch_read_controller_t* scr_r_c,
      scratch_write_controller_t* scr_w_c,
      network_controller_t* net_c) :
    data_controller_t(host), _scr_r_c(scr_r_c), _scr_w_c(scr_w_c), _net_c(net_c) {

    _prev_port_cycle.resize(64); //resize to maximum conceivable ports
  }

  void reset_stream_engines() {
    _read_streams.clear();
    _write_streams.clear();
  }

  void reset_data() {
    reset_stream_engines();
    // _mem_read_reqs=0; -- TODO:CHECK:cleanup mode should wait for all pending memory
    // requests to come back and then reduce it
    _mem_write_reqs=0;
    _fake_scratch_reqs=0;
  }

  void cycle();
  void finish_cycle();
  bool done(bool show, int mask);

  void make_read_request();
  void make_write_request();

  void print_status();
  void cycle_status();

  bool mem_reads_outstanding()  {return _mem_read_reqs;}
  bool mem_writes_outstanding() {return _mem_write_reqs;}
  bool scr_reqs_outstanding()  {return _fake_scratch_reqs;}

  bool indirect_streams_active();
  bool indirect_wr_streams_active();

  //----------------------
  float calc_min_port_ready();
  // for initiation interval
  // bool first_entry = true; 
  //---------------------------

  int mem_reqs() {return _mem_read_reqs + _mem_write_reqs;}

  scratch_read_controller_t*  scr_r_c() {return _scr_r_c;}
  scratch_write_controller_t* scr_w_c() {return _scr_w_c;}
  network_controller_t* net_c() {return _net_c;}
  private:

  scratch_read_controller_t* _scr_r_c;
  scratch_write_controller_t* _scr_w_c;
  network_controller_t* _net_c;

  void port_resp(unsigned i);

  unsigned _which_rd=0, _which_wr=0;

  std::vector<base_stream_t*> _read_streams;
  std::vector<base_stream_t*> _write_streams;

  //address to stream -> [stream_index, data]
  uint64_t _mem_read_reqs=0, _mem_write_reqs=0;
  std::vector<uint64_t> _prev_port_cycle;
  uint64_t _prev_scr_cycle=0;
  int _fake_scratch_reqs=0;

};

class scratch_read_controller_t : public data_controller_t {
  public:
  std::vector <bool> mask;

  scratch_read_controller_t(accel_t* host, dma_controller_t* d)
    : data_controller_t(host) {
    _dma_c=d; //save this for later
    mask.resize(SCR_WIDTH);
    _indirect_scr_read_requests.resize(NUM_SCRATCH_BANKS);
  }

  // std::vector<SBDT> read_scratch(LinearReadStream& stream);
  std::vector<uint8_t> read_scratch(LinearReadStream& stream, bool is_banked);
  bool checkLinearSpadStream(LinearReadStream& stream);

  // float calc_min_port_ready();
  float calc_min_port_ready(bool is_banked);
  float calc_min_ind_port_ready();
  // void cycle();
  void cycle(bool &performed_read);
  void linear_scratch_cycle();
  void serve_ind_read_banks();
  // banked scratchpad read buffers
  bool indirect_scr_read_requests_active();

  int cycle_read_queue();

  void finish_cycle();

  void print_status();
  void cycle_status();

  void push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry);
  void push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry);

  private:
  int _which_rd=0;
  int _which_linear_rd=0;

  struct ind_reorder_entry_t {
    uint8_t data[64]; //64 bytes per request
    int size; //number of writes that should be completed
    int completed=0;
    int data_bytes;
    base_stream_t* stream;
    bool last = false;
    int id = -1; // should store this I guess
  };

  // a single entry (which entry it is associated with)
  // should be id?
  // FIXME: should be deleted once done..
  std::unordered_map<int, ind_reorder_entry_t*> _reorder_entry_id;
  int _cur_irob_ptr = -1;

  struct indirect_scr_read_req{
    // void *ptr;
    int data_ptr;
    uint64_t addr;
    size_t bytes;
    int irob_entry_id;
    bool remote = false;
    int req_core = -1;
    // ind_reorder_entry_t* reorder_entry=NULL;
  };

  std::vector<base_stream_t*> _read_streams;

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
    // mask.resize(SCR_WIDTH/DATA_WIDTH);
    mask.resize(SCR_WIDTH);
    // _remote_scr_w_buf.resize(DEFAULT_FIFO_LEN); // same size as cgra port fifo's
    _atomic_scr_issued_requests.resize(NUM_SCRATCH_BANKS);

    _network_streams.resize(1); // just one such stream for now

    // initialize a dummy network streams
    // remote_core_net_stream_t* implicit_stream = new remote_core_net_stream_t();
    // push_net_stream(implicit_stream);

    //if(is_shared()) {
    //  _scr_scr_streams.resize(NUM_ACCEL);
    //} else {
    //  _scr_scr_streams.resize(1);
    //}
  }

  void write_scratch_remote_ind(remote_core_net_stream_t& stream);
  void write_scratch_remote_direct(direct_remote_scr_stream_t& stream);
  void atomic_scratch_update(atomic_scr_stream_t& stream);
  void serve_atomic_requests(bool &performed_atomic_scr);
  void push_remote_wr_req(uint8_t *val, int num_bytes, addr_t scr_addr);
  void scr_write(addr_t addr, LinearWriteStream& stream, port_data_t& out_vp);
  // for remote atomic update
  void push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc);

  void insert_pending_request_queue(int tid, std::vector<int> start_addr, int bytes_waiting);
  int push_and_update_addr_in_pq(int tid, int num_bytes);
  void push_atomic_inc(int tid, std::vector<uint8_t> inc, int repeat_times);

  void reset_stream_engines() {
    _atomic_scr_streams.clear();
    _write_streams.clear();
    // while(!_remote_scr_w_buf.empty()) { _remote_scr_w_buf.pop(); }
  }

  void reset_data() {
    reset_stream_engines();
  }

  bool crossbar_backpressureOn();
  bool atomic_addr_full(int bytes);
  bool atomic_val_full(int bytes);
  bool pending_request_queue_full();

  int atomic_val_size() { return _atom_val_store.size(); }
  int pending_request_queue_size() { return _pending_request_queue.size(); }
  int conflict_queue_size() { return _conflict_detection_queue.size(); }

  // void cycle();
  void cycle(bool can_perform_atomic_scr, bool &performed_atomic_scr);
  void linear_scratch_write_cycle();
  void finish_cycle();
  bool done(bool,int);

  bool schedule_atomic_scr_op(atomic_scr_stream_t& s);
  bool schedule_network_stream(remote_core_net_stream_t& s);

  void print_status();
  void cycle_status();

  bool atomic_scr_streams_active();
  bool atomic_scr_issued_requests_active();

  bool release_df_barrier(){
    assert(_df_count!=-1);
    printf("df_count: %ld current_writes: %ld\n",_df_count,_remote_scr_writes);
    return (_remote_scr_writes==_df_count);
  }

  void set_df_count(int64_t df_count){
    // printf("Setting df count to be: %ld\n",df_count);
    // printf("Current remote writes: %ld\n",_remote_scr_writes);
    _df_count = df_count;
  }

  void set_atomic_cgra_addr_port(int p) { _atomic_cgra_addr_port=p; }
  void set_atomic_cgra_val_port(int p) { _atomic_cgra_val_port=p; }
  void set_atomic_cgra_out_port(int p) { _atomic_cgra_out_port=p; }
  void set_atomic_addr_bytes(int p) { _atomic_addr_bytes=p; }
  void set_atomic_val_bytes(int p) { _atomic_val_bytes=p; }
  bool is_conflict(addr_t scr_addr, int num_bytes);
  void push_atomic_val();

  private:
  int _which_wr=0; // for banked scratchpad
  int _which_linear_wr=0; // for linear scratchpad
  int which_buffet{0};

  // int _num_bytes_to_update=0;
  std::vector<int> _update_broadcast_dest;
  std::vector<int> _update_coalesce_vals;

  int _atomic_cgra_addr_port=-1;
  int _atomic_cgra_val_port=-1;
  int _atomic_cgra_out_port=-1;
  int _atomic_addr_bytes=8;
  int _atomic_val_bytes=8;
  // for each bank
  std::unordered_map<int, std::pair<int, std::vector<int>>> _pending_request_queue; // [NUM_SCRATCH_BANKS];
  // std::unordered_map<int, int> _conflict_detection_queue;
  // addr, bytes
  std::deque<std::pair<int,int>> _conflict_detection_queue;
  // std::queue<std::vector<uint8_t>> _atom_val_store;
  // tid, and its corresponding data
  /*
  struct atomic_reorder_entry { // create when you receive an address
    std::vector<uint8_t> data;
    int tid;
    bool done;
    int req_bytes;
    int repeat_times;
  };
  atomic_reorder_entry reorder_buffer[MAX_ATOM_REQ_QUEUE_SIZE]; // tid, done?
  */

  std::unordered_map<int, std::vector<uint8_t>> _atom_val_store;
  // tid, repeat_times, done?
  std::pair<int,std::pair<int,bool>> reorder_buffer[MAX_ATOM_REQ_QUEUE_SIZE]; // tid, done?
  int _cur_drob_fill_ptr=0;
  int _cur_drob_pop_ptr=0;
  int _drob_size_used=0;
  // std::vector<uint8_t> _temp_accum;

  struct atomic_scr_op_req{
    addr_t _scr_addr;
    SBDT _inc;
    int _opcode;
    int _value_bytes;
    int _output_bytes;
  };

  // FIXME: how do we make it group? (val,addr)
  // TODO: check if we can do this in a better way!
  struct ind_write_req{
    addr_t scr_addr;
    int64_t val;
    ind_write_req(addr_t addr, int64_t value){
      scr_addr=addr;
      val=value;
    }
  };

  void delete_stream(int i, LinearWriteStream* s);
  void delete_stream(int i, atomic_scr_stream_t* s);

  int _logical_banks = NUM_SCRATCH_BANKS;
  int64_t _remote_scr_writes=0;
  int64_t _df_count=-1; // only 1 active at a time
  std::vector<base_stream_t*> _write_streams;

  std::vector<atomic_scr_stream_t*> _atomic_scr_streams;

  // TODO: fix the size of these queues
  // std::queue<struct ind_write_req> _remote_scr_w_buf;
  std::vector<std::queue<atomic_scr_op_req>> _atomic_scr_issued_requests;
  std::vector<remote_core_net_stream_t*> _network_streams;
  dma_controller_t* _dma_c;
};

// It deals with all the remote rd/wr requests
class network_controller_t : public data_controller_t {
  public:
  // std::vector <bool> mask;

  network_controller_t(accel_t* host, dma_controller_t* d)
    : data_controller_t(host) {
    _dma_c=d; //save this for later

    // it might be needed to port->spad things
    // mask.resize(SCR_WIDTH/DATA_WIDTH);
    reset_stream_engines();
  }

  void reset_stream_engines() {
    _remote_port_multicast_streams.clear();
    _remote_scr_streams.clear();
    _direct_remote_scr_streams.clear();
  }

  void reset_data() {
    reset_stream_engines();
  }

  void serve_pending_net_req();
  void check_cpu_response_queue();
  void multicast_data(remote_port_multicast_stream_t& stream, int message_size);
  void write_remote_scr(remote_scr_stream_t& stream);
  void write_direct_remote_scr(direct_remote_scr_stream_t& stream);

  void cycle();
  bool remote_port_multicast_requests_active();
  bool remote_scr_requests_active();
  bool direct_remote_scr_requests_active();

  void finish_cycle();
  bool done(bool show, int mask);

  bool schedule_remote_port_multicast(remote_port_multicast_stream_t& s);
  bool schedule_remote_scr(remote_scr_stream_t& s);
  bool schedule_direct_remote_scr(direct_remote_scr_stream_t& s);

  void print_status();
  void cycle_status();

  private:
  // to schedule the streams in the stream table
  int _which_remote=0;

  // It contains all kind of streams: copy?
  std::vector<base_stream_t*> _remote_streams;

  void delete_stream(int i, remote_port_multicast_stream_t* s);
  void delete_stream(int i, remote_scr_stream_t* s);
  void delete_stream(int i, direct_remote_scr_stream_t* s);

  std::vector<remote_port_multicast_stream_t*> _remote_port_multicast_streams;
  std::vector<remote_scr_stream_t*> _remote_scr_streams;
  std::vector<direct_remote_scr_stream_t*> _direct_remote_scr_streams;

  // queues are the buffers associated with each bank
  // std::vector<std::queue<indirect_scr_read_req>> _indirect_scr_read_requests;
  // std::queue<ind_reorder_entry_t*> _ind_ROB;

  // TODO: do we need that?
  dma_controller_t* _dma_c;
};

struct stream_stats_histo_t {
  uint64_t vol_by_type[(int)STR_PAT::LEN];
  uint64_t vol_by_len[64];
  //std::unordered_map<uint64_t,uint64_t> vol_by_len_map;
  std::map<uint64_t,uint64_t> vol_by_len_map;
  //std::unordered_map<std::pair<int,int>,uint64_t,pair_hash> vol_by_source;
  std::map<std::pair<int,int>,uint64_t> vol_by_source;
  uint64_t total_vol=0;
  uint64_t total=0;

  stream_stats_histo_t() {
    for(int i = 0; i < (int)STR_PAT::LEN; ++i) {vol_by_type[i]=0;}
    for(int i = 0; i < 64; ++i)     {vol_by_len[i]=0;}
  }

#define check_and_add(a, b, c) \
  do {\
    if (a.find(b) != a.end()) \
      a[b] += c; \
    else \
      a[b] = c; \
  } while(false)

  void add(STR_PAT t, LOC src, LOC dest, uint64_t vol) {
    check_and_add(vol_by_source, std::make_pair((int)src,(int)dest), vol);
    vol_by_type[(int)t] += vol;
    vol_by_len[ilog2(vol)+1]+=vol;
    check_and_add(vol_by_len_map, vol, vol);
    total_vol+=vol;
    total++;
  }

#undef check_and_add

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
      out << LOC_NAME[i.first.first] << "->"
          << LOC_NAME[i.first.second] << ": ";
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

class accel_t {
  friend class ssim_t;
  friend class scratch_read_controller_t;
  friend class scratch_write_controller_t;
  friend class network_controller_t;
  friend class dma_controller_t;

public:

  /*!
   * \brief The stream scheduler.
   */
  sim::StreamArbiter *arbiter{nullptr};

  /*!
   * \brief The statistics of the accelerator.
   */
  stat::Accelerator statistics;

  /*!
   * \brief The information of soft configuration.
   */
  dsa::sim::BitstreamWrapper bsw;

  accel_t(int i, ssim_t* ssim);

  Minor::LSQ *lsq();

  bool in_use();
  void timestamp(); //print timestamp
  uint64_t now(); //return's sim's current time

  //Stats Interface
  void print_stats();
  void pedantic_statistics(std::ostream&);
  void print_statistics(std::ostream&);
  void print_status();

  void cycle_status();
  void cycle_status_backcgra();
  void clear_cycle();

  void request_reset_data();
  void request_reset_streams();
  void switch_stream_cleanup_mode_on();
  bool all_ports_empty();

  port_interf_t& port_interf() {
    return _port_interf;
  }


  bool done(bool show = false, int mask = 0);

  uint64_t forward_progress_cycle() { return _forward_progress_cycle; }

  void process_stream_stats(base_stream_t& s) {
    uint64_t    vol  = s.data_volume();
    _stream_stats.add((STR_PAT) 0,s.src(),s.dest(),vol,0);
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

    //inefficient (can be done at the end), but improve later : )
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
  bool is_shared();
  int accel_index() {return _accel_index;}
  int get_cur_cycle();

  scratch_read_controller_t*  scr_r_c() {return &_scr_r_c;}
  scratch_write_controller_t* scr_w_c() {return &_scr_w_c;}
  network_controller_t* net_c() {return &_net_c;}

private:
  ssim_t* _ssim;
  bool _cleanup_mode=false;
  bool _stream_cleanup_mode=false;

  //***timing-related code***
  bool done_internal(bool show, int mask);
  bool done_concurrent(bool show, int mask);

  void cycle_cgra();   //Tick on each cycle
  void cycle_cgra_backpressure();

  void reset_data(); //carry out the work

  void cycle_in_interf();
  void cycle_out_interf();
  void cycle_indirect_interf(); //forward from indirect inputs to indirect outputs
  void schedule_streams();

  bool cgra_done(bool, int mask);
  bool cgra_input_active() {
    for (int i = 0; i < bsw.ports[1].size(); ++i) {
      int cur_port = bsw.ports[1][i].port;
      auto& in_vp = _port_interf.in_port(cur_port);
      if(in_vp.in_use() || in_vp.num_ready() || in_vp.mem_size()) {
        return true;
      }
    }
    return false;
  }

  bool cgra_compute_active() {
    for(int i = 0; i < bsw.ports[0].size(); ++i) {
      int cur_port = bsw.ports[0][i].port;
      auto& out_vp = _port_interf.out_port(cur_port);
      if(out_vp.num_in_flight()) {
        return true;
      }
    }
    return false;
  }

  bool cgra_output_active() {
    for(int i = 0; i < bsw.ports[0].size(); ++i) {
      int cur_port = bsw.ports[0][i].port;
      auto& out_vp = _port_interf.out_port(cur_port);
      if(out_vp.in_use() || out_vp.num_ready() || out_vp.mem_size()) {
        return true;
      }
    }
    return false;
  }

  bool in_roi();

  void verif_cmd(base_stream_t* s) {}

  void sanity_check_stream(base_stream_t* s);

  void do_cgra();

  void forward_progress() {
    _waiting_cycles=0;
    _forward_progress_cycle=now();
  }


  void read_scratchpad(void* dest, uint64_t scr_addr,
      std::size_t count, int id) {
    if(_linear_spad){
      assert(scr_addr < SCRATCH_SIZE+LSCRATCH_SIZE);
    } else {
      assert(scr_addr < SCRATCH_SIZE);
    }

    std::memcpy(dest, &scratchpad[scr_addr], count);
    // TODO: change this for linear spad case
    for(int i = 0; i < count; i+=sizeof(SBDT)) {
      uint64_t running_addr=  scr_addr+i;
      if(scratchpad_writers[running_addr]) {
        DSA_LOG(CHECK_SCR_ALIAS)
          << "WARNING: scr_addr: " << running_addr
          << " constistency is potentially violated; "
          << " writer_id: " << scratchpad_writers[running_addr]
          << " reader_id: " << id;
      }
      scratchpad_readers[running_addr]=id;
    }
  }

  void write_scratchpad(uint64_t scr_addr, const void* src,
      std::size_t count, int id) {
    // std::cout << "NEW SCRATCHPAD SIZE: " << scratchpad.size() << "\n";
    std::memcpy(&scratchpad[scr_addr], src, count);
    for(int i = 0; i < count; i+=sizeof(SBDT)) {
      uint64_t running_addr = scr_addr+i;
      if(scratchpad_writers[running_addr]) {
        DSA_LOG(CHECK_SCR_ALIAS)
          << "WARNING: scr_addr: " << running_addr
          << " constistency is potentially violated; "
          << " writer_id: " << scratchpad_writers[running_addr]
          << " writer_id: " << id;
      }
      if(scratchpad_readers[running_addr]) {
        DSA_LOG(CHECK_SCR_ALIAS)
          << "WARNING: scr_addr: " << running_addr
          << " constistency is potentially violated; "
          << " reader_id: " << scratchpad_readers[running_addr]
          << " writer_id: " << id;
      }
      scratchpad_writers[running_addr]=id;
    }
  }

  void receive_message(int8_t* data, int num_bytes, int remote_in_port) {
    port_data_t& in_vp = _port_interf.in_port(remote_in_port);
    // TODO: Check the max port size here and apply backpressure
    
    // assert(num_bytes%in_vp.get_port_width()==1);
    std::vector<uint8_t> temp;
    // TODO: make it uint everywhere on n/w side
    for(int i=0; i<num_bytes; ++i){
      temp.push_back(data[i]);
    }
    DSA_LOG(NET_REQ) << "Received value: " << *reinterpret_cast<SBDT *>(&temp[0])
                 << " at remote port: " << remote_in_port;
    in_vp.push_data(temp);
    // inc remote values received at this port
    in_vp.inc_rem_wr(num_bytes/in_vp.get_port_width());
  }

  void push_scratch_remote_buf(uint8_t* val, int num_bytes, uint16_t scr_addr){
      _scr_w_c.push_remote_wr_req(val, num_bytes, scr_addr);
  }

  void push_atomic_update_req(int scr_addr, int opcode, int val_bytes, int out_bytes, uint64_t inc) {
    _scr_w_c.push_atomic_update_req(scr_addr, opcode, val_bytes, out_bytes, inc);
  }

  void insert_pending_request_queue(int tid, std::vector<int> start_addr, int bytes_waiting) {
    _scr_w_c.insert_pending_request_queue(tid, start_addr, bytes_waiting);
  }

  int push_and_update_addr_in_pq(int tid, int num_bytes) {
    return _scr_w_c.push_and_update_addr_in_pq(tid, num_bytes);
  }
  
  void push_atomic_inc(int tid, std::vector<uint8_t> inc, int repeat_times) {
    _scr_w_c.push_atomic_inc(tid, inc, repeat_times);
  }

  void push_ind_rem_read_req(bool is_remote, int req_core, int request_ptr, int addr, int data_bytes, int reorder_entry) {
      _scr_r_c.push_ind_rem_read_req(is_remote, req_core, request_ptr, addr, data_bytes, reorder_entry);
  }

  void push_ind_rem_read_data(int8_t* data, int request_ptr, int addr, int data_bytes, int reorder_entry) {
      _scr_r_c.push_ind_rem_read_data(data, request_ptr, addr, data_bytes, reorder_entry);
  }

  bool isLinearSpad(addr_t addr){
    if(!_linear_spad) return false;
    // int spad_offset_bits = log2(SCRATCH_SIZE+LSCRATCH_SIZE);
    // could be remote location
    // assert(addr < (SCRATCH_SIZE+LSCRATCH_SIZE));
    int spad_offset_bits = log2(SCRATCH_SIZE);
    int spad_type = (addr >> spad_offset_bits) & 1;
    return spad_type; // for 1, it is linear
  }

  // to debug
  void print_spad_addr(int start, int end){
    uint16_t val = 0;
    for(int i=start; i<end; i+=2){
      memcpy(&val, &scratchpad[i], 2);
      std::cout << "value is: " << val << "\n";
    }

  }

  void push_net_in_cmd_queue(base_stream_t* s);


  //members------------------------
  port_interf_t _port_interf;

  SSModel *_ssconfig{nullptr};

  int _fu_fifo_len=DEFAULT_FIFO_LEN;
  int _ind_rob_size=DEFAULT_IND_ROB_SIZE;


  std::vector<uint8_t> scratchpad;

  unsigned scratch_line_size = 16;                //16B line
  unsigned fifo_depth = 32;
  unsigned _queue_size=16;

  uint64_t _accel_index = 0;
  uint64_t _accel_mask = 0;

  // Controllers
  dma_controller_t _dma_c;
  scratch_read_controller_t _scr_r_c;
  scratch_write_controller_t _scr_w_c;
  network_controller_t _net_c;

  std::map<uint64_t,std::vector<int>> _cgra_output_ready;

  //Stuff for tracking stats
  uint64_t _waiting_cycles=0;
  uint64_t _forward_progress_cycle=0;

 public:
  std::vector<dsa::sim::ScratchMemory> spads;
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
  uint64_t _stat_port_multicast = 0; // TODO: use this!
  uint64_t _stat_remote_scratch_writes = 0; // TODO: use this!
  uint64_t _stat_scratch_bank_requests_pushed = 0;
  uint64_t _stat_scratch_bank_requests_executed = 0;
  double _stat_bank_conflicts=0.0;
  uint64_t _stat_cycles_atomic_scr_pushed=0;
  uint64_t _stat_cycles_atomic_scr_executed=0;

  uint64_t _stat_tot_mem_fetched=0;
  uint64_t _stat_tot_mem_stored=0;

  uint64_t _stat_tot_loads=0;
  uint64_t _stat_tot_stores=0;
  uint64_t _stat_tot_updates=0;
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
  int _stat_tot_mem_wait_cycles=0;
  int _stat_hit_bytes_rd=0;
  int _stat_miss_bytes_rd=0;
  int _stat_num_spu_req_coalesced=0;
  int _stat_conflict_cycles=0;
  int _stat_tot_atom_cycles=0;
  // for backcgra
  // int _stat_mem_initiation_interval = 10000;
  double _stat_port_imbalance=0;
  double _stat_ss_dfg_util=0.0;
  double _stat_ss_data_avail_ratio=0.0;
  int _slot_avail[NUM_IN_PORTS] = {0};
  int _could_not_serve[NUM_IN_PORTS] = {0};

  //FIXME: just for debug, fix later
  int _num_cycles_issued=0;
  int _num_atomic_sent=0;
  int _assumed_bytes=512; // to debug

  // int _bytes_rd5=0;

  bool _back_cgra=false;
  bool _linear_spad=false;

  // newly added
  const char* _banked_spad_mapping_strategy = "";

  std::map<dsa::OpCode,int> _total_histo;
  std::map<int,int> _vport_histo;

  stream_stats_t _stream_stats;
  pipeline_stats_t _pipe_stats;
  uint64_t _prev_roi_clock;

  std::map<std::pair<LOC,LOC>, std::pair<uint64_t,uint64_t>> _bw_map;

  //Checking data structures
  std::vector<int> scratchpad_writers;
  std::vector<int> scratchpad_readers;

};
