#ifndef __SOFTSIM_H__
#define __SOFTSIM_H__

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

#include "sim-debug.hh"
#include "ticker.hh"
//#include "softsim_interf.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/lsq.hh"
#include "stream.hh"
#include "consts.hh"

using namespace SB_CONFIG;

//This should be depricated now
#define WAIT_FOR(func,cyc) \
  _ticker->set_in_use(); \
  while(!func()) { \
    _ticker->tick(); \
    if(_waiting_cycles > MAX_WAIT) { \
      cout << "in port queue size:" << _in_port_queue.size() << "\n"; \
      cout << "SOFTBRAIN IS STUCK -- CMD QUEUE FULL FOR TOOO LONG)\n"; \
      cout << "start_wait: " << now() - _waiting_cycles \
                             << " now: " << now() << "\n"; \
      print_statistics(cerr); \
      print_status(); \
      assert(0 && "stopping"); \
    }\
  }

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
class ticker_t;


//configuration
class soft_config_t {
public:
  std::vector<int> in_ports_active;
  std::vector<int> out_ports_active;
  std::vector<int> in_port_delay; //delay ports

  std::vector<int> out_ports_lat;

  std::vector<bool> cgra_in_ports_active;

  std::vector<std::vector<SbPDG_Input*>>  input_pdg_node; //input pdg nodes for vec. port
  std::vector<std::vector<SbPDG_Output*>> output_pdg_node;

  std::map<SB_CONFIG::sb_inst_t,int> inst_histo;

  void reset();
};

// -------------- Vector Ports ------------------------

// "Wide" or interface port (AKA Vector Ports)
class port_data_t {
public:
  enum class STATUS {FREE, COMPLETE, BUSY};

  void initialize(SbModel* sbconfig, int port, bool isInput);
  void reset(); //reset if configuration happens
  unsigned port_cgra_elem() {return _port_map.size();} //num of pairs in mapping 
  
  unsigned port_vec_elem(); //total size elements of the std::vector port 
  unsigned port_depth() { return port_vec_elem() / port_cgra_elem();} //depth of queue
  unsigned cgra_port_for_index(unsigned i) { return _port_map[i].first;}

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


  void push_data(SBDT data, uint64_t id=0) {
    //std::cout << data << " -> vp:" << _port << "\n"; 
    if(id!=0) {
//      if(id != _highest_stream_id) {
//        std::cout << "str id: " << std::hex << id<<" old: "<<_highest_stream_id<<"\n";
//      }
      assert(id >= _highest_stream_id);
      _highest_stream_id=id;
    }
    _mem_data.push_back(data); 

    _total_pushed++;
  }
  void push_data(std::vector<SBDT> data,uint64_t id=0) { 
    for(auto i : data) push_data(i,id); 
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
  void push_val(unsigned cgra_port, SBDT val) {_cgra_data[cgra_port].push_back(val);}

  void inc_ready(unsigned instances) {_num_ready+=instances;}

  //get the value of an instance in cgra port
  SBDT value_of(unsigned port_idx, unsigned instance) {
    return _cgra_data[port_idx][instance];
  }

  SBDT pop_data(); // pop one data from mem
  SBDT peek_data(); // peek one data from mem
  SBDT peek_data(int i); // peek one data from mem


  unsigned mem_size() {return _mem_data.size();}
  unsigned num_ready() {return _num_ready;}         //Num of ready instances
  unsigned num_in_flight() {return _num_in_flight;}  //outputs in flight

  void push_fake(unsigned instances);   //Push fake data to each cgra output port

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

  void set_status(STATUS status, LOC loc=LOC::NONE) {
    if(_status == STATUS::BUSY) {
      assert(status != STATUS::BUSY && "can't set busy if already busy\n");
    }
    if(_status == STATUS::FREE) {
      assert(status != STATUS::FREE && "can't free if already free\n");
      assert(status != STATUS::COMPLETE && "can't complete a free\n");
    }
    if(status == STATUS::BUSY) {
      assert((_loc==loc || _status==STATUS::FREE) && 
          "can only assign to a port with eqiv. loc, or if the status is FREE");
    }

    if(SB_DEBUG::VP_SCORE) {
      std::cout << (_isInput ? "ip" : "op") << std::dec << _port;
      std::cout << " " << status_string();
    }
    
    if(status == STATUS::FREE) {
      //We are really just freeing one stream! So...
      //only enter truely free status if no outstanding streams
      _outstanding--;
      if(_outstanding==0) {
        _status=STATUS::FREE;
      } 
    } else {
       _status=status; // no need 
      if(status == STATUS::BUSY) {
        _outstanding++;
        _loc=loc;
      }
    }

    if(SB_DEBUG::VP_SCORE) {
      std::cout << " -> " << status_string() << "\n";
    }

  }

  bool can_take(LOC loc) {
    return (_status == STATUS::FREE) || 
           (_status == STATUS::COMPLETE && _loc == loc);
  }
  bool in_use() {
    return !(_status == STATUS::FREE);
  }
  bool completed() {
    return _status == STATUS::COMPLETE;
  }

  void pop(unsigned instances);  //Throw away data in CGRA input ports

  //NOTE:TODO:FIXME: Right now we only support wide maps, so pm is not really
  //necessary -- we construct our own pm from the mask -- maybe fix this later
  //when we are more ambitious
  void set_port_map(std::vector<std::pair<int, std::vector<int> > > pm, 
                    std::vector<bool> mask) {
    assert(mask.size() == pm.size()); //masks should be the same!
    assert(mask.size() != 0);
    int total=0;
    for(unsigned i = 0; i < mask.size(); ++i) {
      if(mask[i]) {
        std::vector<int> v;
        v.push_back(total);
        _port_map.push_back(std::make_pair(pm[i].first,v));
         ++total;
      }
    }
    _cgra_data.resize(port_cgra_elem());
    assert(_cgra_data.size() > 0);
  }

  uint64_t total_pushed() { return _total_pushed; }

private:
  // cgra_port: vec_loc(offset)
  // 23: 1, 2   24: 3, 4
  bool _isInput;
  int _port=-1;
  int _outstanding=0;
  uint64_t _highest_stream_id=0;
  STATUS _status=STATUS::FREE;
  LOC _loc=LOC::NONE;
  std::vector<std::pair<int, std::vector<int> > > _port_map;    //loc_map
  std::deque<SBDT> _mem_data;
  std::vector<std::deque<SBDT>> _cgra_data; //data per port
  unsigned _num_in_flight=0; 
  unsigned _num_ready=0;

  uint64_t _total_pushed=0;
};

//Entire Port interface with each being port_data_t
class port_interf_t {
public:

  void initialize(SbModel* sbconfig);
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

struct data_buffer_base {
  static const int length = 128;

  bool can_push(int size) {
    return size + _data.size() <= length;
  }

  int data_ready() {
    return _data.size() - _just_pushed;
  }
  int empty_buffer() {
    return _data.size() ==0;
  }
  void push_data(std::vector<SBDT>& in_data) {
    for(SBDT i : in_data) {
      _data.push_back(i);
    }
    _just_pushed=in_data.size();
  }
  SBDT pop_data() {
    SBDT item = _data[0];
    _data.pop_front();
    return item;
  }
  void finish_cycle() {
    _just_pushed=0;
  }

  int size() {
    return _data.size();
  }

protected:
  std::deque<SBDT> _data;
  int _just_pushed=0;
};

struct data_buffer : public data_buffer_base {
  bool can_push_addr(int size, addr_t addr) {
    if(size + _data.size() <= length) {
      if((_data.size()==0) ||
         (_dest_addr+DATA_WIDTH*_data.size()==addr)) {
        return true;
      }
    }
    return false;
  }

  bool push_data(addr_t addr, std::vector<SBDT>& in_data) {
    if(!can_push(in_data.size())) {
      return false;
    }
    if(_data.size()==0) {
      _dest_addr=addr;
      data_buffer_base::push_data(in_data);      
    } else if(_dest_addr+DATA_WIDTH*_data.size()==addr) {
      data_buffer_base::push_data(in_data);      
    } else {
      return false;
    }
    return true;
  }

  SBDT pop_data() {
    _dest_addr+=DATA_WIDTH;
    return data_buffer_base::pop_data();
  }

  addr_t dest_addr() {
    return _dest_addr;
  }
protected:
  uint64_t _dest_addr;
};


// Each controller forwards up to one "block" of data per cycle.
class data_controller_t {
  public:
  data_controller_t(ssim_t* host) {
    _sb=host;
  }
  ssim_t* _sb;
};



//Limitations: 1 simultaneously active scratch stream
class dma_controller_t : public data_controller_t {
  friend class scratch_write_controller_t;
  friend class scratch_read_controller_t;

  public:

  static const int data_width=64; //data width in bytes
  //static const int data_sbdts=data_width/SBDT; //data width in bytes
  std::vector<bool> mask;

  dma_controller_t(ssim_t* host) : data_controller_t(host) {
    //Setup DMA Controllers, eventually this should be configurable
    _dma_port_streams.resize(10);
    _indirect_streams.resize(4);
    _indirect_wr_streams.resize(4);
    _port_dma_streams.resize(4); // IS THIS ENOUGH?
    _tq_read = _dma_port_streams.size()+1/*dma->scr*/+_indirect_streams.size();
    _tq = _tq_read + _port_dma_streams.size()+1+_indirect_wr_streams.size();
    
    //set everything to be empty
    for(auto& i : _dma_port_streams) {i.reset();}
    for(auto& i : _indirect_streams) {i.reset();}
    for(auto& i : _indirect_wr_streams) {i.reset();}
    for(auto& i : _port_dma_streams) {i.reset();}
    _dma_scr_stream.reset();

    _prev_port_cycle.resize(64); //resize to maximum conceivable ports
    mask.resize(MEM_WIDTH/DATA_WIDTH);
  }

  void cycle();
  void finish_cycle();
  bool done(bool show, int mask);

  int req_read(mem_stream_base_t& stream, uint64_t scr_addr);
  void req_write(port_dma_stream_t& stream, port_data_t& vp);

  void ind_read_req(indirect_stream_t& stream, uint64_t scr_addr);
  void ind_write_req(indirect_wr_stream_t& stream);

  void make_request(unsigned s, unsigned t, unsigned& which);

  void print_status();
  void cycle_status();

  dma_scr_stream_t& dma_scr_stream() {return _dma_scr_stream;}

  bool schedule_dma_port(dma_port_stream_t& s);
  bool schedule_dma_scr(dma_scr_stream_t& s);
  bool schedule_port_dma(port_dma_stream_t& s);
  bool schedule_indirect(indirect_stream_t&s);
  bool schedule_indirect_wr(indirect_wr_stream_t&s);

  int scr_buf_size() {return _scr_write_buffer.size();}
  int mem_reqs() {return _mem_read_reqs + _mem_write_reqs;}

  private:

  void port_resp(unsigned i);

  unsigned _which_read=0;
  unsigned _which=0;
  unsigned _tq, _tq_read;

//  std::priority_queue<base_stream_t*, std::vector<base_stream_t*>, Compare> pq;

  //This ordering defines convention of checking
  std::vector<dma_port_stream_t> _dma_port_streams;  //reads
  dma_scr_stream_t _dma_scr_stream;  
  data_buffer _scr_write_buffer;
  data_buffer _scr_read_buffer; 

  std::vector<port_dma_stream_t> _port_dma_streams; //writes

  std::vector<indirect_stream_t> _indirect_streams; //indirect reads
  std::vector<indirect_wr_stream_t> _indirect_wr_streams; //indirect reads
 
  struct fake_mem_req {
    addr_t scr_addr;
    int port=-1;
    std::vector<SBDT> data;
    bool last;
    uint64_t orig_cmd_id;
    fake_mem_req(addr_t a, int p, std::vector<SBDT>& d, bool l, int id)
      : scr_addr(a), port(p), data(d), last(l) {
      static uint64_t ID_SOURCE=0;
      orig_cmd_id=((uint64_t)id)*4294967296+(ID_SOURCE++); 
      assert(d.size());
      assert(data.size());  
      if(SB_DEBUG::MEM_REQ) {
        //_ticker->timestamp();
        if(port == -1) {
          std::cout << "mem_req to scr, scr_addr:" << scr_addr << ", words: " << d.size() << "\n";
        } else {
          std::cout << "mem_req to port, port:" << port << ", words: " << d.size() << "\n";
        }
      }
      
    }
    /*bool operator < (const struct fake_mem_req& other) const {
      return time > other.time;
    }
    bool operator == (const struct fake_mem_req& other) const {
      return false;
    }*/
  };
  //structure for faking memory acces stime
  //address to stream -> [stream_index, data]
  uint64_t _mem_read_reqs=0, _mem_write_reqs=0;
  std::vector<uint64_t> _prev_port_cycle;
  uint64_t _prev_scr_cycle=0;
  int _fake_scratch_reqs=0;

  //std::unordered_map<uint64_t, uint64_t> port_youngest_data;

};

//Limitation: 1 simulteanously active scratch read stream
class scratch_read_controller_t : public data_controller_t {
  public:
  std::vector <bool> mask;

  scratch_read_controller_t(ssim_t* host, dma_controller_t* d) 
    : data_controller_t(host) {
    _dma_c=d; //save this for later
    _scr_port_stream.reset();
    _scr_dma_stream.reset();
    mask.resize(SCR_WIDTH/DATA_WIDTH);
  }

  std::vector<SBDT> read_scratch(mem_stream_base_t& stream);
  void cycle();

  void finish_cycle();
  bool done(bool,int);


  bool schedule_scr_port(scr_port_stream_t& s);
  bool schedule_scr_dma(scr_dma_stream_t& s);

  void print_status();
  void cycle_status();

  scr_dma_stream_t& scr_dma_stream() {return _scr_dma_stream;}

  private:
  int _which=0;

  scr_dma_stream_t _scr_dma_stream;
  scr_port_stream_t _scr_port_stream;
  dma_controller_t* _dma_c;  
};


class scratch_write_controller_t : public data_controller_t {
  public:
  std::vector<bool> mask;

  scratch_write_controller_t(ssim_t* host, dma_controller_t* d) 
    : data_controller_t(host) {
    _dma_c=d;
    _port_scr_stream.reset();
    mask.resize(SCR_WIDTH/DATA_WIDTH);
  }

  void cycle();
  void finish_cycle();
  bool done(bool,int);

  void print_status();
  void cycle_status();

  bool schedule_port_scr(port_scr_stream_t& s);

  private:
  int _which=0;
  port_scr_stream_t _port_scr_stream;
  dma_controller_t* _dma_c;
};


class port_controller_t : public data_controller_t {
  public:
  port_controller_t(ssim_t* host) : data_controller_t(host) {
    _port_port_streams.resize(4);  //IS THIS ENOUGH?
    _const_port_streams.resize(4);  //IS THIS ENOUGH?
    for(auto& i : _port_port_streams) {i.reset();}
    for(auto& i : _const_port_streams) {i.reset();}
  }

  void cycle();
  void finish_cycle();
  bool done(bool,int);

  bool schedule_port_port(port_port_stream_t& s);
  bool schedule_const_port(const_port_stream_t& s);

  void print_status();
  void cycle_status();


  private:
  unsigned _which_pp=0;
  unsigned _which_cp=0;

  std::vector<port_port_stream_t> _port_port_streams;
  std::vector<const_port_stream_t> _const_port_streams;

  //std::deque<port_port_stream_t> _port_port_queue;
  //std::deque<const_port_stream_t> _const_port_queue;
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
    vol_by_len[ilog2(vol)]+=vol;
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
      out << base_stream_t::name_of((LOC)(i.first.first)) << "->"
          << base_stream_t::name_of((LOC)(i.first.second)) << ": ";
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

//class accel_t
//{
//  friend class ssim_t;
//  friend class ticker_t;
//  friend class scratch_read_controller_t;
//  friend class scratch_write_controller_t;
//  friend class dma_controller_t;
//  friend class port_port_controller_t;
// 
//
//
//};




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

  void roi_entry(bool enter);
  void set_not_in_use(); // tell softsim to not compute things: ) 
  bool in_use();
  void timestamp(); //print timestamp
  uint64_t now(); //return's sim's current time

  //Stats Interface
  void print_stats();
  void pedantic_statistics(std::ostream&);
  void print_statistics(std::ostream&);
  void reset_statistics();
  void print_status();

  void cycle_status();
  void clear_cycle();

  void set_debug(bool value) { debug = value; }
  bool get_debug() { return debug; }

  // Interface from instructions to streams
  // IF SB_TIMING, these just send the commands to the respective controllers
  // ELSE, they carry out all operations that are possible at that point
  void req_config(addr_t addr, int size);
  void cfg_port(uint64_t config, uint64_t in_port) {} //new -- define
  void configure(addr_t addr, int size, uint64_t* bits);
  void load_dma_to_scratch(addr_t mem_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t scratch_addr);
  void write_dma_from_scratch(addr_t scratch_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, addr_t mem_addr); //new
  void load_dma_to_port(addr_t mem_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, int port);
  void load_scratch_to_port(addr_t scratch_addr, uint64_t stride, 
      uint64_t access_size, uint64_t num_strides, int in_port);
  void write_scratchpad(int out_port, addr_t scratch_addr, 
                        uint64_t num_bytes, uint64_t shift_bytes); //new
  void write_dma(uint64_t garb_elem, //new
      int out_port, uint64_t stride, uint64_t access_size, 
      uint64_t num_strides, addr_t mem_addr, int shift_bytes, int garbage);
  void reroute(int out_port, int in_port, uint64_t num_elem);
  void indirect(int ind_port, int ind_type, int in_port, addr_t index_addr,
    uint64_t num_elem);
  void indirect_write(int ind_port, int ind_type, int out_port, 
    addr_t index_addr, uint64_t num_elem);
  bool can_receive(int out_port);
  uint64_t receive(int out_port);
  void write_constant(int num_strides, int in_port, 
                      SBDT constant, uint64_t num_elem, 
                      SBDT constant2, uint64_t num_elem2, 
                      uint64_t flags); //new
  port_interf_t& port_interf() {
    return _port_interf;
  }

  void set_cur_minst(Minor::MinorDynInstPtr m) {
    assert(m);
    _cur_minst=m;
  }
  void step();
  bool done(bool show = false, int mask = 0);

  bool set_in_config() {return _in_config = true;}
  bool is_in_config() {return _in_config;}
  bool can_add_stream();

  uint64_t forward_progress_cycle() { return _forward_progress_cycle; }

  Minor::MinorDynInstPtr cur_minst() {return _cur_minst;}

  void process_stream_stats(base_stream_t& s) {
    uint64_t    vol  = s.data_volume();
    uint64_t    reqs = s.requests();
    STR_PAT     t  = s.stream_pattern();
    _stream_stats.add(t,s.src(),s.dest(),vol,reqs);
  }


private:
  Minor::LSQ* _lsq;
  std::ofstream in_port_verif, out_port_verif, scr_wr_verif, scr_rd_verif, cmd_verif;

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

  void cycle_cgra();   //Tick on each cycle

  void cycle_in_interf();
  void cycle_out_interf();
  void schedule_streams();
  bool cgra_done(bool, int mask);

  void verif_cmd(base_stream_t* s) {
    if(SB_DEBUG::VERIF_CMD) {
      cmd_verif << s->short_name();
      cmd_verif << s->mem_addr()     << " ";   
      cmd_verif << s->access_size()  << " ";   
      cmd_verif << s->stride()       << " ";   
      cmd_verif << s->scratch_addr() << " ";   
      cmd_verif << s->num_strides()  << " ";   
      cmd_verif << s->num_bytes()    << " ";   
      cmd_verif << s->constant()     << " ";   
      cmd_verif << s->in_port()      << " ";   
      cmd_verif << s->out_port()     << " ";   
      cmd_verif << s->wait_mask()    << " ";   
      cmd_verif << s->shift_bytes()  << "\n";
    }
  }

  void add_port_based_stream(base_stream_t* s) {
    s->set_id();
    assert(_cur_minst);
    s->set_minst(_cur_minst);
    _in_port_queue.push_back(s);
    forward_progress();
    verif_cmd(s);
  }

  void add_dma_port_stream(dma_port_stream_t* s)     {add_port_based_stream(s);} 
  void add_port_dma_stream(port_dma_stream_t* s)     {add_port_based_stream(s);} 
  void add_scr_port_stream(scr_port_stream_t* s)     {add_port_based_stream(s);} 
  void add_port_scr_stream(port_scr_stream_t* s)     {add_port_based_stream(s);} 
  void add_port_port_stream(port_port_stream_t* s)   {add_port_based_stream(s);} 
  void add_indirect_stream(indirect_base_stream_t* s){add_port_based_stream(s);} 
  void add_const_port_stream(const_port_stream_t* s) {add_port_based_stream(s);} 


  void add_dma_scr_stream(dma_scr_stream_t* s) {
    if(_sbconfig->dispatch_inorder()) {
      add_port_based_stream(s);
    } else {
      _dma_scr_queue.push_back(s);
      s->set_minst(_cur_minst);
      forward_progress();
      verif_cmd(s);
    }
  }

  void add_scr_dma_stream(scr_dma_stream_t* s) {
    if(_sbconfig->dispatch_inorder()) {
      add_port_based_stream(s);
    } else {
      _scr_dma_queue.push_back(s);
      s->set_minst(_cur_minst);
      forward_progress();
      verif_cmd(s);
    }
  }


  bool can_add_dma_port_stream()   {return _in_port_queue.size()  < _queue_size;} 
  bool can_add_port_dma_stream()   {return _in_port_queue.size()  < _queue_size;}
  bool can_add_scr_port_stream()   {return _in_port_queue.size()  < _queue_size;}
  bool can_add_port_scr_stream()   {return _in_port_queue.size()  < _queue_size;}
  bool can_add_port_port_stream()  {return _in_port_queue.size()  < _queue_size;}
  bool can_add_indirect_stream()   {return _in_port_queue.size()  < _queue_size;}
  bool can_add_const_port_stream() {return _in_port_queue.size()  < _queue_size;}

  bool can_add_dma_scr_stream()    {
    if(_sbconfig->dispatch_inorder()) {
      return _in_port_queue.size()  < _queue_size;
    } else {
      return _dma_scr_queue.size()  < _queue_size;
    }
  }
  bool can_add_scr_dma_stream()    {
    if(_sbconfig->dispatch_inorder()) {

      return _in_port_queue.size()  < _queue_size;
    } else {      
      return _scr_dma_queue.size()  < _queue_size;
    }
  }


  void do_cgra();
  void execute_pdg(unsigned instance);

  void forward_progress() {
    _waiting_cycles=0; 
    _forward_progress_cycle=now();
  }

  //members------------------------
  ticker_t* _ticker=NULL;
  soft_config_t _soft_config;
  port_interf_t _port_interf;

  bool _in_config=false;

  Minor::MinorDynInstPtr _cur_minst;

  SbModel* _sbconfig = NULL;
  Schedule* _sched   = NULL;
  SbPDG*    _pdg     = NULL;

  std::vector<uint8_t> scratchpad;     

  unsigned scratch_line_size = 16;                //16B line 
  unsigned fifo_depth = 32;  // probably not needed in functional model
  bool debug;
  unsigned _queue_size=16;

  // Controllers
  dma_controller_t _dma_c;
  scratch_read_controller_t _scr_r_c;
  scratch_write_controller_t _scr_w_c;
  port_controller_t _port_c;

  unsigned _outstanding_scr_read_streams=0;


  std::list<base_stream_t*> _in_port_queue;
  std::list<dma_scr_stream_t*> _dma_scr_queue;
  std::list<scr_dma_stream_t*> _scr_dma_queue;

  std::map<uint64_t,std::vector<int>> _cgra_output_ready;

  //std::deque<base_stream_t> _scratch_queue;
  //std::deque<mem_stream_base_t> _dma_queue;

  //* Stats
  uint64_t _stat_comp_instances = 0;
  uint64_t _stat_scratch_read_bytes = 0;
  uint64_t _stat_scratch_write_bytes = 0;
  uint64_t _stat_scratch_reads = 0;
  uint64_t _stat_scratch_writes = 0;

  uint64_t _stat_start_cycle = 0;
  uint64_t _stat_stop_cycle = 0;
  uint64_t _stat_commands_issued = 0;

  uint64_t _stat_tot_mem_fetched=0;
  uint64_t _stat_tot_mem_stored=0;

  uint64_t _stat_tot_loads=0;
  uint64_t _stat_tot_stores=0;
  uint64_t _stat_tot_mem_store_acc=0;
  uint64_t _stat_tot_mem_load_acc=0;
  
  uint64_t _roi_cycles=0;

  //Cycle stats
  std::map<int,int> _stat_ivp_put;
  std::map<int,int> _stat_ivp_get;
  std::map<int,int> _stat_ovp_put;
  std::map<int,int> _stat_ovp_get;
  int _stat_mem_bytes_wr=0;
  int _stat_mem_bytes_rd=0;
  int _stat_scr_bytes_wr=0;
  int _stat_scr_bytes_rd=0;
  int _stat_cmds_issued=0;
  int _stat_cmds_complete=0;
  int _stat_sb_insts=0;


  std::map<SB_CONFIG::sb_inst_t,int> _total_histo;
  std::map<int,int> _vport_histo;

  stream_stats_t _stream_stats;  

  //Stuff for tracking stats
  uint64_t _waiting_cycles=0;
  uint64_t _forward_progress_cycle=0;
};

#if 0
  void add_dma_port_stream(dma_port_stream_t s)     {_dma_port_queue.push_back(s);}
  void add_dma_scr_stream(dma_scr_stream_t s)       {_dma_scr_queue.push_back(s); }
  void add_port_dma_stream(dma_port_stream_t s)     {_port_dma_queue.push_back(s);}
  void add_scr_dma_stream(dma_scr_stream_t s)       {_scr_dma_queue.push_back(s); }
  void add_scr_port_stream(scr_port_stream_t s)     {_scr_port_queue.push_back(s);}
  void add_port_scr_stream(scr_port_stream_t s)     {_port_scr_queue.push_back(s);}
  void add_port_port_stream(port_port_stream_t s)   {_port_port_streams.push_back(s);}
  void add_const_port_stream(const_port_stream_t s) {_const_port_streams.push_back(s);}

  void can_add_scr_port_stream()   {return _scr_port_queue.size() < _queue_size;}
  void can_add_dma_port_stream()   {return _dma_port_queue.size() < _queue_size;}
  void can_add_dma_scr_stream()    {return _dma_scr_queue.size() < _queue_size;}
  void can_add_port_dma_stream()   {return _port_dma_queue.size() < _queue_size;}
  void can_add_scr_dma_stream()    {return _scr_dma_queue.size() < _queue_size;}
  void can_add_port_scr_stream()   {return _port_scr_queue.size() < _queue_size;}
  void can_add_port_port_stream()  {return _port_port_queue.size() < _queue_size;}
  void can_add_const_port_stream() {return _const_port_queue.size() < _queue_size;}
#endif

#endif
