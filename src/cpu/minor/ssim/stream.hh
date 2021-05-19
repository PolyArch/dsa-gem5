#pragma once

#include <cstdint>
#include "consts.hh"
#include <iostream>
#include "loc.hh"
#include "sim-debug.hh"
#include "state.h"

#include "cpu/minor/dyn_inst.hh" //don't like this, workaround later (TODO)

class soft_config_t;
class accel_t;

//This is a hierarchical classification of access types
enum class STR_PAT {
  PURE_CONTIG,
  SIMPLE_REPEATED,
  SIMPLE_STRIDE,
  OVERLAP,
  CONST,
  REC,
  IND,
  NONE,
  OTHER,
  LEN
};

struct base_stream_t;
struct IPortStream;
struct OPortStream;
struct PortPortStream;
struct LinearReadStream;
struct LinearWriteStream;
struct Barrier;
struct ConstPortStream;
struct IndirectReadStream;
struct RecurrentStream;

/*!
 * \brief Bookkeeping information of Buffet streams.
 */
struct BuffetEntry {
  /*!
   * \brief [start, end) address on the scratch pad that is dedicated to
   *        this buffet stream.
   */
  int begin, end;
  /*!
   * \brief The window of data buffered by buffet.
   */
  int front{0}, tail{0};
  /*!
   * \brief The number of elements buffered.
   */
  int occupied{0};
  /*!
   * \brief The starting address of the data buffered.
   */
  int64_t address{0};
  /*!
   * \brief Use stream involved by this buffet.
   */
  IPortStream *use{nullptr};
  /*!
   * \brief Load stream involved by this buffet.
   */
  OPortStream *load{nullptr};

  BuffetEntry(int begin_, int end_) : begin(begin_), end(end_) {}

  /*!
   * \brief Dump to string for debugging.
   */
  std::string toString();

  /*!
   * \brief The number of bytes allocated for this Buffet.
   */
  int Size() {
    return end - begin;
  }

  /*!
   * \brief If we still have space available in the Buffet buffer.
   */
  int SpaceAvailable();

  bool InRange(int64_t addr) {
    return addr >= address && addr < address + occupied;
  }

  /*!
   * \brief Translate the requested absolute address to relative address in buffet.
   */
  int Translate(int64_t addr, bool linebase);

  /*!
   * \brief Append bytes of data to the Buffet FIFO.
   * \param bytes The number of data to occupy the allocated Buffet space.
   */
  void Append(int bytes);

  /*!
   * \brief Append bytes of data to the Buffet FIFO
   * \param bytes The number of data to remove from the allocated Buffet space
   */
  void Shrink(int bytes);

};

namespace dsa {
namespace sim {
namespace stream {

/*!
 * \brief The visitor functor class of the streams supported.
 */
struct Functor {
  virtual void Visit(base_stream_t *);
  virtual void Visit(IPortStream *);
  virtual void Visit(OPortStream *);
  virtual void Visit(PortPortStream *);
  virtual void Visit(LinearReadStream *);
  virtual void Visit(LinearWriteStream *);
  virtual void Visit(Barrier *);
  virtual void Visit(ConstPortStream *);
  virtual void Visit(IndirectReadStream *);
  virtual void Visit(RecurrentStream *);
};

BarrierFlag Loc2BarrierFlag(LOC loc);

}
}
}

//1.DMA -> Port    or    2.Port -> DMA
struct base_stream_t {
  static int ID_SOURCE;

  int on_the_fly{0};

  virtual bool stream_active() = 0;

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) {
    f->Visit(this);
  }

  /*! \brief Dump this stream to debug string. */
  virtual std::string toString() { return ""; }

  void set_empty(bool b);

  virtual LOC src() { return LOC::NONE; }
  virtual LOC dest() { return LOC::NONE; }
  LOC side(bool is_source) { return is_source ? src() : dest(); }

  std::string soft_port_name(int x, bool is_input);

  static void sep(std::string &s) {
    if(s.length()!=0) s+="|";
  }

  static std::string loc_name(LOC loc) {
    return LOC_NAME[loc];
  }

  static std::string loc_short_name(LOC loc) {
    return loc_name(loc);
  }



  virtual std::string short_name() {
    return loc_name(src()) + "->" + loc_name(dest());
  }

  bool check_set_empty() {
    inc_requests(); // check if correct
    if(!stream_active()) {
      set_empty(true);
    }
    return _empty;
  }

  virtual ~base_stream_t() { }
  void print_empty() {
    if(stream_active()) {
      std::cout << "               ACTIVE";
    } else {
      std::cout << "             INACTIVE";
    }
  }
  virtual void print_status() {
    std::cout << short_name() << " " << toString() << std::endl;
  }

  int id() { return _id; }
  uint64_t barrier_mask() { return barrier_mask_; }

  void inc_requests() {_reqs++;}
  uint64_t requests()     {return _reqs;}

  virtual uint64_t mem_addr()    {return 0;}
  virtual uint64_t ctx_offset()  {return _ctx_offset;}
  virtual uint64_t scratch_addr(){return 0;}
  virtual uint64_t num_strides() {return 0;}
  virtual int64_t  stretch()     {return 0;}
  virtual uint64_t num_bytes()   {return 0;}
  virtual int64_t out_port()     {return -1;}
  virtual int64_t val_port()     {return -1;}
  virtual uint64_t shift_bytes() {return 0;}
  virtual uint64_t offset_list() {return 0;}
  virtual uint64_t ind_mult()    {return 1;}
  uint64_t data_width()          {return _data_width;}
  uint64_t partition_size()      {return _part_size;}
  uint64_t active_core_bv()      {return _active_core_bv;}
  int map_pattern()              { return _map_pattern;}
  int num_dist_cores()              { return _num_dist_cores;}
  uint64_t straddle_bytes()    {return _straddle_bytes;}
  uint64_t wait_cycles()    {return _wait_cycles;} // waiting for whole cache line

  // PART_CORE_BANK_REST
  uint64_t get_core_id(addr_t logical_addr);
  addr_t memory_map(addr_t logical_addr, addr_t cur_scr_offset);
  void set_mem_map_config();

  bool timeout() {
    if(_wait_cycles > 50) {
      _wait_cycles = 0;
      return true;
    }
    return false;
  }

  virtual int repeat_in()   {return 1;}
  virtual int repeat_str()   {return 0;}
  virtual bool repeat_flag()   {return false;}

  virtual uint64_t data_volume() {return 0;}
  virtual STR_PAT stream_pattern() {return STR_PAT::OTHER;}

  virtual void set_orig() {}

  void set_minst(Minor::MinorDynInstPtr m) {_minst=m;}
  Minor::MinorDynInstPtr minst() {return _minst;}

  void set_context_offset(uint64_t offset) {_ctx_offset = offset;}

  void set_soft_config(soft_config_t* s) {_soft_config=s;}
  LOC unit() {return _unit;}

  virtual void set_data_width(int d) {_data_width=d;}
  virtual void set_part_size(uint64_t d) {_part_size=d;}
  virtual void set_dist_cores() {_num_dist_cores=_used_cores.size();}
  virtual void push_used_core(int d) {_used_cores.push_back(d);}
  virtual void set_active_core_bv(int d) {_active_core_bv=d;}
  virtual void set_mapping_type(int d) {_map_pattern=d;}
  virtual void set_straddle_bytes(int d) {_straddle_bytes=d;}
  virtual void inc_wait_cycles() {_wait_cycles++;}

  base_stream_t(LOC unit_ = LOC::NONE, uint64_t barrier_mask = 0) :
    _id(++ID_SOURCE), _unit(unit_), barrier_mask_(barrier_mask) {}

  // TODO(@were): reorder this
  int _id=0;
  LOC _unit = LOC::PORT;
  uint64_t barrier_mask_;

protected:

  /*! \brief The width of each element in this stream. */
  int _data_width{DATA_WIDTH};
  uint64_t _part_size=0;
  uint64_t _part_bits=0;
  uint64_t _core_bits=0;
  uint64_t _num_dist_cores=0;
  uint64_t _active_core_bv=0;
  std::vector<int> _used_cores;
  int _map_pattern=0;
  int _straddle_bytes=0; // bytes straddling over cache lines (used only in mem streams as of now!)
  // TODO: add this in all streams
  int _wait_cycles=0; // cycles to wait to get whole cache line at ports for write streams 
  bool _empty=false; //presumably, when we create this, it won't be empty
  Minor::MinorDynInstPtr _minst;
  uint64_t _reqs=0;
  uint64_t _ctx_offset=0;
  soft_config_t* _soft_config=NULL;
};


//return -1 if x == 0
static inline int ilog2(const uint64_t x) {
  if(x==0) return -1;
  uint64_t y;
  asm ("\tbsr %1, %0\n" : "=r"(y) : "r" (x));
  return y;
}

//This class represents a network stream at the remote core
struct remote_core_net_stream_t : public base_stream_t {
  int _addr_port=NET_ADDR_PORT;
  int64_t _val_port=NET_VAL_PORT;
  int64_t _num_elements;
  // can be used later if we want separate ports for linear and banked scratchpad
  remote_core_net_stream_t() : base_stream_t(LOC::NONE, 0) {
    _num_elements=1000; // TODO: see some other constants
  }

  int addr_port() { return _addr_port; }
  int64_t val_port() { return _val_port; }

  virtual bool stream_active() {
     return true;
  }

  virtual void print_status() {
    std::cout << "remote core net stream _num_elem" << _num_elements << "\taddr_port= " << _addr_port << "\tval_port" << _val_port << "\n";
  }
};

//This class represents a barrier (does not stall core)
struct Barrier : public base_stream_t {
  uint64_t _mask=0;
  // int64_t _num_remote_writes=-1;
  bool _scr_type=0; // 0 means banked scratchpad

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  bool bar_scr_wr_df() {return _mask & WAIT_SCR_WR_DF;}
  bool bar_scr_rd() {return _mask & WAIT_SCR_RD;}
  bool bar_scr_wr() {return _mask & WAIT_SCR_WR;}

  bool stream_active() {
    return true;
  }

  void print_status() {
    if (_mask >> DBF_DMAStreams & 1) {
      std::cout << "|Sync DMA";
    }
    if (_mask >> DBF_SPadStreams & 1) {
      std::cout << "|Sync SPAD";
    }
    if (_mask >> DBF_RecurStreams & 1) {
      std::cout << "|Sync Recur";
    }
    if (_mask >> DBF_ReadStreams & 1) {
      std::cout << "|Sync Read";
    }
    if (_mask >> DBF_WriteStreams & 1) {
      std::cout << "|Sync Write";
    }
    if (_mask >> DBF_AtomicStreams & 1) {
      std::cout << "|Sync Atomic";
    }
    std::cout << std::hex << "|mask=0x" << _mask << std::dec << "\n";
  }

};

#include "./linear_stream.h"

using dsa::sim::stream::LinearStream;
using dsa::sim::stream::Linear1D;
using dsa::sim::stream::Linear2D;
using dsa::sim::stream::Linear3D;
using dsa::sim::rt::PortExecState;

/*!
 * \brief Stream that only involves input ports.
 */
struct IPortStream : base_stream_t {
  /*! \brief The status of the ports involved by this input stream. */
  std::vector<PortExecState> pes;

  void print_in_ports();

  LOC dest() override { return LOC::PORT; }

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  /*!
   * \brief Deep copy this object.
   */
  virtual IPortStream *clone() { CHECK(false); return nullptr; }

  IPortStream(LOC unit, uint64_t barrier_flag, const std::vector<PortExecState> &pes_) :
    base_stream_t(unit, barrier_flag | (1 << DBF_ReadStreams)), pes(pes_) {}
};

/*!
 * \brief Stream that only involves output ports.
 */
struct OPortStream : base_stream_t {
  /*!
   * \brief The output port.
   */
  int port;

  LOC src() override { return LOC::PORT; }

  /*!
   * \brief Deep copy this object.
   */
  virtual OPortStream *clone() { CHECK(false); return nullptr; }

  OPortStream(LOC unit, uint64_t barrier_flag, int port_) :
    base_stream_t(unit, barrier_flag | (1 << DBF_WriteStreams)), port(port_) {}

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

};

/*!
 * \brief Stream that only involves both input and output ports.
 */
struct PortPortStream : base_stream_t {
  /*!
   * \brief The output port.
   */
  std::vector<int> oports;
  /*!
   * \brief The status of the ports involved by this input stream.
   */
  std::vector<PortExecState> pes;

  LOC src() override { return LOC::PORT; }

  /*!
   * \brief Deep copy this object.
   */
  virtual PortPortStream *clone() { CHECK(false); return nullptr; }

  PortPortStream(LOC unit, uint64_t barrier_flag,
                 const std::vector<PortExecState> &pes_, const std::vector<int> &oports_) :
    base_stream_t(unit, barrier_flag), oports(oports_), pes(pes_) {}

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

};

struct LinearReadStream : public IPortStream {
  /*!
   * \brief The linear access pattern.
   */
  LinearStream *ls;
  /*!
   * \brief The padding policy.
   */
  int padding;
  /*!
   * \brief The buffet entry to which this stream belongs.
   */
  BuffetEntry *be{nullptr};

  LinearReadStream(LOC unit, LinearStream *ls_, const std::vector<PortExecState> &pes_,
                   int padding_) :
    IPortStream(unit, (1 << dsa::sim::stream::Loc2BarrierFlag(unit)), pes_),
    ls(ls_), padding(padding_) {}

  uint64_t data_volume() override {
    return ls->volume;
  }

  IPortStream *clone() override {
    auto res = new LinearReadStream(*this);
    if (res->be) {
      res->be->use = res;
    }
    return res;
  }

  bool stream_active() override {
    return ls->hasNext();
  }

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  LOC src() override { return unit(); }

  void print_status() override {
    std::cout << toString() << std::endl;
  }

  virtual std::string toString()  {
    std::ostringstream oss;
    oss << short_name() << "\t" << ls->toString();
    for (auto &elem : pes) {
      oss << " " << elem.toString();
    }
    return oss.str();
  }

};

struct GarbageStream : public OPortStream {
  /*!
   * \brief The data type of each element to discard.
   */
  int dtype;
  /*!
   * \brief The number of elements to discard.
   */
  uint64_t n;
  /*!
   * \brief The current state of discarding.
   */
  uint64_t i{0};

  bool stream_active() override {
    return i < n;
  }

  OPortStream *clone() {
    return new GarbageStream(*this);
  }

  GarbageStream(int port, int dtype_, int n_) :
    OPortStream(LOC::NONE, 0, port), dtype(dtype_), n(n_) {}
};

struct LinearWriteStream : public OPortStream {
  /*! \brief The linear access pattern. */
  LinearStream *ls;
  /*! \brief The atomic operation. */
  int operation;
  /*!
   * \brief The buffet entry to which this stream belong.
   */
  BuffetEntry *be{nullptr};

  LinearWriteStream(LOC unit, LinearStream *ls_, int port_, int operation_) :
    OPortStream(unit, 1 << DBF_WriteStreams, port_),
    ls(ls_), operation(operation_) {
  }

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  OPortStream *clone() override {
    auto res = new LinearWriteStream(*this);
    // After dispatching, the buffet entry should be updated accordingly.
    if (res->be) {
      res->be->load = res;
    }
    return res;
  }

  bool garbage() {
    if (src() != LOC::DMA) {
      return false;
    }
    if (auto l1d = dynamic_cast<Linear1D*>(ls)) {
      return l1d->start == 0;
    }
    return false;
  }

  LOC src() override { return LOC::PORT; }
  LOC dest() override { return unit(); }

  virtual std::string toString() {
    std::ostringstream oss;
    oss << short_name() << "\t" << ls->toString() << " out_port=" << soft_port_name(port, false);
    if (garbage()) {
      oss << " garbage";
    }
    return oss.str();
  }

  virtual void print_status() {
    std::cout << toString() << std::endl;
  }

  bool stream_active() override {
    return ls->hasNext();
  }

};

/*!
 * \brief Send a sequence of value to the ports.
 */
struct ConstPortStream : public IPortStream {
  /*!
   * \brief We use a linear stream to generate a sequence of value.
   */
  LinearStream *ls;

  bool stream_active() override {
    return ls && ls->hasNext();
  }

  LOC src() override { return LOC::CONST; }

  IPortStream *clone() override {
    return new ConstPortStream(*this);
  }

  std::string toString() override {
    std::ostringstream oss;
    oss << short_name() << " data width: " << data_width() << " " << ls->toString();
    for (auto &elem : pes) {
      oss << " " << elem.toString();
    }
    return oss.str();
  }

  void print_status() override {
    std::cout << toString() << std::endl;
  }

  /*! \brief The entrance of the visitor pattern. */
  void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  ConstPortStream(const std::vector<PortExecState> &pes = {}, LinearStream *ls_ = nullptr) :
    IPortStream(LOC::CONST, 0, pes), ls(ls_) {}

};


struct IndirectReadStream : public PortPortStream {
  /*!
   * \brief The base address of the indirect array.
   */
  uint64_t start;
  /*!
   * \brief The number of elements from the index port.
   */
  uint64_t len;
  /*!
   * \brief The progress of this stream.
   */
  uint64_t i{0};

  bool stream_active() {
    return i < len;
  }

  LOC src() override { return unit(); }

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  /*!
   * \brief Deep copy this object.
   */
  virtual PortPortStream *clone() {
    return new IndirectReadStream(*this);
  }

  int idx_port() {
    return oports[0];
  }

  std::string toString() override {
    std::ostringstream oss;
    oss << "array:" << start << " "  << "idx_port:" << idx_port();
    for (auto &elem : pes) {
      oss << " " << elem.toString();
    }
    oss << " " << i << "/" << len;
    return oss.str();
  }

  IndirectReadStream(LOC unit, const std::vector<PortExecState> &pes,
                     int idx_port_, uint64_t start_, uint64_t len_) :
                     PortPortStream(unit, 1 << dsa::sim::stream::Loc2BarrierFlag(unit), pes, {idx_port_}),
                     start(start_), len(len_) {}
};


/*!
 * \brief Port to port stream struct
 */
struct RecurrentStream : PortPortStream {
  /*!
   * \brief The "word" size.
   */
  int dtype;
  /*!
   * \brief The current state.
   */
  uint64_t i{0};
  /*!
   * \brief The total number of elements streamed.
   */
  uint64_t n{0};

  /*! \brief The entrance of the visitor pattern. */
  virtual void Accept(dsa::sim::stream::Functor *f) override {
    f->Visit(this);
  }

  RecurrentStream(int dtype_, int oport_, const std::vector<PortExecState> &iports_, uint64_t n_) :
    PortPortStream(LOC::PORT, 1 << DBF_RecurStreams, iports_, {oport_}),
    dtype(dtype_), n(n_) {}

  bool stream_active() override {
    return i < n;
  }

  std::string toString() override {
    std::ostringstream oss;
    oss << oports[0] << " -> " << pes[0].port << " " << i << "/" << n;
    return oss.str();
  }

  void print_status() override {
    std::cout << toString() << std::endl;
  }

  PortPortStream *clone() {
    return new RecurrentStream(*this);
  }

};

// struct remote_port_stream_t : public port_port_stream_t {
//   remote_port_stream_t() {
//     _num_elements=0;
//   }
// 
//   //core describes relative core position (-1 or left, +1 for right)
//   // TODO: sending fix 8-byte for port->remote port, make it configurable
//   remote_port_stream_t(int out_port, const PortExecState &in_port, uint64_t num_elem,
//        int repeat, int repeat_str, int core, bool is_source, int access_size, bool repeat_flag) :
//                        port_port_stream_t(out_port,in_port,num_elem,
//                            repeat,repeat_str, 8, access_size, repeat_flag) { // send default p.w. to be 1, also default repeat 
// 
//     _is_source = is_source;
//     _which_core = core;
//     _is_ready=false;
//   }
// 
//   //this is the only pointer to source stream after source issues
//   remote_port_stream_t* _remote_stream;
//   int _which_core = 0;
//   bool _is_source = false;
//   bool _is_ready = false;
// 
//   virtual STR_PAT stream_pattern() {return STR_PAT::REC;}
// 
//   int64_t out_port()    {
//     if(_is_source) return _out_port;
//     else          return -1;
//   }
// 
//   virtual LOC src() {
//     if(_is_source) return LOC::PORT;
//     else          return LOC::REMOTE_PORT;
//   }
//   virtual LOC dest() {
//     if(_is_source) return LOC::REMOTE_PORT;
//     else          return LOC::PORT;
//   }
// 
//   virtual void print_status() {
//     if(_is_source) {
//       std::cout << "port->remote";
//     } else {
//       std::cout << "remote->port";
//       if(_is_ready) { std::cout << " (source is ready)";}
//       else { std::cout << " (source NOT ready)";}
//     }
//     std::cout << "\tout_port=" << _out_port;
//     print_in_ports();
//     std::cout << "\tdir:" << _which_core << "\trepeat:" << _repeat_in
//               << "\telem_left=" << _num_elements;
// 
//     base_stream_t::print_status();
//   }
// };


//Indirect Read Port -> Port
struct indirect_base_stream_t : public base_stream_t {
  int _ind_port;
  int _ind_type, _dtype; //index and data types
  addr_t _num_elements;
  addr_t _index_addr;
  uint64_t _offset_list;
  uint64_t _ind_mult;
  int _val_num=1; // -1;
  std::vector<char> _offsets;
  int _sstream_size=1; // size of sub-stream: should be extracted from the num_elem port
  int _ssind=0;
  int _sstride=-1, _sacc_size=-1, _sn_port=1; // to prevent NULL (check in assert)
  bool _is_2d_stream=false;
  bool _first_ss_access=true;

  addr_t _orig_elements;
  //These get set based on _type
  unsigned _index_bytes, _data_bytes; // , _indices_in_word;
  uint64_t _index_mask, _data_mask;

  //Note: since ports hold 1-bit, this is the adapter
  //for indirect read so that it works regardless
  // uint64_t _cur_ind_val=0;
  int _ind_bytes_complete=0;

  virtual void set_orig() { //like constructor but lazier
    _orig_elements = _num_elements;
    // _index_in_word=0;
    _index_in_offsets=0;

    switch(_ind_type) {
      case T64: _index_bytes= 8; _index_mask = 0xFFFFFFFFFFFFFFFF;  break;
      case T32: _index_bytes= 4; _index_mask = 0xFFFFFFFF;          break;
      case T16: _index_bytes= 2; _index_mask = 0xFFFF;              break;
      case T08: _index_bytes= 1; _index_mask = 0xFF;                break;
      default: assert(0);
    }
    switch(_dtype) {
      case T64:  _data_bytes= 8; _data_mask = 0xFFFFFFFFFFFFFFFF;  break;
      case T32:  _data_bytes= 4; _data_mask = 0xFFFFFFFF;          break;
      case T16:  _data_bytes= 2; _data_mask = 0xFFFF;              break;
      case T08:  _data_bytes= 1; _data_mask = 0xFF;                break;
      default: assert(0);
    }

    // _indices_in_word = DATA_WIDTH / _index_bytes;
    
    // set up offset list
    _offsets.push_back(0);
    // FIXME: check this!
    // for(int i = 0; i < DATA_WIDTH; i++) {
    for(int i = 0; i < _data_width; i++) {
      char offset = (_offset_list >> i*8) & 0xFF;
      if(offset != 0) {
        _offsets.push_back(offset);
      }
    }
  }

  virtual uint64_t ind_port()     {return _ind_port;}
  virtual uint64_t ind_type()     {return _ind_type;}
  virtual uint64_t num_strides()  {return _num_elements;}
  virtual uint64_t index_addr()   {return _index_addr;}
  virtual uint64_t offset_list()  {return _offset_list;}
  virtual uint64_t ind_mult()     {return _ind_mult;}

  bool scratch()     {return _unit==LOC::SCR;}

  // virtual uint64_t data_volume() {return _num_elements * sizeof(SBDT);} //TODO: config
  virtual uint64_t data_volume() { return _num_elements; }
  virtual STR_PAT stream_pattern() {return STR_PAT::IND;} 

  //if index < 64 bit, the index into the word from the port
  // unsigned _index_in_word=0;
  unsigned _index_in_offsets=0;

  addr_t cur_addr(SBDT val) {
    // uint64_t index =  (val >> (_index_in_word * _index_bytes * 8)) & _index_mask;
    uint64_t index =  val & _index_mask;
    if(SS_DEBUG::MEM_REQ) {
      std::cout << "addr offset: " << _index_addr << " index: " << index << " mult: " << _ind_mult << " ss_ind: " << _ssind << " offset: " << unsigned(_offsets[_index_in_offsets]) << " sstream size: " << _sstream_size << "\n";
      addr_t x = _index_addr + index * _ind_mult + _offsets[_index_in_offsets]*_data_bytes + _ssind*_sstride;
      std::cout << "The computed address is: " << x << "\n";
    }
    // return   _index_addr + index * _ind_mult + _offsets[_index_in_offsets]*_data_bytes + _ssind*_sstride;
    addr_t cur_scr_offset = _index_addr + _offsets[_index_in_offsets]*_data_bytes;
    addr_t addr = index * _ind_mult + cur_scr_offset + _ssind*_sstride;
    return memory_map(addr, cur_scr_offset);
  }

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::PORT;}

  virtual bool stream_active() {
    return _num_elements!=0;
  }

  //return value: should pop vector port
  bool pop_elem() {
    _index_in_offsets++;
    // std::cout << "index in offsets: " << _index_in_offsets << " and offset size: " << _offsets.size() << std::endl;

    if(_index_in_offsets >= _offsets.size()) {
      _index_in_offsets=0;
      _ssind++;
      // if(_sstream_size!=_ssind) std::cout << "sstream: " << _sstream_size << " ssind: " << _ssind << std::endl;
      if(_sstream_size!=_ssind) return false;
      _ssind=0;
      _num_elements--;
      //std::cout << _num_elements << " ";
      return true;
    }
    //std::cout << "\n";

    return false;
  }

  virtual void cycle_status() {
  }
};

//Indirect Read Port -> Port
struct indirect_stream_t : public indirect_base_stream_t {
  int _repeat_in=1, _repeat_str=0;
  addr_t cur_base_addr = 0;
  
  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}


  virtual void print_status() {
    std::cout << "mem[ind_port]->in_port" << "\tscratch? " << scratch()
      << "\tout_port width" << _data_width << "\tind_port=" << _ind_port
              << "\tind_type:" << _ind_type  << "\tind_addr:" << _index_addr
              << "\tnum_elem:" << _num_elements << "\tin_port:" 
              << "\tmult: " << _ind_mult
              << "\t2d_stream? " << _is_2d_stream
              << "\tnum_elem_port: " << _sn_port
              << "\tstride: " << _sstride
              << "\taccess_size: " << _sacc_size
              << "\tcur_ss_ind: " << _ssind
              << "\tsstream_size: " << _sstream_size << std::endl;
    // print_in_ports();
    std::cout << "\toffsets:" << _offset_list;
    base_stream_t::print_status();
  }
  virtual bool stream_active() {
    return indirect_base_stream_t::stream_active();
  }

  // FIXME:CHECKME: this consumes both port->dma_ctrl and dma->port b/w
  virtual LOC src() {
    if(!scratch()) {
      // FIXME:CHECKME
      // return LOC::PORT|LOC::DMA;
      return LOC::DMA;
    } else {
      return (LOC) (LOC::PORT|LOC::SCR);
    }
  }
  virtual LOC dest() { return LOC::PORT; }

};

//Indirect Read Port -> Port
struct indirect_wr_stream_t : public indirect_base_stream_t {
  int _out_port;

  int64_t out_port()     {return _out_port;}

  uint64_t cur_value(uint64_t val) {
    return (val >> (_ind_bytes_complete * 8)) & _data_mask;
  }

  virtual void print_status() {
    std::cout << "out_port->mem[ind_port]" << "\tout_port width" << _data_width << "\tind_port=" << _ind_port // check this!
              << "\tind_type:" << _ind_type  << "\tind_addr:" << std::hex <<_index_addr
        << std::dec << "\tnum_elem:" << _num_elements << "\tout_port" << _out_port;
    base_stream_t::print_status();
  }

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {
    if(!scratch()) {
      return LOC::DMA;
    } else {
      return LOC::SCR;
    }
  }
};

//Port -> Remote Port
//TODO: TO reuse some info, can convert to port_port_stream later
struct remote_port_multicast_stream_t;
struct remote_port_multicast_stream_t : public base_stream_t {
  remote_port_multicast_stream_t() : base_stream_t(LOC::NONE, 0) {
    _num_elements=0;
  }

remote_port_multicast_stream_t(int out_port, int remote_in_port, uint64_t num_elem,
                       int64_t mask) : base_stream_t(LOC::NONE, 0) {
    _remote_port = remote_in_port;
    _core_mask = mask;
    // _is_ready=false;
  }

  LOC src() {return LOC::PORT;}
  LOC dest() {return LOC::NETWORK;}

    // _core_mask = mask;
  // this was core_id instead of the mask
  // bool _is_ready = false;
  // int _which_core = 0;

  int64_t _core_mask = 0;
  int64_t _remote_port = -1;
  int64_t _out_port;
  addr_t _num_elements=0; // TODO: this is num_bytes
  addr_t _orig_elements;

  virtual void set_orig() {
    _orig_elements = _num_elements;
  }

  int64_t out_port()    {return _out_port;}
  uint64_t num_strides() {return _num_elements;}

  virtual STR_PAT stream_pattern() {return STR_PAT::REC;}

  virtual bool stream_active() {
    return _num_elements!=0;
  }

  virtual void cycle_status() {
  }

  // port
  virtual void print_status() {
    std::cout << "port->remote port";
    std::cout << "\tout_port=" << _out_port;
    // print_in_ports();
    std::cout << "\tremote_in_port=" << _remote_port;
    std::cout << "\tmask:" << _core_mask << "\telem_left=" << _num_elements;

    base_stream_t::print_status();
  }

  // virtual LOC src() {return LOC::PORT;}
  // virtual LOC dest() {return LOC::REMOTE_PORT;}
};

struct remote_scr_stream_t;
struct remote_scr_stream_t : public remote_port_multicast_stream_t {
    remote_scr_stream_t() {
    _num_elements=0;
  }

  remote_scr_stream_t(int out_port, int addr_port, addr_t scratch_base_addr, uint64_t num_elem, bool spad_type) :
                       remote_port_multicast_stream_t(out_port,-1,num_elem,-1) {

    // _which_core = core;
    // _is_ready=false;
    _remote_scr_base_addr = scratch_base_addr;
    _scr_type = spad_type;
    _addr_port = addr_port;
  }

  // LOC src() {return LOC::PORT;}
  // LOC dest() {return LOC::NETWORK;}


  addr_t _remote_scr_base_addr = -1;
  bool _scr_type = 0; // 0 means banked scr
  int _addr_port = -1;
  // out_port is val port
  // int64_t _out_port;
  // addr_t _num_elements=0;
  // addr_t _orig_elements;

  virtual void set_orig() {
    _orig_elements = _num_elements;
  }

  int64_t val_port()    {return _out_port;}
  int addr_port()    {return _addr_port;}
  addr_t scratch_base_addr()    {return _remote_scr_base_addr;}
  uint64_t num_strides() {return _num_elements;}

  virtual STR_PAT stream_pattern() {return STR_PAT::REC;}

  virtual bool stream_active() {
    return _num_elements!=0;
  }

  virtual void cycle_status() {
  }

  // port
  virtual void print_status() {
    std::cout << "indirect port->remote scratch";
    std::cout << "\tval_port=" << _out_port;
    std::cout << "\taddr_port=" << _addr_port;
    std::cout << "\tremote_scratch_base_addr:" << _remote_scr_base_addr << "\telem_left=" << _num_elements;

    // base_stream_t::print_status(); // configuration may not have been done yet!
  }

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::REMOTE_SCR;}
};

struct direct_remote_scr_stream_t : public remote_scr_stream_t {
  direct_remote_scr_stream_t() {
    _num_elements=0;
  }
  virtual void set_data_width(int data_width) override {
    _data_width = data_width;
    assert(_access_size >= _data_width);
    assert(_access_size%_data_width==0);
    _max_count = _access_size/_data_width; // assuming we can send max data width at a time
  }

  direct_remote_scr_stream_t(addr_t base_addr, int64_t acc_size, int64_t stride) {
    _num_elements=0;
    _remote_scr_base_addr = base_addr;
    _cur_addr = _remote_scr_base_addr;
    _stride = stride;
    _access_size = acc_size;
  }

  /*
  remote_scr_stream_t(int out_port, int addr_port, addr_t scratch_base_addr, uint64_t num_elem, bool spad_type) :
                       remote_port_multicast_stream_t(out_port,-1,num_elem,-1) {

    // _which_core = core;
    // _is_ready=false;
    _remote_scr_base_addr = scratch_base_addr;
    _scr_type = spad_type;
    _addr_port = addr_port;
  }
  */

  // addr_t _remote_scr_base_addr = -1;
  bool _scr_type = 0; // 0 means banked scr
  int64_t _stride = -1;
  int64_t _access_size = -1;
  addr_t _cur_addr = -1;
  int _count = 0;
  int _max_count = 0;
  // int _addr_port = -1;
  // out_port is val port
  // int64_t _out_port;
  // addr_t _num_elements=0;
  // addr_t _orig_elements;

  virtual void set_orig() override {
    _orig_elements = _num_elements;
  }

  int64_t val_port() override {return _out_port;}
  addr_t scratch_base_addr() {return _remote_scr_base_addr;}
  uint64_t num_strides() override {return _num_elements;}

  virtual STR_PAT stream_pattern() override {return STR_PAT::REC;}

  virtual bool stream_active() override {
    return _num_elements!=0;
  }

  // Oh, 2 dimensions?
  // TODO: use the affine_base_stream for this
  virtual addr_t cur_addr() {
    if(_count == 0) { // the first base addr
      _count++;
    } else if(_count < _max_count) { // the next ones in acc_size dimension
      _cur_addr += _data_width;
      _count++;
    } else {
      _cur_addr = _cur_addr - _access_size + _stride + _data_width;
      _count = 0;
      _count++;
      // _num_strides--;
      _num_elements--;
    }
    return _cur_addr;
  }

  virtual void cycle_status() override {
  }

  // port
  virtual void print_status() override {
    std::cout << "direct port->remote scratch";
    std::cout << "\tval_port=" << _out_port << "(" << soft_port_name(_out_port, false) << ")";
    // std::cout << "\tremote_scratch_base_addr:" << _mem_addr << "\telem_left=" << _num_elements;
    std::cout << "\telem_left=" << _num_elements;
    std::cout << "\tdata_width=" << _data_width;

    base_stream_t::print_status(); // configuration may not have been done yet!
  }

  virtual LOC src() override {return LOC::PORT;}
  virtual LOC dest() override {return LOC::REMOTE_SCR;}
};



//Indirect Read Port -> SCR
struct atomic_scr_stream_t;
struct atomic_scr_stream_t : public base_stream_t {

  // int _atomic_cgra_addr_port=-1;
  // int _atomic_cgra_val_port=-1;
  // int _atomic_cgra_out_port=-1;

  int _val_port;
  int _out_port;
  int _op_code;
  int _value_type;
  int _output_type;
  int _addr_type;
  int _val_num=1; // base_addr to be reused, consume val (Priority=2)
  int _sstream_left=1;
  int _val_sstream_left=1;
  uint64_t _num_strides;
  uint64_t _mem_addr;
  uint8_t _value_bytes, _addr_bytes, _output_bytes;
  uint64_t _value_mask, _addr_mask, _output_mask;
  uint64_t _values_in_word;
  uint64_t _addr_in_word;
  uint64_t _cur_val_index;
  uint64_t _cur_addr_index;

  // TODO: make use to know how many addr to consume for an input value (will
  // be used in broadcast message)
  int _num_updates=1; // values to be reused, consume addr (Priority=1)
  bool _is_update_cnt_port=false;
  int _num_update_port=-1;

  atomic_scr_stream_t() : base_stream_t(LOC::SCR, 0) {}
  virtual void set_orig() { //like constructor but lazier
    switch(_value_type) {
      case T64: _value_bytes= 8; _value_mask = 0xFFFFFFFFFFFFFFFF;  break;
      case T32: _value_bytes= 4; _value_mask = 0xFFFFFFFF;          break;
      case T16: _value_bytes= 2; _value_mask = 0xFFFF;              break;
      case T08: _value_bytes= 1; _value_mask = 0xFF;                break;
      default: assert(0);
    }
    switch(_output_type) {
      case T64: _output_bytes= 8; _output_mask = 0xFFFFFFFFFFFFFFFF;  break;
      case T32: _output_bytes= 4; _output_mask = 0xFFFFFFFF;          break;
      case T16: _output_bytes= 2; _output_mask = 0xFFFF;              break;
      case T08: _output_bytes= 1; _output_mask = 0xFF;                break;
      default: assert(0);
    }
    switch(_addr_type) {
      case T64:  _addr_bytes= 8; _addr_mask = 0xFFFFFFFFFFFFFFFF; break;
      case T32:  _addr_bytes= 4; _addr_mask = 0xFFFFFFFF;         break;
      case T16:  _addr_bytes= 2; _addr_mask = 0xFFFF;             break;
      case T08:  _addr_bytes= 1; _addr_mask = 0xFF;               break;
      default: assert(0);
    }
    _cur_val_index=0;
    _cur_addr_index=0;
    _values_in_word = DATA_WIDTH / _value_bytes;
    _addr_in_word = DATA_WIDTH / _addr_bytes;
  }

  int64_t out_port()     {return _out_port;}
  int64_t val_port()     {return _val_port;} // this is the inc
  int op_code()     {return _op_code;} // opcode

  uint64_t num_strides() {return _num_strides;} // iters
  uint64_t mem_addr()    {return _mem_addr;}

  // this should also consider val sstream left (addr mode or val mode?)
  void inc_done_update() {
    _val_sstream_left--;
    if(_val_sstream_left<=0) { // ==0) { // this can go less than 0
      _sstream_left--;
      _val_sstream_left=_num_updates;
      if(_sstream_left==0) {
        _num_strides--;
        _sstream_left=_val_num;
      }
    }
  }

  void inc_done_val() {
    _sstream_left--;
    /*if(_sstream_left<=0) {
      _sstream_left=_val_num;
    }*/
  }

  void inc_done_addr() {
    _val_sstream_left--;
    /*if(_val_sstream_left<=0) { // ==0) { // this can go less than 0
      _val_sstream_left=_num_updates=-1;
    }*/
  }

  void update_strides() {
    _num_strides--;
    if(_is_update_cnt_port) _num_updates=-1;
    // std::cout << "Reducing num strides, value sstream left: " << _sstream_left << " addr left: " << _val_sstream_left << "\n";
    //i if strides are set to 0, not an issue...
    _val_sstream_left=_num_updates; _sstream_left=_val_num;
    
    /*if(_val_sstream_left==0 && _sstream_left==0) {
    // if(_val_sstream_left==_num_updates && _sstream_left==_val_num) {
    }*/
  }

  // FIXME: this should from most significant (Although doesn't matter much
  // because our operations our idempotent)
  uint64_t cur_offset(){
    // extracting from right (least significant bits)
    // return (mem_addr() >> (_cur_addr_index*_addr_bytes*8)) & _addr_mask;
    // std::cout << "mem addr: " << mem_addr() << " addr in word: " << _addr_in_word << " addr index: " << _cur_addr_index << " addr bytes: " << _addr_bytes << "\n";
    return (mem_addr() >> ((_addr_in_word-_cur_addr_index-1)*_addr_bytes*8)) & _addr_mask;
  }
  uint64_t cur_addr(uint64_t loc){
    addr_t addr = loc*_val_num*_value_bytes;
    /*if(SS_DEBUG::SHOW_CONFIG) {
      std::cout << "input loc: " << loc << " val num: " << _val_num << " value bytes: " << _value_bytes << " computd addr: " << addr << "\n";
    }*/
    // addr += (_val_num-_sstream_left)*_addr_bytes;
    return memory_map(addr, 0);
  }
  uint64_t cur_val(uint64_t val){
    // return (val >> (_cur_val_index*_value_bytes*8)) & _value_mask;
    return (val >> ((_values_in_word-_cur_val_index-1)*_value_bytes*8)) & _value_mask;
  }
  void inc_val_index(){
    // if(_val_sstream_left==_num_updates) // if we are looking for vals this cycle
    if(_sstream_left>0) // if we are looking for vals this cycle
    _cur_val_index = (_cur_val_index+1)%(_values_in_word+1);
  }

  // increment when some data is available
  void inc_addr_index(){
    // if(_sstream_left==_val_num) // if we are looking for addr this cycle
    if(_val_sstream_left>0) // if we are looking for addr this cycle
    _cur_addr_index = (_cur_addr_index+1)%(_addr_in_word+1);
  }

  // can pop only when required data is reached
  bool can_pop_val(){
    // std::cout << "_cur_val_index: " << _cur_val_index << " values in word: " << _values_in_word << "\n";
    // bool end_update = true; // (_val_sstream_left==_num_updates);
    bool can_pop = (_cur_val_index==_values_in_word)  && (_sstream_left==_val_num);
    // if(end_update && _is_update_cnt_port) _num_updates=-1;
    return can_pop;
  }

  // cannot pop if not sufficient data arrived till now (inc index 
  bool can_pop_addr() {
    // std::cout << "_cur_addr_index: " << _cur_addr_index << " values in word: " << _addr_in_word << " sstream left: " << _sstream_left << "\n";
    bool can_pop = (_cur_addr_index==_addr_in_word) && (_val_sstream_left==_num_updates); // && (_sstream_left==_val_num);
    return can_pop;

  }

  virtual bool stream_active() {
    return _num_strides!=0;
  }
  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::SCR;}

  virtual void print_status() {
    std::cout << "atomic_scr " << "\tval_port=" << _val_port
              << "\taddr_port:" << _out_port  << "\top_code:" << _op_code << "\titers left: " << _num_strides
              << std::dec << "\tinput_type:" << (int) _value_bytes << "\toutput_type:" << (int) _output_bytes << "\taddr_type:" << (int) _addr_bytes
              << " num values in vector: " << _val_num << " broadcast updates: " << _num_updates << "\n";
  }

};
