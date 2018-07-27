#ifndef __SS_STREAM_H__
#define __SS_STREAM_H__

#include <cstdint>
#include "consts.hh"
#include <iostream>
#include "loc.hh"

#include "cpu/minor/dyn_inst.hh" //don't like this, workaround later (TODO)




//This is a hierarchical classification of access types
enum class STR_PAT {PURE_CONTIG, SIMPLE_REPEATED, 
                        SIMPLE_STRIDE, OVERLAP, CONST, REC, IND,
                        NONE, OTHER, LEN};

//1.DMA -> Port    or    2.Port -> DMA
struct base_stream_t {
  static int ID_SOURCE;

  virtual bool stream_active() = 0;
  bool empty() {return _empty;} //This must be set by controller itself
  
  void reset() {
    _empty=true;
  }

  void set_empty(bool b);

  virtual LOC src() {return LOC::NONE;}
  virtual LOC dest() {return LOC::NONE;}

  static std::string loc_name(LOC loc) {
    std::string a;
    if(loc==LOC::NONE)                     
    if((loc&LOC::DMA)!=LOC::NONE)         return a+="DMA";
    if((loc&LOC::SCR)!=LOC::NONE)         return a+="SCR";
    if((loc&LOC::PORT)!=LOC::NONE)        return a+="PORT";
    if((loc&LOC::CONST)!=LOC::NONE)       return a+="CONST";
    if((loc&LOC::REMOTE_PORT)!=LOC::NONE) return a+="REM_PORT";
    if((loc&LOC::REMOTE_SCR)!=LOC::NONE)  return a+="REM_SCR";
    if((loc&LOC::REC_BUS)!=LOC::NONE)     return a+="REC_BUS";
    return "NONE";
  }

  static std::string loc_short_name(LOC loc) {
    std::string a;
    if(loc==LOC::NONE)                     
    if((loc&LOC::DMA)!=LOC::NONE)         return a+="D";
    if((loc&LOC::SCR)!=LOC::NONE)         return a+="S";
    if((loc&LOC::PORT)!=LOC::NONE)        return a+="P";
    if((loc&LOC::CONST)!=LOC::NONE)       return a+="C";
    if((loc&LOC::REMOTE_PORT)!=LOC::NONE) return a+="R";
    if((loc&LOC::REMOTE_SCR)!=LOC::NONE)  return a+="Q";
    if((loc&LOC::REC_BUS)!=LOC::NONE)     return a+="B";
    return "?";
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
  virtual void print_status() {
    if(stream_active())  std::cout << "               ACTIVE";
    else                 std::cout << "             inactive";
 
    if(empty())          std::cout << "EMPTY!\n";
    else                 std::cout << "\n";
  }

  virtual int ivp_dest() {return -1;}

  void set_id() {_id=++ID_SOURCE;}
  int id() {return _id;}

  void inc_requests() {_reqs++;}
  uint64_t requests()     {return _reqs;}

  virtual uint64_t mem_addr()    {return 0;}  
  virtual uint64_t ctx_offset()  {return _ctx_offset;}  
  virtual int64_t  access_size() {return 0;}  
  virtual int64_t  stride()      {return 0;} 
  virtual uint64_t scratch_addr(){return 0;} 
  virtual uint64_t num_strides() {return 0;} 
  virtual int64_t  stretch()     {return 0;} 
  virtual uint64_t num_bytes()   {return 0;} 
  virtual uint64_t constant()    {return 0;} 
  virtual int64_t in_port()      {return -1;} 
  virtual int64_t out_port()     {return -1;} 
  virtual int64_t val_port()     {return -1;} 
  virtual uint64_t wait_mask()   {return 0;} 
  virtual uint64_t shift_bytes() {return 0;} 
  virtual uint64_t offset_list() {return 0;} 
  virtual uint64_t ind_mult()    {return 1;} 


  virtual int repeat_in()   {return 1;}  
  virtual int repeat_str()   {return 0;}  
  virtual uint32_t fill_mode() {return _fill_mode;} 
  virtual bool stride_fill() {return _fill_mode == STRIDE_DISCARD_FILL ||
                                     _fill_mode == STRIDE_ZERO_FILL;} 

  virtual uint64_t data_volume() {return 0;} 
  virtual STR_PAT stream_pattern() {return STR_PAT::OTHER;} 

  virtual void set_orig() {}

  void set_minst(Minor::MinorDynInstPtr m) {_minst=m;}
  Minor::MinorDynInstPtr minst() {return _minst;}

  void set_fill_mode(uint32_t mode) {_fill_mode = mode;}
  void set_context_offset(uint64_t offset) {_ctx_offset = offset;}

protected:
  int      _id=0;
  uint32_t _fill_mode=0; //0: none, 1 post-zero fill, 2 pre-zero fill (not implemented)
  bool _empty=false; //presumably, when we create this, it won't be empty
  Minor::MinorDynInstPtr _minst;
  uint64_t _reqs=0;
  uint64_t _ctx_offset=0;
};



static inline uint64_t ilog2(const uint64_t x) {
  uint64_t y;
  asm ("\tbsr %1, %0\n" : "=r"(y) : "r" (x));
  return y;
}

//This class represents a barrier (does not stall core) 
struct stream_barrier_t : public base_stream_t {
  uint64_t _mask=0;

  bool bar_scr_rd() {return _mask & WAIT_SCR_RD;}
  bool bar_scr_wr() {return _mask & WAIT_SCR_WR;}

  virtual bool stream_active() {return true;}

  virtual void print_status() {
    if(bar_scr_rd()) {
      std::cout << " read_barrier";
    }
    if(bar_scr_wr()) {
      std::cout << " write_barrier";
    }
    std::cout << std::hex << " mask=0x" << _mask << std::dec << "\n";
  }

};

struct mem_stream_base_t : public base_stream_t {
  uint64_t _context_bitmask=0; //
  int64_t _access_size;    // length of smallest access
  int64_t _stride;         // length of entire slide
  addr_t _stretch;        // stretch of access size over time
  addr_t _bytes_in_access=0; // bytes in access completed

  // vidushi:
  // int access_ind_stream_port=-1; 
  // int mem_ind_stream_port=-1; 
  
  addr_t _mem_addr;     // CURRENT address of stride
  addr_t _num_strides=0;  // CURRENT strides left
  addr_t _orig_strides=0;

  mem_stream_base_t() {}

  mem_stream_base_t(addr_t mem_addr, uint64_t stride, 
    uint64_t access_size, int stretch, uint64_t num_strides) {
    _mem_addr=mem_addr;
    _stride=stride;
    _access_size=access_size;
    _stretch=stretch;
    _num_strides=num_strides;
  }

  virtual void set_orig() {
    _orig_strides = _num_strides;
  }

  int _shift_bytes=0;

  virtual uint64_t mem_addr()    {return _mem_addr;}  
  virtual int64_t  access_size() {return _access_size;}  
  virtual int64_t  stride()      {return _stride;} 
  virtual int64_t  stretch()     {return _stretch;} 
  virtual uint64_t num_strides() {return _num_strides;} 
  virtual uint64_t shift_bytes() {return _shift_bytes;} 

  virtual STR_PAT stream_pattern() {
    if(_access_size==0 || _orig_strides==0) {
      return STR_PAT::NONE;
    } else if(_access_size == _stride || _orig_strides == 1) {
      return STR_PAT::PURE_CONTIG;
    } else if(_stride > _access_size) { //know _orig_strides > 1
      return STR_PAT::SIMPLE_STRIDE;
    } else if(_stride == 0) {
      return STR_PAT::SIMPLE_REPEATED;
    } else { 
      return STR_PAT::OVERLAP;
    }
  } 

  virtual uint64_t data_volume() { //TODO/FIXME: THIS IS NOT VALID FOR STRETCH==0
    return _orig_strides * abs_access_size();
  }

  addr_t cur_addr() { 
    //return _mem_addr + _bytes_in_access;
    if(_num_strides==0) {
      return 0;
    } else {
      if(_access_size>=0) {
        return _mem_addr + _bytes_in_access;
      } else {
        return _mem_addr - _bytes_in_access - DATA_WIDTH;
      }
    }
  }

  bool stride_hit() {
    return _bytes_in_access==0;
  }

  int64_t abs_access_size() {
    return _access_size>0?_access_size:-_access_size;
  }

  //Return next address
  addr_t pop_addr() { 
    assert(_access_size!=0);
    if(!stream_active()) {
      assert(0 && "inactive stream popped");
    }

    if(_shift_bytes==2) {
      _bytes_in_access+=2;
    } else {
      _bytes_in_access+=DATA_WIDTH;
    }

    if(_bytes_in_access==abs_access_size()) { // go to next stride
      _bytes_in_access=0;
      _mem_addr+=_stride;
      _num_strides--;
      _access_size+=_stretch; 
    }
    assert((_bytes_in_access<abs_access_size()
            || _access_size==0) && "something went wrong");

    return cur_addr();
  }

  virtual bool stream_active() {
    return _num_strides!=0 && _access_size !=0;
  } 
};

//.........STREAM DEFINITION.........
struct dma_port_stream_t : public mem_stream_base_t {
  int _in_port;           //source or destination port
  int _repeat_in=1, _repeat_str=0;

  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}

  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t access_size()  {return _access_size;}  
  int64_t stride()       {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  int64_t in_port()     {return _in_port;} 

  virtual LOC src() {return LOC::DMA;}
  virtual LOC dest() {return LOC::PORT;}

  virtual int ivp_dest() {return _in_port;}

  virtual bool stream_active() {
    return mem_stream_base_t::stream_active();
  }

  virtual void print_status() {  
    std::cout << "dma->port" << "\tport=" << _in_port << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " mem_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides 
              << " repeat_in=" << _repeat_in << " stretch=" << _stretch;
    base_stream_t::print_status();
  }

};

struct port_dma_stream_t : public mem_stream_base_t {
  int _out_port;           //source or destination port
  int _garbage;

  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t access_size()  {return _access_size;}  
  int64_t stride()       {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  int64_t out_port()    {return _out_port;} 
  uint64_t garbage()     {return _garbage;} 

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::DMA;}

  virtual void print_status() {  
    std::cout << "port->dma" << "\tport=" << _out_port << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " mem_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides;
    base_stream_t::print_status();
  }

};

//3. DMA -> Scratch    
struct dma_scr_stream_t : public mem_stream_base_t {
  int _scratch_addr; //CURRENT scratch addr

  dma_scr_stream_t() {}

  dma_scr_stream_t(addr_t mem_addr, uint64_t stride, uint64_t access_size, 
      int stretch, uint64_t num_strides, addr_t scratch_addr) : 
         mem_stream_base_t(mem_addr,stride,access_size,stretch,
        num_strides) {
    _scratch_addr=scratch_addr;
    set_orig();
  }

  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t access_size()  {return _access_size;}  
  int64_t stride()       {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  uint64_t scratch_addr(){return _scratch_addr;} 

  virtual LOC src() {return LOC::DMA;}
  virtual LOC dest() {return LOC::SCR;}  

  virtual void print_status() {  
    std::cout << "dma->scr" << "\tscr=" << _scratch_addr 
              << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " mem_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides;
    base_stream_t::print_status();
  }

};

//3. Scratch -> Scratch    
struct scr_scr_stream_t;
struct scr_scr_stream_t : public mem_stream_base_t {
  uint64_t _scratch_addr; //CURRENT scratch addr
  bool _is_source;
  bool _is_ready;

  scr_scr_stream_t* _remote_stream;
  uint64_t          _remote_bitmask;

  scr_scr_stream_t() {}

  scr_scr_stream_t(addr_t mem_addr, uint64_t stride, uint64_t access_size, 
      int stretch, uint64_t num_strides, addr_t scratch_addr, bool is_src) : 
         mem_stream_base_t(mem_addr,stride,access_size,stretch,
        num_strides) {
    _scratch_addr=scratch_addr;

    _is_source = is_src;
    _is_ready=false;

    set_orig();
  }

  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t  access_size() {return _access_size;}  
  int64_t  stride()      {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  uint64_t scratch_addr(){return _scratch_addr;} 

  void set_remote(scr_scr_stream_t* r, uint64_t r_bitmask) {
    _remote_bitmask=r_bitmask;
    _remote_stream=r;
  }
  virtual LOC src() {
    if(_is_source) return LOC::SCR;
    else          return LOC::REMOTE_SCR;
  }
  virtual LOC dest() {
    if(_is_source) return LOC::REMOTE_SCR;
    else          return LOC::SCR;
  }

  virtual void print_status() {  
    if(_is_source) {
      std::cout << "scr->remote_scr";
    } else {
      std::cout << "remote_scr->scr";
    }

    std::cout << "\tscr_addr=" << _scratch_addr 
              << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " mem_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides;

    base_stream_t::print_status();
  }

};

//4.Scratch -> DMA 
struct scr_dma_stream_t : public mem_stream_base_t {
  addr_t _dest_addr; //current dest addr

  scr_dma_stream_t() {}
  scr_dma_stream_t(addr_t scr_addr, uint64_t stride, uint64_t access_size, 
                  uint64_t num_strides, addr_t mem_addr) :
       mem_stream_base_t(scr_addr, //don't worry this is correct
           stride,access_size,0/*stretch*/,num_strides) {
    _dest_addr  =mem_addr;    //... this too
    _num_strides=num_strides;
    set_orig();
  }

  //NOTE/WEIRD/DONOTCHANGE: 
  uint64_t mem_addr()    {return _mem_addr;}  //referse to memory addr
  int64_t  access_size() {return _access_size;}  
  int64_t  stride()      {return _stride;}
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
//  uint64_t scratch_addr(){return _mem_addr;} 

  virtual LOC src() {return LOC::SCR;}
  virtual LOC dest() {return LOC::DMA;}

  virtual void print_status() {  
    std::cout << "scr->dma" << "\tscr=" << _mem_addr
              << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " dest_addr=" << std::hex << _dest_addr << std::dec 
              << " strides_left=" << _num_strides;
    base_stream_t::print_status();
  }
};


//4. Scratch->Port     
struct scr_port_stream_t : public mem_stream_base_t {
  int _in_port;
  int _repeat_in=1, _repeat_str=0;

  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}

  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t access_size() {return _access_size;}  
  int64_t stride()      {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
//  uint64_t scratch_addr(){return _scratch_addr;} 
  int64_t in_port()     {return _in_port;} 

  virtual LOC src() {return LOC::SCR;}
  virtual LOC dest() {return LOC::PORT;}

  virtual int ivp_dest() {return _in_port;}

  virtual bool stream_active() {
    return mem_stream_base_t::stream_active();
  }

  virtual void print_status() {  
    std::cout << "scr->port" << "\tport=" << _in_port << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " scr_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides;
    base_stream_t::print_status();
  }
};


struct scr_port_base_t : public base_stream_t {
  addr_t _scratch_addr; // CURRENT address
  addr_t _num_bytes=0;  // CURRENT bytes left
  addr_t _orig_bytes=0;  // bytes left
  int _repeat_in=1, _repeat_str=0;

  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}


  virtual uint64_t scratch_addr(){return _scratch_addr;} 
  virtual uint64_t num_bytes()   {return _num_bytes;} 

  virtual uint64_t data_volume() {return _orig_bytes;}
  virtual STR_PAT stream_pattern() {return STR_PAT::PURE_CONTIG;}

  virtual void set_orig() {
    _orig_bytes = _num_bytes;
  }

  //Return next address
  addr_t pop_addr() {
    _scratch_addr+=DATA_WIDTH; 
    _num_bytes-=DATA_WIDTH;
    return _scratch_addr;
  }

  virtual bool stream_active() {
    return _num_bytes>0;
  }  

};


// 5. Port -> Scratch -- TODO -- NEW -- CHECK
struct port_scr_stream_t : public scr_port_base_t {
  int _out_port;
  uint64_t _shift_bytes; //new unimplemented

  uint64_t scratch_addr(){return _scratch_addr;} 
  uint64_t num_bytes()   {return _num_bytes;} 
  int64_t  out_port()    {return _out_port;} 
  uint64_t shift_bytes()    {return _shift_bytes;} 

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::SCR;}

  virtual void print_status() {  
    std::cout << "port->scr" << "\tport=" << _out_port
              << "\tscr_addr=" << _scratch_addr 
              << " bytes_left=" << _num_bytes << " shift_bytes=" << _shift_bytes;
    base_stream_t::print_status();
  }

};

//Constant -> Port
struct const_port_stream_t : public base_stream_t {
  int _in_port;

  addr_t _constant;
  addr_t _num_elements=0;
  addr_t _constant2;
  addr_t _num_elements2=0;
  addr_t _iters_left=0;

  //running counters
  addr_t _elements_left;
  addr_t _elements_left2;
  addr_t _num_iters;

  addr_t _orig_elements;

  void check_for_iter() {
    if(!_elements_left && !_elements_left2 && _iters_left) {
      _iters_left--;
      _elements_left=_num_elements;
      _elements_left2=_num_elements2;
    }
  }

  virtual void set_orig() {
    _iters_left=_num_iters;
    _elements_left=0;
    _elements_left2=0;
    check_for_iter();
  }

  uint64_t constant()    {return _constant;} 
  int64_t in_port()     {return _in_port;} 
  uint64_t num_strides() {return _num_elements;} 

  virtual int ivp_dest() {return _in_port;}

  virtual LOC src() {return LOC::CONST;}
  virtual LOC dest() {return LOC::PORT;}

  virtual uint64_t data_volume() {
    return (_num_elements + _num_elements2) * _num_iters * sizeof(SBDT);
  }
  virtual STR_PAT stream_pattern() {
    return STR_PAT::CONST;
  }

  uint64_t pop_item() {
    check_for_iter();
    if(_elements_left > 0) {
      _elements_left--;
      return _constant;
    } else if(_elements_left2) {
      _elements_left2--;
      return _constant2;
    }
    assert(0&&"should not have popped");
    return 0;
  }

  virtual bool stream_active() {
    return _iters_left!=0 || _elements_left!=0 || _elements_left2!=0;
  }

  virtual void print_status() {  
     std::cout << "const->port" << "\tport=" << _in_port;
     if(_num_elements) {
       std::cout << "\tconst:" << _constant << " left=" << _elements_left 
                 << "/" << _num_elements;
     }
     if(_num_elements2) {
       std::cout << "\tconst2:" << _constant2  << " left=" << _elements_left2
                 << "/"  << _num_elements2;
     }
     std::cout << "\titers=" << _iters_left << "/" << _num_iters << "";

    base_stream_t::print_status();
  }

};






// const->scr
struct const_scr_stream_t : public base_stream_t {

  uint64_t _constant;
  int _num_elements;
  int _iters_left=0; //needs zeroing here, otherwise default can be active
  addr_t _scratch_addr;
    
  virtual void set_orig() {
    _iters_left=_num_elements;
 }

  uint64_t constant()    {return _constant;} 
  uint64_t num_strides() {return _num_elements;} 

  uint64_t scratch_addr(){return _scratch_addr;} 

  virtual LOC src() {return LOC::CONST;}
  virtual LOC dest() {return LOC::SCR;}

  virtual bool stream_active() {
    return _iters_left!=0;
  }

  virtual void print_status() {  
     if(_num_elements) {
       std::cout << "\tconst:" << _constant << " left=" << _iters_left
                 << "/" << _num_elements;
     }
     std::cout << "\titers=" << _iters_left << "/" << _num_elements << "";

    base_stream_t::print_status();
  }

};

//Port -> Port 
struct port_port_stream_t : public base_stream_t {
  port_port_stream_t(){
    _num_elements=0;
  }

  port_port_stream_t(int out_port, int in_port, 
                     uint64_t num_elem, int repeat, int repeat_str) { 
    _out_port=out_port;
    _in_port=in_port;
    _num_elements=num_elem;
    _repeat_in=repeat;
    _repeat_str=repeat_str;
    set_orig();
  }

  int _in_port;
  int _repeat_in=1, _repeat_str=0;

  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}

  int _out_port;
  addr_t _num_elements=0;
  addr_t _orig_elements;

  virtual void set_orig() {
    _orig_elements = _num_elements;
  }

  virtual uint64_t data_volume() {return _num_elements * sizeof(SBDT);}
  virtual STR_PAT stream_pattern() {return STR_PAT::REC;}

  int64_t in_port()     {return _in_port;} 
  int64_t out_port()    {return _out_port;} 
  uint64_t num_strides() {return _num_elements;} 

  virtual int ivp_dest()  {return _in_port;}

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::PORT;}

  virtual bool stream_active() {
    return _num_elements!=0;
  }

  virtual void cycle_status() {
  }

  virtual void print_status() {  
    std::cout << "port->port" << "\tout_port=" << _out_port
              << "\tin_port:" << _in_port  << "\trepeat:" << _repeat_in
              << " elem_left=" << _num_elements;
    base_stream_t::print_status();
  }
};

struct remote_port_stream_t;

struct remote_port_stream_t : public port_port_stream_t {
  remote_port_stream_t() {
    _num_elements=0;
  }

  //core describes relative core position (-1 or left, +1 for right)
  remote_port_stream_t(int out_port, int in_port, uint64_t num_elem, 
                       int repeat, int repeat_str, int core, bool is_source) : 
                       port_port_stream_t(out_port,in_port,num_elem,
                           repeat,repeat_str) {
    _is_source = is_source;
    _which_core = core;
    _is_ready=false;
  }

  //this is the only pointer to source stream after source issues
  remote_port_stream_t* _remote_stream; 
  int _which_core = 0;
  bool _is_source = false;
  bool _is_ready = false;

  virtual STR_PAT stream_pattern() {return STR_PAT::REC;}

  int64_t in_port()     {
    if(_is_source) return -1;
    else          return _in_port;
  } 
  int64_t out_port()    {
    if(_is_source) return _out_port;
    else          return -1;
  } 

  virtual LOC src() {
    if(_is_source) return LOC::PORT;
    else          return LOC::REMOTE_PORT;
  }
  virtual LOC dest() {
    if(_is_source) return LOC::REMOTE_PORT;
    else          return LOC::PORT;
  }

  virtual void print_status() {  
    if(_is_source) {
      std::cout << "port->remote";
    } else {
      std::cout << "remote->port";
      if(_is_ready) { std::cout << " (source is ready)";}
      else { std::cout << " (source NOT ready)";}
    }
    std::cout << "\tout_port=" << _out_port
              << "\tin_port:" << _in_port  
              << "\tdir:" << _which_core << "\trepeat:" << _repeat_in
              << "\telem_left=" << _num_elements;

    base_stream_t::print_status();
  }
};


//Indirect Read Port -> Port 
struct indirect_base_stream_t : public base_stream_t {
  int _ind_port;
  int _ind_type, _dtype; //index and data types, TODO: data type must be T64
  addr_t _num_elements;
  addr_t _index_addr;
  uint64_t _offset_list;
  uint64_t _ind_mult;
  std::vector<char> _offsets;
  bool _scratch=false; // Use this to tell the command 
                    // decoder where this stream should go

  addr_t _orig_elements;
  //These get set based on _type
  unsigned _index_bytes, _data_bytes, _indices_in_word;
  uint64_t _index_mask, _data_mask; 

  //Note: since ports hold 64-bits, this is the adapter
  //for indirect read so that it works regardless
  uint64_t _cur_ind_val=0;
  int _ind_bytes_complete=0;

  virtual void set_orig() { //like constructor but lazier
    _orig_elements = _num_elements;
    _index_in_word=0;
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

    _indices_in_word = DATA_WIDTH / _index_bytes;

    //set up offset list 
    _offsets.push_back(0);
    for(int i = 0; i < DATA_WIDTH; i++) {
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

  bool scratch()     {return _scratch;} 


  virtual uint64_t data_volume() {return _num_elements * sizeof(SBDT);} //TODO: config
  virtual STR_PAT stream_pattern() {return STR_PAT::IND;}

  //if index < 64 bit, the index into the word from the port
  unsigned _index_in_word=0; 
  unsigned _index_in_offsets=0; 

  addr_t cur_addr(SBDT val) {    
    uint64_t index =  (val >> (_index_in_word * _index_bytes * 8)) & _index_mask;
    return   _index_addr + index * _ind_mult + _offsets[_index_in_offsets]*_data_bytes;
  }  

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::PORT;}

  virtual bool stream_active() {
    return _num_elements!=0;
  }

  //return value: should pop vector port
  bool pop_elem() {
    _index_in_offsets++;
    //std::cout << "pop ";

    if(_index_in_offsets >= _offsets.size()) {
      _index_in_offsets=0;

      _num_elements--;
      //std::cout << _num_elements << " ";

      _index_in_word++;
      if(_index_in_word >= _indices_in_word) {
        _index_in_word=0;
        //std::cout << "\n";

        return true;
      }
    }
    //std::cout << "\n";

    return false;
  }

  virtual void cycle_status() {
  }
};

//Indirect Read Port -> Port 
struct indirect_stream_t : public indirect_base_stream_t {
  int _in_port;
  int _repeat_in=1, _repeat_str=0;

  virtual int repeat_in() {return _repeat_in;}
  virtual int repeat_str() {return _repeat_str;}
  int64_t  in_port()      {return _in_port;} 

  virtual int ivp_dest() {return _in_port;}

  virtual void print_status() {  
    std::cout << "mem[ind_port]->in_port" << "\tind_port=" << _ind_port
              << "\tind_type:" << _ind_type  << "\tind_addr:" << _index_addr
              << "\tnum_elem:" << _num_elements << "\tin_port" << _in_port
              << "\toffets:" << _offset_list;
    base_stream_t::print_status();
  }
  virtual bool stream_active() {
    return indirect_base_stream_t::stream_active();
  }
  virtual LOC src() {
    if(!scratch()) {
      return LOC::PORT|LOC::DMA;
    } else {
      return LOC::PORT|LOC::SCR;
    }
  }  virtual LOC dest() {return LOC::PORT;}

};

//Indirect Read Port -> Port 
struct indirect_wr_stream_t : public indirect_base_stream_t {
  int _out_port;

  int64_t out_port()     {return _out_port;} 

  uint64_t cur_value(uint64_t val) {
    return (val >> (_ind_bytes_complete * 8)) & _data_mask;
  }

  virtual void print_status() {  
    std::cout << "out_port->mem[ind_port]" << "\tind_port=" << _ind_port
              << "\tind_type:" << _ind_type  << "\tind_addr:" << std::hex <<_index_addr
        << std::dec << "\tnum_elem:" << _num_elements << "\tout_port" << _out_port;
    base_stream_t::print_status();
  }
  
  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {
    if(!scratch()) {
      return LOC::PORT|LOC::DMA;
    } else {
      return LOC::PORT|LOC::SCR;
    }
  }
};


//Indirect Read Port -> SCR
struct atomic_scr_stream_t;
struct atomic_scr_stream_t : public mem_stream_base_t {
  int _val_port;
  int _out_port;
  int _op_code;
  int _value_type;
  int _output_type;
  int _addr_type;
  uint8_t _value_bytes, _addr_bytes, _output_bytes; //, _indices_in_word;
  uint64_t _value_mask, _addr_mask, _output_mask;
  uint64_t _values_in_word;
  uint64_t _addr_in_word;
  uint64_t _cur_val_index;
  uint64_t _cur_addr_index;

  atomic_scr_stream_t() {}
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
  

  // it's datatypes?
  // uint64_t ind_port()     {return _ind_port;} // this is out addr port
  // uint64_t ind_type()     {return _type;} 
  // uint64_t num_strides() {return _num_elements;} // iters
  uint64_t num_strides() {return _num_strides;} // iters
  // uint64_t index_addr() {return _index_addr;} // this is offset
  int64_t out_port()     {return _out_port;} 
  int64_t val_port()     {return _val_port;} // this is the inc
  int op_code()     {return _op_code;} // opcode
  uint64_t mem_addr()    {return _mem_addr;}  
  int64_t  access_size() {return _access_size;}  
  int64_t  stride()      {return _stride;} 

  uint64_t cur_offset(){
    // return (loc >> ((_addr_in_word - _cur_addr_index - 1)*_value_bytes*8)) & _addr_mask;
    // extracting from right (least significant bits)
    return (mem_addr() >> (_cur_addr_index*_addr_bytes*8)) & _addr_mask;
  }
  uint64_t cur_addr(uint64_t loc){
    // return (loc >> ((_addr_in_word - _cur_addr_index - 1)*_value_bytes*8)) & _addr_mask;
    // extracting from right (least significant bits)
    // return (loc >> (_cur_addr_index*_value_bytes*8)) & _addr_mask;
    return (loc >> (_cur_addr_index*_addr_bytes*8)) & _addr_mask;
  }
  uint64_t cur_val(uint64_t val){
    // return (val >> ((_values_in_word - _cur_val_index - 1)*_value_bytes*8)) & _value_mask;
    return (val >> (_cur_val_index*_value_bytes*8)) & _value_mask;
  }
  void inc_val_index(){
    _cur_val_index = (_cur_val_index+1)%(_values_in_word+1);
  }
  void inc_addr_index(){
    _cur_addr_index = (_cur_addr_index+1)%(_addr_in_word+1);
  }

  bool can_pop_val(){
    // std::cout << "_cur_val_index: " << _cur_val_index << " values in word: " << _values_in_word << "\n";
    bool can_pop = (_cur_val_index==_values_in_word);
    // if(can_pop) { _cur_val_index=0; }
    return can_pop;
  }
  bool can_pop_addr(){
    // std::cout << "_cur_addr_index: " << _cur_addr_index << " values in word: " << _addr_in_word << "\n";
    bool can_pop = (_cur_addr_index==_addr_in_word);
    // if(can_pop) { _cur_addr_index=0; }
    return can_pop;

  }

  virtual bool stream_active() {
    return _num_strides!=0;
  }
  // set both src and dest

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::SCR;}

  virtual void print_status() {  
    std::cout << "atomic_scr " << "\tval_port=" << _val_port
              << "\taddr_port:" << _out_port  << "\top_code:" << _op_code << "\titers left: " << _num_strides
         << std::dec << "\tinput_type:" << _value_type << "\toutput_type:" << _output_type << "\taddr_type:" << _addr_type;
    // base_stream_t::print_status();
  }
};



#endif
