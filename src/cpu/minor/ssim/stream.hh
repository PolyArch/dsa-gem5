#ifndef __SS_STREAM_H__
#define __SS_STREAM_H__

#include <cstdint>
#include "consts.hh"
#include <iostream>

#include "cpu/minor/dyn_inst.hh" //don't like this, workaround later (TODO)


enum class LOC {NONE, DMA, SCR, PORT, CONST}; 

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

  static std::string name_of(LOC loc) {
    switch(loc) {
      case LOC::NONE: return "NONE";
      case LOC::DMA: return "DMA";
      case LOC::SCR: return "SCR";
      case LOC::PORT: return "PORT";
      case LOC::CONST: return "CONST";
    }
    return "XXXXX";
  }

  virtual LOC src() {return LOC::NONE;}
  virtual LOC dest() {return LOC::NONE;}

  virtual std::string short_name() {
    return name_of(src()) + "->" + name_of(dest());
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

  void set_id() {_id=ID_SOURCE++;}
  int id() {return _id;}

  void inc_requests() {_reqs++;}
  uint64_t requests()     {return _reqs;}

  virtual uint64_t mem_addr()    {return 0;}  
  virtual uint64_t access_size() {return 0;}  
  virtual uint64_t stride()      {return 0;} 
  virtual uint64_t scratch_addr(){return 0;} 
  virtual uint64_t num_strides() {return 0;} 
  virtual uint64_t num_bytes()   {return 0;} 
  virtual uint64_t constant()    {return 0;} 
  virtual uint64_t in_port()     {return 0;} 
  virtual uint64_t out_port()    {return 0;} 
  virtual uint64_t wait_mask()   {return 0;} 
  virtual uint64_t shift_bytes() {return 0;} 

  virtual int repeat_in()   {return 1;}  //intentional


  virtual uint64_t data_volume() {return 0;} 
  virtual STR_PAT stream_pattern() {return STR_PAT::OTHER;} 

  virtual void set_orig() {}

  void set_minst(Minor::MinorDynInstPtr m) {_minst=m;}
  Minor::MinorDynInstPtr minst() {return _minst;}

protected:
  int _id=0;
  bool _empty=false; //presumably, when we create this, it won't be empty
  Minor::MinorDynInstPtr _minst;
  uint64_t _reqs=0;
};



static inline uint64_t ilog2(const uint64_t x) {
  uint64_t y;
  asm ("\tbsr %1, %0\n" : "=r"(y) : "r" (x));
  return y;
}

struct wait_stream_t : public base_stream_t {
  uint64_t _mask;
};


struct mem_stream_base_t : public base_stream_t {
  addr_t _access_size;    // length of smallest access
  addr_t _stride;         // length of entire slide
  addr_t _bytes_in_access=0; // bytes in access completed
  
  addr_t _mem_addr;     // CURRENT address of stride
  addr_t _num_strides=0;  // CURRENT strides left
  addr_t _orig_strides=0;

  virtual void set_orig() {
    _orig_strides = _num_strides;
  }

  int _shift_bytes=0;

  virtual uint64_t mem_addr()    {return _mem_addr;}  
  virtual uint64_t access_size() {return _access_size;}  
  virtual uint64_t stride()      {return _stride;} 
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

  virtual uint64_t data_volume() {
    return _orig_strides * _access_size;
  }

  addr_t cur_addr() { 
    //return _mem_addr + _bytes_in_access;
    if(_num_strides==0) {
      return 0;
    } else {
      return _mem_addr + _bytes_in_access;
    }
  }

  //Return next address
  addr_t pop_addr() { 
    if(!stream_active()) {
      assert(0 && "inactive stream popped");
    }

    if(_shift_bytes==2) {
      _bytes_in_access+=2;
    } else {
      _bytes_in_access+=DATA_WIDTH;
    }
    if(_bytes_in_access==_access_size) { // go to next stride
      _bytes_in_access=0;
      _mem_addr+=_stride;
      _num_strides--;
    }
    assert(_bytes_in_access<_access_size && "something went wrong");

    return cur_addr();
  }

  bool stream_active() {
    return _num_strides!=0;
  } 
};

//.........STREAM DEFINITION.........
struct dma_port_stream_t : public mem_stream_base_t {
  int _in_port;           //source or destination port
  int _repeat_in;

  virtual int repeat_in() {return _repeat_in;}

  uint64_t mem_addr()    {return _mem_addr;}  
  uint64_t access_size() {return _access_size;}  
  uint64_t stride()      {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  uint64_t in_port()     {return _in_port;} 

  virtual LOC src() {return LOC::DMA;}
  virtual LOC dest() {return LOC::PORT;}

  virtual int ivp_dest() {return _in_port;}

  virtual void print_status() {  
    std::cout << "dma->port" << "\tport=" << _in_port << "\tacc_size=" << _access_size 
              << " stride=" << _stride << " bytes_comp=" << _bytes_in_access 
              << " mem_addr=" << std::hex << _mem_addr << std::dec 
              << " strides_left=" << _num_strides;
    base_stream_t::print_status();
  }

};

struct port_dma_stream_t : public mem_stream_base_t {
  int _out_port;           //source or destination port
  int _garbage;

  uint64_t mem_addr()    {return _mem_addr;}  
  uint64_t access_size() {return _access_size;}  
  uint64_t stride()      {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
  uint64_t out_port()    {return _out_port;} 
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

  uint64_t mem_addr()    {return _mem_addr;}  
  uint64_t access_size() {return _access_size;}  
  uint64_t stride()      {return _stride;} 
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

//4.Scratch -> DMA  TODO -- NEW -- CHECK
struct scr_dma_stream_t : public mem_stream_base_t {
  int _dest_addr; //current dest addr

  //NOTE/WEIRD/DONOTCHANGE: 
  uint64_t mem_addr()    {return _mem_addr;}  //referse to memory addr
  uint64_t access_size() {return _access_size;}  
  uint64_t stride()      {return _stride;}
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
  int _repeat_in;

  virtual int repeat_in() {return _repeat_in;}

  uint64_t mem_addr()    {return _mem_addr;}  
  uint64_t access_size() {return _access_size;}  
  uint64_t stride()      {return _stride;} 
  uint64_t num_strides() {return _num_strides;} 
  uint64_t shift_bytes() {return _shift_bytes;} 
//  uint64_t scratch_addr(){return _scratch_addr;} 
  uint64_t in_port()     {return _in_port;} 

  virtual LOC src() {return LOC::SCR;}
  virtual LOC dest() {return LOC::PORT;}

  virtual int ivp_dest() {return _in_port;}

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
  int _repeat_in;

  virtual int repeat_in() {return _repeat_in;}

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

  bool stream_active() {
    return _num_bytes>0;
  }  

};


// 5. Port -> Scratch -- TODO -- NEW -- CHECK
struct port_scr_stream_t : public scr_port_base_t {
  int _out_port;
  uint64_t _shift_bytes; //new unimplemented

  uint64_t scratch_addr(){return _scratch_addr;} 
  uint64_t num_bytes()   {return _num_bytes;} 
  uint64_t out_port()    {return _out_port;} 
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
  addr_t _num_elements;
  addr_t _constant2;
  addr_t _num_elements2;
  addr_t _iters_left;

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
  uint64_t in_port()     {return _in_port;} 
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

  bool stream_active() {
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
     std::cout << "\titers=" << _iters_left << "/" << _num_iters << "\n";


    base_stream_t::print_status();
  }

};

//Port -> Port 
struct port_port_stream_t : public base_stream_t {
  int _in_port;
  int _repeat_in;

  int _out_port;
  addr_t _num_elements;
  addr_t _orig_elements;

  virtual void set_orig() {
    _orig_elements = _num_elements;
  }

  virtual uint64_t data_volume() {return _num_elements * sizeof(SBDT);}
  virtual STR_PAT stream_pattern() {return STR_PAT::REC;}

  uint64_t in_port()     {return _in_port;} 
  uint64_t out_port()     {return _out_port;} 
  uint64_t num_strides() {return _num_elements;} 

  virtual int ivp_dest() {return _in_port;}
  virtual int repeat_in() {return _repeat_in;}

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::PORT;}

  bool stream_active() {
    return _num_elements!=0;
  }

  virtual void cycle_status() {
  }

  virtual void print_status() {  
    std::cout << "port->port" << "\tout_port=" << _out_port
              << "\tin_port:" << _in_port  << " elem_left=" << _num_elements;
    base_stream_t::print_status();
  }

};


//Indirect Read Port -> Port 
struct indirect_base_stream_t : public base_stream_t {
  int _ind_port;
  int _type;
  addr_t _num_elements;
  addr_t _index_addr;

  addr_t _orig_elements;

  virtual void set_orig() {
    _orig_elements = _num_elements;
  }

  virtual uint64_t ind_port()     {return _ind_port;} 
  virtual uint64_t ind_type()     {return _type;} 
  virtual uint64_t num_strides() {return _num_elements;} 
  virtual uint64_t index_addr() {return _index_addr;} 

  virtual uint64_t data_volume() {return _num_elements * sizeof(SBDT);} //TODO: config
  virtual STR_PAT stream_pattern() {return STR_PAT::IND;}

  //if index < 64 bit, the index into the word from the port
  unsigned _index_in_word; 

  uint64_t index_mask() {
    switch(_type) {
      case 0: return 0xFFFFFFFFFFFFFFFF;
      case 1: return 0xFFFFFFFF;
      case 2: return 0xFFFF;
      case 3: return 0xFF; 
    }
    assert(0);
    return -1;
  }

  unsigned index_size() {
    switch(_type) {
      case 0: return 8; //bytes
      case 1: return 4;
      case 2: return 2;
      case 3: return 1;
    }
    assert(0);
    return -1;
  }

  addr_t calc_index(SBDT val) {    
    if(_type==0) {
      return val;
    }
    //TODO: index in word is always 0 for now, and index mask is always full mask 
    return (val >> (_index_in_word * index_size())) & index_mask();
  }  

  virtual LOC src() {return LOC::PORT;}
  virtual LOC dest() {return LOC::PORT;}

  bool stream_active() {
    return _num_elements!=0;
  }

  void pop_elem() {
    _index_in_word++;
    if(_index_in_word >= index_size()) {
      _index_in_word=0;
    }
    _num_elements--;
  }

  virtual void cycle_status() {
  }
};

//Indirect Read Port -> Port 
struct indirect_stream_t : public indirect_base_stream_t {
  int _in_port;
  int _repeat_in;

  virtual int repeat_in() {return _repeat_in;}

  uint64_t ind_port()     {return _ind_port;} 
  uint64_t ind_type()     {return _type;} 
  uint64_t num_strides()  {return _num_elements;} 
  uint64_t index_addr()   {return _index_addr;} 
  uint64_t in_port()      {return _in_port;} 

  virtual int ivp_dest() {return _in_port;}

  virtual void print_status() {  
    std::cout << "ind_port->port" << "\tind_port=" << _ind_port
              << "\tind_type:" << _type  << "\tind_addr:" << _index_addr
              << "\tnum_elem:" << _num_elements << "\tin_port" << _in_port;
    base_stream_t::print_status();
  }
};

//Indirect Read Port -> Port 
struct indirect_wr_stream_t : public indirect_base_stream_t {
  int _out_port;

  uint64_t ind_port()     {return _ind_port;} 
  uint64_t ind_type()     {return _type;} 
  uint64_t num_strides() {return _num_elements;} 
  uint64_t index_addr() {return _index_addr;} 
  uint64_t out_port()     {return _out_port;} 

  virtual void print_status() {  
    std::cout << "port->ind_port" << "\tind_port=" << _ind_port
              << "\tind_type:" << _type  << "\tind_addr:" << _index_addr
              << "\tnum_elem:" << _num_elements << "\tin_port" << _out_port;
    base_stream_t::print_status();
  }
};

#endif
