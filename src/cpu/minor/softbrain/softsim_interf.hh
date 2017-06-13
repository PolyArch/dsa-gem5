#ifndef __SOFTSIM_INTERF_H__
#define __SOFTSIM_INTERF_H__

#include "cpu/minor/dyn_inst.hh"

typedef uint64_t addr_t;

/*
 * This is Softbrain's interface to the sourrounding
 * simulator code.  
 */
class softsim_interf_t {
  public:
  virtual uint8_t ld_mem8(addr_t addr,uint64_t& cyc_complete, Minor::MinorDynInstPtr) =0; //Load 8 Bits
  virtual uint64_t ld_mem(addr_t addr,uint64_t& cyc_complete,  Minor::MinorDynInstPtr)  =0; //Load 64 Bits
  virtual void st_mem16(addr_t addr, uint16_t val, uint64_t& cyc_complete,  Minor::MinorDynInstPtr) =0;  
  virtual void st_mem(addr_t addr, uint64_t val, uint64_t& cyc_complete,  Minor::MinorDynInstPtr)   =0;

  virtual void inform_cycle(uint64_t cycle) = 0;
};


#endif
