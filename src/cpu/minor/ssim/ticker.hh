#ifndef __EVENT_H__
#define __EVENT_H__

#include <time.h>
#include <cstdint>
#include <iostream>
#include "ssim.hh"

class ticker_t;
class ssim_t;

class ticker_t {
  public:
  ticker_t(ssim_t* sb) : _sb(sb) {
  }

  bool in_roi() {return _in_roi;}

  void roi_entry(bool enter);

  void tick(); //Tick one time

  uint64_t now();

  void timestamp();
  uint64_t elpased_time_in_roi() {return _elapsed_time_in_roi;}

  void set_in_use() {_in_use=true;}
  void set_not_in_use() {_in_use=false;}
  bool in_use() {return _in_use;}

  private:
  ssim_t* _sb=NULL;
  bool _prev_done = true;

  bool _in_roi = true;
  timespec _start_ts, _stop_ts;
  uint64_t _elapsed_time_in_roi=0;
  uint64_t _times_roi_entered=0;

  bool _in_use=false;
};




#endif
