#include "ticker.hh"

using namespace std;

uint64_t ticker_t::now() {
  return _sb->now();
}


void ticker_t::timestamp() {
  cout << std::dec << now() - _sb->_stat_start_cycle << "\t";
}

void ticker_t::roi_entry(bool enter) {
  if(enter) {
    _sb->_stat_start_cycle=now();
    clock_gettime(CLOCK_REALTIME,&_start_ts);
    _in_roi=true;
    _times_roi_entered+=1;
  } else {
    _sb->_stat_stop_cycle=now();
    clock_gettime(CLOCK_REALTIME,&_stop_ts);
    _elapsed_time_in_roi += 1000000000 * (_stop_ts.tv_sec - _start_ts.tv_sec) +
                                          _stop_ts.tv_nsec - _start_ts.tv_nsec;

    _sb->_roi_cycles += _sb->_stat_stop_cycle - _sb->_stat_start_cycle;
    _in_roi=false;
  }
}

void ticker_t::tick() {
  if(!_in_use) {
    return;
  }

  _sb->_dma_c.cycle();
  _sb->_scr_r_c.cycle();
  _sb->_scr_w_c.cycle();
  _sb->_port_c.cycle();
  _sb->cycle_in_interf();
  _sb->cycle_cgra();
  _sb->cycle_out_interf();
  _sb->schedule_streams();

  _sb->_waiting_cycles++;
  _sb->_dma_c.finish_cycle(); 
  
   if(SB_DEBUG::CYC_STAT) {
     if(_in_roi) {
      _sb->cycle_status();
     }
   }
}

