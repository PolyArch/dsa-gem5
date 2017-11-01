#include "stream.hh"
#include "sim-debug.hh"


int base_stream_t::ID_SOURCE=0;

void base_stream_t::set_empty(bool b) {
  assert(b && "only goes one way for now");
  assert(!_empty && "can only empty once");

  if(SB_DEBUG::SB_COMMAND_O) {
    if(b == true) {
      std::cout << "COMPLETED: "; 
    } 
    print_status();
  }

  _empty=b;
}


