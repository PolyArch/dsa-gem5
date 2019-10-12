#include "accel.hh"
#include "stream.hh"
#include "sim-debug.hh"

int base_stream_t::ID_SOURCE=0;

void base_stream_t::set_empty(bool b) {
  assert(b && "only goes one way for now");
  assert(!_empty && "can only empty once");

  if(SS_DEBUG::COMMAND_O) {
    if(b == true) {
      // timestamp();
      std::cout << "COMPLETED: "; 
    } 
    print_status();
  }

  _empty=b;
}

void base_stream_t::print_in_ports() {
  for(int i = 0; i < _in_ports.size();++i) {
    std::cout << soft_port_name(_in_ports[i], true) << " ";
  }
}

std::string base_stream_t::soft_port_name(int x, bool is_input) {
  std::ostringstream oss;
  oss << x;
  auto &vec = is_input ? _soft_config->in_ports_name : _soft_config->out_ports_name;
  if (_soft_config && x < vec.size()) {
    oss << "(" << vec[x] << ")";
  }
  return oss.str();
}
