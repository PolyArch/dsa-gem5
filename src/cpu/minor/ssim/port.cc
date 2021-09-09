#include "./port.h"


namespace dsa {
namespace sim {

bool PortExecState::tick() {
  if (exec_repeat()) {
    repeat_counter = (repeat_counter + 1) % exec_repeat();
  } else {
    // If it is zero, we should avoid dividen by zero,
    // and just pop the value.
    repeat_counter = 0;
  }
  if (repeat_counter == 0) {
    repeat += stretch;
    return true;
  }
  return false;
}

std::string PortExecState::toString() {
  std::ostringstream oss;
  oss << "port=" << port;
  if (repeat != 1) {
    oss << ", repeat=" << repeat_counter << "/" << exec_repeat()
        << "(" << (double) repeat / (1 << DSA_REPEAT_DIGITAL_POINT) << ")";
  }
  if (stretch) {
    oss << ", stretch=" << stretch
        << "(" << (double) repeat / (1 << DSA_REPEAT_DIGITAL_POINT) << ")";
  }
  return oss.str();
}

std::vector<uint64_t> VectorPort::tick() {
  std::vector<uint64_t> res;
  bool should_pop = pes.exec_repeat() == 0;
  if (pes.exec_repeat()) {
    res = xbar;
    should_pop = pes.tick();
  }
  if (should_pop) {
    xbar.clear();
  }
  if (xbar.empty()) {

  }
  return res;
}


}
}