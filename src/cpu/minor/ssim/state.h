/*!
 * \file state.h
 * @author PolyArch Research Group
 * \brief The configruation states of the DSA.
 * @version 0.1
 * \date 2020-12-28
 * 
 * @copyright Copyright (c) 2020
 */

#pragma once

#include <cstdint>
#include <sstream>

namespace dsa {
namespace sim {

/*!
 * \brief The state of the configuration registers.
 */
struct ConfigState {
  /*!
   * \brief The value of the register.
   */
  uint64_t value;
  /*!
   * \brief The stickiness of this reigster. By stickiness, we mean if this value sticks
   *        its old value after instantiating a data stream.
   */
  bool sticky;
};

/*!
 * \brief The state of the vector ports, both in and out.
 */
struct IVPState {
  /*!
   * \brief If this port will be broadcasted when instantiating the next stream.
   */
  bool broadcast{false};
  /*!
   * \brief The repeat time of each popped value.
   */
  int64_t repeat;
  /*!
   * \brief The change of the repeat time.
   */
  int64_t stretch;
  /*!
   * \brief The period of changing the repeat time.
   */
  int64_t period;

  IVPState(int64_t repeat_ = 1, int64_t stretch_ = 0, int64_t period_ = INT64_MAX) :
    repeat(repeat_), stretch(stretch_), period(period_) {}


  virtual std::string toString() {
    std::ostringstream oss;
    if (repeat != 1) {
      oss << "repeat=" << repeat;
    }
    if (stretch) {
      oss << ", stretch=" << stretch;
    }
    if (period != INT64_MAX) {
      oss << ", period=" << period;
    }
    return oss.str().empty() ? "[default]" : oss.str();
  }
};

namespace rt {

/*!
 * \brief The runtime simulation entry of the port state
 */
struct PortExecState : IVPState {

  PortExecState(const IVPState &vps, int port_) :
    IVPState(vps.repeat, vps.stretch, vps.period), port(port_) {}

  /*!
   * \brief Move forward the state machine.
   * \return If we should pop the head of the current FIFO.
   */
  bool tick() {
    if (repeat) {
      repeat_counter = (repeat_counter + 1) % repeat;
    } else {
      // If it is zero, we should avoid dividen by zero,
      // and just pop the value.
      repeat_counter = 0;
    }
    if (repeat_counter == 0) {
      stretch_counter = (stretch_counter + 1) % period;
      if (stretch_counter == 0) {
        repeat += stretch;
      }
      return true;
    }
    return false;
  }

  std::string toString() override {
    std::ostringstream oss;
    oss << "port=" << port;
    if (repeat != 1) {
      oss << ", repeat=" << repeat_counter << "/" << repeat;
    }
    if (stretch) {
      oss << ", stretch=" << stretch;
    }
    if (period != INT64_MAX) {
      oss << ", " << stretch_counter << "/" << period;
    }
    return oss.str();
  }

  /*! \brief The current state machine. */
  int64_t stretch_counter{0};
  /*! \brief The number of times repeated. */
  int64_t repeat_counter{0};
  /*! \brief The port involved. */
  int port;
};

}

}
}