#pragma once

#include <vector>

#include "stream.hh"
#include "loc.hh"

class port_interf_t;

namespace dsa {
namespace sim {

struct BitstreamWrapper;

/*!
 * \brief The base class of determining which streams to be executed.
 */
struct StreamArbiter {
  /*!
   * \brief Determine the streams to be executed according to the state of each port.
   */
  virtual std::vector<base_stream_t*> Arbit(accel_t *accel) = 0;
};

struct RoundRobin : StreamArbiter {
  /*!
   * \brief The last executed streams of round robin.
   */
  std::vector<int> last_executed;

  RoundRobin() : last_executed(std::vector<int>(LOC::TOTAL * 2, 0)) {}

  std::vector<base_stream_t*> Arbit(accel_t *accel) override;
};

}
}
