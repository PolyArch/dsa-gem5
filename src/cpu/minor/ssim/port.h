#include <cstdint>
#include <queue>

#include "dsa-ext/spec.h"
#include "stream.hh"
#include "state.h"

class accel_t;

namespace dsa {
namespace sim {

/*!
 * \brief The runtime simulation entry of the port state
 */
struct PortExecState : IVPState {

  PortExecState(const IVPState &vps, int port_) :
    IVPState(vps.repeat, vps.stretch), port(port_) {}

  /*!
   * \brief Move forward the state machine.
   * \return If we should pop the head of the current FIFO.
   */
  bool tick();

  std::string toString() override;

  /*! \brief The number of times repeated. */
  int64_t repeat_counter{0};
  /*! \brief The port involved. */
  int port;
};


// TODO(@were): Use this to replace port_data_t
class VectorPort {
public:

  /*!
   * \brief The accelerator this port belongs to.
   */
  accel_t *parent;

  /*!
   * \brief The size of the value buffer.
   */
  int value_buffer_size;

  /*!
   * \brief The size of the data buffer.
   */
  std::deque<uint8_t> buffer;

  /*!
   * \brief The size of the crossbar buffer.
   */
  int xbar_buffer_size;

  /*!
   * \brief The data in the buffer of the crossbar is ready to fire to the spatial arch.
   */
  std::deque<std::vector<uint64_t>> xbar;

  /*!
   * \brief The stream this port currently executes.
   */
  base_stream_t *stream{nullptr};

  /*!
   * \brief The number of outstanding data requests.
   */
  int outstanding{0};

  /*!
   * \brief A stream can be broadcast to multiple ports.
   *        This stores port repeat information of "this" port.
   */
  PortExecState pes;

  VectorPort(accel_t *parent_, int vbs, int xbs) :
    parent(parent_), value_buffer_size(vbs), xbar_buffer_size(xbs),
    pes(dsa::sim::IVPState(), -1) {}

  /*!
   * \brief Move on the state this port.
   */
  std::vector<uint64_t> tick();

  /*!
   * \brief If we still have space in the FIFO.
   */
  bool bufferAvailable();

};

}
}