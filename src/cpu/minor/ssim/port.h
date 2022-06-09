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

#include "dsa-ext/rf.h"
#include "dsa-ext/spec.h"
#include "dsa/arch/fabric.h"
#include "dsa/dfg/port.h"
#include "dsa/simulation/data.h"

#include "./linear_stream.h"


struct base_stream_t;
class accel_t;

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
  int64_t broadcast{-1};
  /*!
   * \brief The repeat time of each popped value.
   */
  int64_t repeat;
  /*!
   * \brief The change of the repeat time.
   */
  int64_t stretch;

  IVPState(int64_t repeat_ = 1 << DSA_REPEAT_DIGITAL_POINT,
           int64_t stretch_ = 0) :
    repeat(repeat_), stretch(stretch_) {}

  /*!
   * \brief We adopt a fixed-point repeat port, so we need to calcuate the ceiling.
   */
  int exec_repeat() {
    auto base = (repeat >> DSA_REPEAT_DIGITAL_POINT);
    auto mask = (1 << DSA_REPEAT_DIGITAL_POINT) - 1;
    return base + ((repeat & mask) != 0);
  }

  virtual std::string toString();
};

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

  /*!
   * \brief Dump to text format for logging.
   */
  std::string toString() override;

  /*! \brief The number of times repeated. */
  int64_t repeat_counter{0};
  /*! \brief The port involved. */
  int port;
};

struct Port {
  /*!
   * \brief The accelertor this port belongs to.
   */
  accel_t *parent{nullptr};
  /*!
   * \brief The stream to be executed by this port.
   */
  base_stream_t *stream{nullptr};
  /*!
   * \brief The vector port.
   */
  dfg::VectorPort *vp{nullptr};
  /*!
   * \brief The scalar data type of this port.
   */
  int scalarSizeInBytes() const;
  /*
   * \brief Total bytes of the vector.
   */
  int vectorBytes() const;
  /*!
   * \brief Bind stream to this port to execute.
   */
  void bindStream(base_stream_t *s);
  /*!
   * \brief Free the stream bound to this port.
   */
  void freeStream();
  /*!
   * \brief The ID of this port.
   */
  virtual int id() const = 0;
  /*!
   * \brief Tick the progress of this port.
   */
  virtual void tick() = 0;
  /*!
   * \brief Clear all the ongoing data.
   */
  virtual void reset() = 0;
  /*!
   * \brief If this port is empty.
   */
  virtual bool empty() = 0;
  /*!
   * \brief The vector lanes of this port.
   */
  virtual int vectorLanes() const = 0;
  /*!
   * \brief Data in bytes buffered in this port.
   */
  virtual int bytesBuffered() const = 0;
  /*!
   * \brief Affine stream state buffer.
   *        Input: buffers specific state of the state machine.
   *        Output: buffers state to be penetrated.
   */
  std::vector<uint8_t> state;

  Port(accel_t *a) : parent(a) {}

  virtual ~Port() {}
};

struct PortPacket : SpatialPacket {
  /*!
   * \brief The stream state affiliated to compute instances.
   */
  uint8_t stream_state;

  PortPacket(const SpatialPacket &sp) : SpatialPacket(sp.available_at, sp.value, sp.valid) {}
};

struct InPort : Port {
  /*!
   * \brief The data in bytes that is in ongoing requests.
   */
  int ongoing{0};
  /*!
   * \brief The bytes to pop async.
   */
  int to_pop{0};
  /*!
   * \brief The data ready to be distributed by the crossbar.
   */
  std::deque<PortPacket> buffer;
  /*!
   * \brief The state machine of port data repeat.
   */
  PortExecState pes;
  /*!
   * \brief The size of the FIFO buffer.
   */
  int buffer_size;
  /*!
   * \brief If data can be pushed to the buffer.
   * \param reserve_padding If padding buffer is included. false for requests, true for response.
   */
  int canPush(bool include_padding);
  /*!
   * \brief If there is enough data can be popped to the spatial CGRA.
   * \param n The number of lanes to be ready. If -1, the number of the vector configured.
   */
  bool lanesReady();
  /*!
   * \brief Pop the data.
   * \param n Check if ready.
   */
  void pop(bool check = true);
  /*!
   * \brief Get the data ready.
   * \param n Get n elements once.
   */
  std::vector<PortPacket> poll();
  /*!
   * \brief Push raw data to the FIFO.
   * \param data The data to be pushed.
   * \param as The status of the stream for padding.
   * \param immediate If this pushed value is ready immediately. If not, clear ongoing.
   */
  void push(const std::vector<uint8_t> &data, const stream::AffineStatus &as, bool immediate);
  /*!
   * \brief The ID of this port.
   */
  int id() const override;
  /*!
   * \brief Tick the progress of this port.
   */
  void tick() override;
  /*!
   * \brief Clear all the ongoing data.
   */
  void reset() override;
  /*!
   * \brief If this port is empty.
   */
  bool empty() override;
  /*!
   * \brief The vector lanes of this port.
   */
  int vectorLanes() const override;
  /*!
   * \brief The input port sub-class.
   */
  dfg::InputPort *ivp() const;
  /*!
   * \brief Data in bytes buffered in this port.
   */
  int bytesBuffered() const override;

  InPort(accel_t *a, int size, int id) : Port(a), pes(IVPState(), id), buffer_size(size) {}
};

struct OutPort : Port {
  int id_;

  std::deque<uint8_t> raw;

  OutPort(accel_t *a, int size, int id_) : Port(a), id_(id_) {}
  /*!
   * \brief The ID of this port.
   */
  int id() const override { return id_; }
  /*!
   * \brief Tick the progress of this port.
   */
  void tick() override {}
  /*!
   * \brief Clear all the ongoing data.
   */
  void reset() override { raw.clear(); }
  /*!
   * \brief If this port is empty.
   */
  bool empty() override { return raw.empty(); }
  /*!
   * \brief The vector lanes of this port.
   */
  int vectorLanes() const override;
  /*!
   * \brief If this port has enough bytes to pop.
   * \param n The bytes to pop. If -1, pop as many as it can.
   * \return The number of bytes this port can pop.
   */
  int canPop(int n);
  /*!
   * \brief Get the given bytes from the FIFO.
   * \param n The given bytes to get.
   */
  std::vector<uint8_t> poll(int n);
  /*!
   * \brief Pop the given bytes from the FIFO.
   * \param n The given bytes to pop.
   */
  void pop(int n);
  /*!
   * \brief Push data from spatial architecture to the port FIFO buffer.
   * \param data The data in raw byte format.
   */
  void push(const std::vector<uint8_t> &data);
  /*!
   * \brief The output port sub-class.
   */
  dfg::OutputPort *ovp() const;
  /*!
   * \brief Data in bytes buffered in this port.
   */
  int bytesBuffered() const override;
};

}
}
