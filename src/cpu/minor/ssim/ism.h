#pragma once

#include <cstdint>
#include <vector>

#include "dsa/debug.h"

// Indirect Stream State Machine

class accel_t;
struct base_stream_t;

namespace dsa {
namespace sim {
namespace stream {

struct AffineStatus;

/*
 * for (j = 0; j < L2D; ++j) {
 *   ptr = array.pop() + j * I2D;
 *   n = length.pop();
 *   for (i = 0; i < n + j * E1D; ++i) {
 *     addr = SAR + (ptr + (index.pop() + i) * I1D) * dtype;
 *     if (value) val = value->pop();
 *     auto &ref = *reinterpret_cast<uint##dtype##_t*>(addr);
 *     ref = operation(ref, val);
 *   }
 * }
 */
struct IndirectFSM {
  /*!
   * \brief The attributes of an indirect stream.
   */
  struct FSMAttr {
    /*!
     * \brief The port that feeds this attribute. If 0, the stream will be instantiated implicitly.
     */
    int port{-1};
    /*!
     * \brief The number of bytes of each element in this stream.
     */
    int dtype{8};
    /*!
     * \brief The current value of this stream.
     */
    int64_t value{0};
    /*!
     * \brief The index of this stream in stream.
     */
    int64_t i{0};
    /*!
     * \brief The stream that feeds this attribute instantiated according to the port.
     */
    base_stream_t *stream{nullptr};
  };

  std::vector<FSMAttr> attrs;

  /*!
   * \brief idx[i].
   */
  FSMAttr &idx() { return attrs[0]; };
  /*!
   * \brief idx[i].
   */
  FSMAttr &array() { return attrs[1]; };
  /*!
   * \brief ptr[j].
   */
  FSMAttr &length() { return attrs[2]; };
  /*!
   * \brief Can be null, value[i].
   */
  FSMAttr &value() { return attrs[3]; };

  /*!
   * \brief Redundant information. The number of dimensions.
   */
  int dimension;
  /*!
   * \brief SAR.
   */
  int64_t start_offset;
  /*!
   * \brief E2D.
   */
  int64_t stretch;
  /*!
   * \brief I2D.
   */
  int64_t stride2d;
  /*!
   * \brief I1D.
   */
  int64_t stride1d;
  /*!
   * \brief If the stream is penetrated by index.
   */
  bool penetrate{false};
  /*!
   * \brief If the stream is associate with index.
   */
  bool associate{false};

  /*!
   * \brief Get the value address generated from accelerator.
   * \param accel The accelerator this state machine belongs to.
   * \return A 2-long vector: [index, value].
   */
  std::vector<int64_t> poll(accel_t *accel, bool pop, AffineStatus &as);
  /*!
   * \brief If we have next address to generte.
   *        0: stream has no next;
   *        1: stream has next;
   *       -1: stream cannot make progress because of port starving.
   */
  int hasNext(accel_t *accel);

  /*!
   * \brief For the purpose of debugging, dump this instance in text format.
   */
  std::string toString(accel_t *accel);

  /*!
   * \brief Gather the output ports of this stream.
   */
  std::vector<int> oports() const {
    std::vector<int> res;
    for (auto &attr : attrs) {
      if (attr.port != -1) {
        res.push_back(attr.port);
      }
    }
    return res;
  }

  IndirectFSM(int dim, int64_t sar, int64_t stretch_, int64_t i2d, int64_t i1d, int p, int a) :
    dimension(dim), start_offset(sar), stretch(stretch_), stride2d(i2d), stride1d(i1d), penetrate(p), associate(a) {}
};

}
}
}
