#include <cstdint>

// Indirect Stream State Machine

class accel_t;

namespace dsa {
namespace stream {
namespace indirect {

struct IndirectStateMachine {
  /*!
   * \brief If this stream has next element.
   */
  virtual bool hasNest() = 0;
  /*!
   * \brief The get next address of the address to be requested.
   */
  virtual int64_t poll(accel_t *, bool pop) = 0;
  /*!
   * \brief The data type of the indirect stream.
   */
  int dtype;
};

struct ValueFromPort {
  /*!
   * \brief The source of the value.
   */
  int port;
  /*!
   * \brief The length of port stream.
   */
  int64_t n;
  /*!
   * \brief The value counter.
   */
  int64_t i{0};
  /*!
   * \brief The current value.
   */
  int64_t value{-1};
};

struct Indirect1D : IndirectStateMachine {
  /*!
   * \brief The constant starting address.
   */
  int64_t start;
  /*!
   * \brief The index of this 1d indirect stream.
   */
  ValueFromPort index;
};

struct Indirect2DConstLength : IndirectStateMachine {
  /*!
   * \brief The starting address 2d indirect stream.
   */
  ValueFromPort start;
  /*!
   * \brief The counter of the inner stream.
   */
  int64_t i;
  /*!
   * \brief The constant inner length.
   */
  int64_t length;
  /*!
   * \brief The stretched inner length.
   */
  int64_t stretch;
};

struct Indirect2DPortLength : IndirectStateMachine {
  /*!
   * \brief The starting address 2d indirect stream.
   */
  ValueFromPort start;
  /*!
   * \brief The length of each inner dimension.
   */
  ValueFromPort length;
};


}
}
}