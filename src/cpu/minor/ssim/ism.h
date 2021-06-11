#include <cstdint>

// Indirect Stream State Machine

class accel_t;

namespace dsa {
namespace stream {

struct IndirectFSM {
  /*!
   * \brief The array address of indirect memory access.
   */
  int64_t start;
  /*!
   * \brief The iterative variable for the inner-most dimension.
   */
  int i;
  /*!
   * \brief The source port of index.
   */
  int idx_port;
  /*!
   * \brief The length of the inner dimension.
   */
  int length_port;
  /*!
   * \brief The offset for each inner dimension.
   */
  int offset_port;
};

}
}