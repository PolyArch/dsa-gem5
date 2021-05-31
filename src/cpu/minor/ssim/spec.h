#pragma once

/*!
 * \file spec.h
 * @author PolyArch Research Group
 * \brief The specification of the hardware.
 * @version 0.1
 * \date 2021-01-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

namespace dsa {

struct Specification {
  /*!
   * \brief The number of spatial lanes.
   */
  int num_of_lanes{8};
  /*!
   * \brief Cacheline width in bytes.
   */
  int cache_line{64};
  /*!
   * \brief The granularity of the decomposable spatial data path. By default it is
   *        byte decomposable.
   */
  int dsa_granularity{1};
  /*!
   * \brief The number of multipliers that can be composed.
   *        By default, 4 means, 0, 1, 2, 3, is 2^0, 2^1, 2^3, 2^3.
   *        8-byte can be composed to byte, dual-byte, word, dual-word.
   */
  int dsa_composability{4};
  /*!
   * \brief The number of concurrent buffet streams.
   */
  int dsa_buffet_slots{4};

  // TODO(@were): If it is a good idea that all the accelerators have the same number for this?
  /*!
   * \brief The size of the dsa commands that can be buffered.
   */
  int cmd_queue_size{16};
  /*!
   * \brief The width of issue window.
   */
  int cmd_issue_width{4};
  /*!
   * \brief If the issue is out of order.
   */
  bool cmd_issue_ooo{true};
};

}