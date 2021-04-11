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
   * \brief The granularity of memory address. By default, it is byte addressable.
   */
  int memory_addressable{1};
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
  int dsa_buffet_slots;
};

}