/*!
 * \file atomic.h
 * @author PolyArch Research Group
 * \brief The atomic operation implementation.
 * @version 0.1
 * \date 2020-12-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <algorithm>
#include <functional>

namespace dsa {
namespace sim {
namespace atom {

enum Operation {
  Add,
  Sub,
  Mul,
  Div,
  Max,
  Min,
  Total
};

std::function<int64_t(int64_t, int64_t)> FUNC_IMPL[Operation::Total] = {
  [](int64_t a, int64_t b) -> int64_t {
    return a + b;
  },
  [](int64_t a, int64_t b) -> int64_t {
    return a - b;
  },
  [](int64_t a, int64_t b) -> int64_t {
    return a * b;
  },
  [](int64_t a, int64_t b) -> int64_t {
    return a / b;
  },
  [](int64_t a, int64_t b) -> int64_t {
    return std::max(a, b);
  },
  [](int64_t a, int64_t b) -> int64_t {
    return std::min(a, b);
  }
};

}
}
}