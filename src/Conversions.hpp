#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include "MathUtils.hpp"
#include <cmath>

namespace Utils{
  static constexpr double deg2grad(double degree){
    return degree * (PI / 180.0f);
  }
} // Utils

#endif // CONVERSIONS_HPP_
