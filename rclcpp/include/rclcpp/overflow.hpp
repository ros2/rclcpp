// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__OVERFLOW_HPP_
#define RCLCPP__OVERFLOW_HPP_

#include <algorithm>
#include <limits>
#include <type_traits>

// Detect if the built_<op>_overflow is defined on the system. If this is not
// the case, detect if we can use Windows's own implementation.
// TODO(dorezyuk) check if apple is handled gracefully or if we need to
//  include https://opensource.apple.com/source/xnu/xnu-3789.41.3/libkern/os/overflow.h.auto.html
#if defined(__has_builtin)
#if __has_builtin(__builtin_add_overflow) && \
  __has_builtin(__builtin_sub_overflow) && \
  __has_builtin(__builtin_mul_overflow)
// Following the checks from the <linux/overflow.h> we first check if the
// __has_builtin marco is defined and use it to check the presence of
// __builtin_<op>_overflow functions.
#define RCLCPP__BUILTIN_OVERFLOW

#endif  // __has_builtin(__builtin_add_overflow) ...
#endif  // defined(__has_builtin)

#ifndef RCLCPP__BUILTIN_OVERFLOW
#if defined(__GNUC__) && ((__GNUC__ > 5) || (__GNUC__ == 5) && (__GNUC_MINOR__ >= 1))
// Following the documentation from
// https://www.gnu.org/software/gcc/gcc-5/changes.html and the implementation
// from <linux/overflow.h> we check if the current version of gcc is at least
// 5.1 to support __builtin_<op>_overflow checks
#define RCLCPP__BUILTIN_OVERFLOW

#else
// We cannot use any of the provided functions and want to fallback to a
// custom implementation
#define RCLCPP__CUSTOM_OVERFLOW

#endif  // selector logic
#endif  // RCLCPP__BUILTIN_OVERFLOW

#ifdef RCLCPP__CUSTOM_OVERFLOW
// We want to give the user to force to use the custom implementation. This is
// primarly used for testing

template<
  typename T,
  typename std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
constexpr bool
custom_add_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");
  if (x > std::numeric_limits<T>::max() - y) {
    return true;
  }
  *res = x + y;
  return false;
}

template<
  typename T,
  typename std::enable_if_t<std::is_signed<T>::value, int> = 0>
constexpr bool
custom_add_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");

  if (((y > 0) && (x > std::numeric_limits<T>::max() - y)) ||
    ((y < 0) && (x < std::numeric_limits<T>::min() - y)))
  {
    return true;
  }
  *res = x + y;
  return false;
}

template<
  typename T,
  typename std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
constexpr bool
custom_sub_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");

  if (x < y) {
    return true;
  }
  *res = x - y;
  return false;
}

template<
  typename T,
  typename std::enable_if_t<std::is_signed<T>::value, int> = 0>
constexpr bool
custom_sub_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");

  if (((y > 0) && (x < std::numeric_limits<T>::min() + y)) ||
    ((y < 0) && (x > std::numeric_limits<T>::max() + y)))
  {
    return true;
  }
  *res = x - y;
  return false;
}

template<
  typename T,
  typename std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
constexpr bool
custom_mul_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");

  if ((y != 0) && (x > std::numeric_limits<T>::max() / y)) {
    return true;
  }
  *res = x * y;
  return false;
}

template<
  typename T,
  typename std::enable_if_t<std::is_signed<T>::value, int> = 0>
constexpr bool
custom_mul_overflow(T x, T y, T * res) noexcept
{
  static_assert(std::is_integral<T>::value, "must be integral");


  // Only if both operands have the same signum, their product will be always
  // positive. Otherwise their product will always be negative.
  const bool signum_same = (x ^ y) >= 0;

  if (signum_same) {
    // Handle the case, of min specifically, since min = -max + 1.
    // The other cases represent:
    // X * Y > max, where X, Y > 0 --> X > max / Y
    // X * Y > max, where X, Y < 0 --> X < max / Y
    if ((std::min(x, y) == std::numeric_limits<T>::min()) ||
      ((y > 0) && (x > std::numeric_limits<T>::max() / y)) ||
      ((y < 0) && (x < std::numeric_limits<T>::max() / y)))
    {
      return true;
    }
  } else {
    // The cases represent
    // X * Y < min, where X > 0, Y < 0 --> X > min / y.
    // X * Y < min, where X < 0, Y > 0 --> X < min / y.
    if (((y > 0) && (x < std::numeric_limits<T>::min() / y)) ||
      ((y < -1) && (x > std::numeric_limits<T>::min() / y)))
    {
      return true;
    }
  }

  *res = x * y;
  return false;
}

#define __rclcpp_add_overflow(x, y, res) custom_add_overflow(x, y, res)
#define __rclcpp_sub_overflow(x, y, res) custom_sub_overflow(x, y, res)
#define __rclcpp_mul_overflow(x, y, res) custom_mul_overflow(x, y, res)

#elif defined(RCLCPP__BUILTIN_OVERFLOW)
// We have access to the builtin functions.
#define __rclcpp_add_overflow(x, y, res) __builtin_add_overflow(x, y, res)
#define __rclcpp_sub_overflow(x, y, res) __builtin_sub_overflow(x, y, res)
#define __rclcpp_mul_overflow(x, y, res) __builtin_mul_overflow(x, y, res)

#endif  // RCLCPP__CUSTOM_OVERFLOW

template<typename T>
void
check_add_overflow(T x, T y, T * res)
{
  if (__rclcpp_add_overflow(x, y, res)) {
    throw std::overflow_error("addition overflows");
  }
}

template<typename T>
void
check_sub_overflow(T x, T y, T * res)
{
  if (__rclcpp_sub_overflow(x, y, res)) {
    throw std::overflow_error("subtraction overflows");
  }
}

template<typename T>
void
check_mul_overflow(T x, T y, T * res)
{
  if (__rclcpp_mul_overflow(x, y, res)) {
    throw std::overflow_error("multiplication overflow");
  }
}

#endif  // RCLCPP__OVERFLOW_HPP_
