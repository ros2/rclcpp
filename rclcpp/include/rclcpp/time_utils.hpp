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

#ifndef RCLCPP__TIME_UTILS_HPP_
#define RCLCPP__TIME_UTILS_HPP_

#include <limits>
#include <stdexcept>
#include <type_traits>

#include "rcl/time.h"

namespace rclcpp
{

/** \defgroup time_conversions Group contains conversion helper functions
 *
 * A positive time stamp of 1234.567000000 sec will be stored as
 * 1243 seconds and 567000000 nano
 *
 * A negative time stamp of -1234.567000000 sec will be stored as
 * -1243 seconds and 567000000 nano
 *  @{
 */

/// Internal implementation of a signum.
/**
 * Name prepended to avoid namespace pollution
 *
 * \tparam T must be arithmetic
 * \param _in value for sign operation
 * \return -1 if _in smaller zero, 1 otherwise.
 */
template<typename T>
inline
std::enable_if_t<std::is_arithmetic<T>::value, T>
__sign(const T & _in) noexcept
{
  return _in < 0 ? -1 : 1;
}

/// Combines signed seconds and unsigned nanoseconds into one field
/**
 * Given seconds and nanoseconds the function merges both into one field.
 *
 * Pseudo-code:
 * output = sec * 1e10 + nanoseconds % 1e10
 *
 * \param sec seconds
 * \param nano nanoseconds
 * \return field containing seconds and nanoseconds
 */
inline int64_t join_to_nano(int32_t sec, uint32_t nano) noexcept
{
  static constexpr uint32_t max_nano = 1000000000;
  // store seconds and nanoseconds in one field
  // the multiplication cannot overflow
  const auto only_nano = static_cast<int64_t>(nano % max_nano) * __sign(sec);
  return RCL_S_TO_NS(static_cast<int64_t>(sec)) + only_nano;
}

/// Split signed data into seconds and nanoseconds
/**
 * Given a full field of nanoseconds the function splits it up into separate
 * fields.
 *
 * Pseudo-code:
 * sec = full / 1e10
 * nano = full % 1e10
 *
 * \param[in] full contains both, seconds and nanoseconds
 * \param[out] sec contains only seconds
 * \param[out] nano contains only nanoseconds
 */
inline void split_from_nano(int64_t full, int32_t & sec, uint32_t & nano) noexcept
{
  static constexpr int64_t max_nano = 1000000000;
  // split up the full nano data into seconds and nanoseconds
  sec = static_cast<int32_t>(RCL_NS_TO_S(full));

  // casting a negative number x into unsigned will lead to
  // std::numeric_limits<unsigned T>::max() - x. Hence, we must take the abs
  if (full < 0) {
    // std::abs or similar will lead to overflow, if we apply
    // std::numeric_limits<T>::min() to it
    if (std::numeric_limits<int64_t>::min() == full) {
      ++full;
    }
    full *= -1;
  }
  nano = static_cast<uint32_t>(full % max_nano);
}

/** @} */  // end of time_conversions

}  // namespace rclcpp

#endif  // RCLCPP__TIME_UTILS_HPP_
