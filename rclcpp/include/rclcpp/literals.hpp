// Copyright 2016 Open Source Robotics Foundation, Inc.
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


#ifndef RCLCPP__LITERALS_HPP_
#define RCLCPP__LITERALS_HPP_

#include <chrono>

namespace rclcpp
{
namespace literals
{

// NOLINTNEXTLINE(runtime/int)
constexpr std::chrono::seconds operator"" _s(unsigned long long s)
{
  return std::chrono::seconds(s);
}
constexpr std::chrono::duration<long double> operator"" _s(long double s)
{
  return std::chrono::duration<long double>(s);
}

// NOLINTNEXTLINE(runtime/int)
constexpr std::chrono::milliseconds operator"" _ms(unsigned long long ms)
{
  return std::chrono::milliseconds(ms);
}
constexpr std::chrono::duration<long double, std::milli> operator"" _ms(long double ms)
{
  return std::chrono::duration<long double, std::milli>(ms);
}

// NOLINTNEXTLINE(runtime/int)
constexpr std::chrono::nanoseconds operator"" _ns(unsigned long long ns)
{
  return std::chrono::nanoseconds(ns);
}
constexpr std::chrono::duration<long double, std::nano> operator"" _ns(long double ns)
{
  return std::chrono::duration<long double, std::nano>(ns);
}

}  // namespace literals
}  // namespace rclcpp

#endif  // RCLCPP__LITERALS_HPP_
