// Copyright 2023 eSOL Co.,Ltd.
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

#ifndef RCLCPP__THREADS__POSIX__UTILITIES_HPP_
#define RCLCPP__THREADS__POSIX__UTILITIES_HPP_

#include <system_error>

namespace rclcpp
{

namespace detail
{

namespace
{

inline void throw_if_error(int r, char const * msg)
{
  if (r != 0) {
    throw std::system_error(r, std::system_category(), msg);
  }
}

}  // namespace

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__UTILITIES_HPP_
