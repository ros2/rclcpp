// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// Based on: http://the-witness.net/news/2012/11/scopeexit-in-c11/
// But I changed the lambda to include by reference rather than value, see:
// http://the-witness.net/news/2012/11/scopeexit-in-c11/comment-page-1/#comment-86873

#ifndef RCLCPP__SCOPE_EXIT_HPP_
#define RCLCPP__SCOPE_EXIT_HPP_

#include <functional>

#include "rclcpp/macros.hpp"

namespace rclcpp
{

template<typename Callable>
struct ScopeExit
{
  explicit ScopeExit(Callable callable)
  : callable_(callable) {}
  ~ScopeExit() {callable_();}

private:
  Callable callable_;
};

template<typename Callable>
ScopeExit<Callable>
make_scope_exit(Callable callable)
{
  return ScopeExit<Callable>(callable);
}

}  // namespace rclcpp

#define RCLCPP_SCOPE_EXIT(code) \
  auto RCLCPP_STRING_JOIN(scope_exit_, __LINE__) = rclcpp::make_scope_exit([&]() {code;})

#endif  // RCLCPP__SCOPE_EXIT_HPP_
