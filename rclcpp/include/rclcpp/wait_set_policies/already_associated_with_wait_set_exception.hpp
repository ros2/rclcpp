// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_POLICIES__ALREADY_ASSOCIATED_WITH_WAIT_SET_EXCEPTION_HPP_
#define RCLCPP__WAIT_SET_POLICIES__ALREADY_ASSOCIATED_WITH_WAIT_SET_EXCEPTION_HPP_

#include <stdexcept>

#include "rmw/impl/cpp/demangle.hpp"

namespace rclcpp
{
namespace wait_set_policies
{

class AlreadyAssociatedWithWaitSetException : public std::runtime_error
{
public:
  template<typename EntityT>
  explicit
  AlreadyAssociatedWithWaitSetException(const EntityT & entity_instance)
  : std::runtime_error(
    "cannot associate " +
    rmw::impl::cpp::demangle(entity_instance) +
    " with wait set because it is already in use by a wait set")
  {}
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__ALREADY_ASSOCIATED_WITH_WAIT_SET_EXCEPTION_HPP_
