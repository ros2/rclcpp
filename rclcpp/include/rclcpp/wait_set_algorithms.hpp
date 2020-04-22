// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_ALGORITHMS_HPP_
#define RCLCPP__WAIT_SET_ALGORITHMS_HPP_

#include <algorithm>

#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"

namespace rclcpp
{

/// Find next ready entity out of the given sequence from start to last.
template<class InputIt>
InputIt
find_next_ready_wait_set_entity(InputIt start, InputIt last)
{
  return std::find_if(start, last, [](const auto & entity) {return nullptr != entity;});
}

/// Find next ready guard condition given a WaitResult.
template<class WaitSetT>
typename WaitSetT::StoragePolicy::GuardConditionsIterable::const_iterator
find_next_ready_guard_condition(
  const typename rclcpp::WaitResult<WaitSetT> & wait_result,
  typename WaitSetT::StoragePolicy::GuardConditionsIterable::const_iterator start = (
    wait_result.get_wait_set()))
{
  return
}


}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_ALGORITHMS_HPP_
