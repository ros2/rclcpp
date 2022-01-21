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

#ifndef RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_
#define RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_

#include "rosidl_runtime_cpp/traits.hpp"

#include "rclcpp/type_adapter.hpp"

namespace rclcpp
{

template<typename T>
struct is_ros_compatible_type
{
  static constexpr bool value =
    rosidl_generator_traits::is_message<T>::value ||
    rclcpp::TypeAdapter<T>::is_specialized::value;
};

}  // namespace rclcpp

#endif  // RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_
