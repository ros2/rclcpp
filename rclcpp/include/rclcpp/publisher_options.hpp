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

#ifndef RCLCPP__PUBLISHER_OPTIONS_HPP_
#define RCLCPP__PUBLISHER_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Structure containing optional configuration for Publishers.
template<typename Alloc = std::allocator<void>>
struct PublisherOptions
{
  /// The quality of service profile to pass on to the rmw implementation.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  /// Optional custom allocator.
  std::shared_ptr<Alloc> allocator = nullptr;
  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_
