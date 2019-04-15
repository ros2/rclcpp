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

#ifndef RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
#define RCLCPP__SUBSCRIPTION_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos_event.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Structure containing optional configuration for Subscriptions.
template<typename Allocator>
struct SubscriptionOptionsWithAllocator
{
  SubscriptionEventCallbacks event_callbacks;
  /// The quality of service profile to pass on to the rmw implementation.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  /// True to ignore local publications.
  bool ignore_local_publications = false;
  /// The callback group for this subscription. NULL to use the default callback group.
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
  /// Optional custom allocator.
  std::shared_ptr<Allocator> allocator;
  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;
};

using SubscriptionOptions = SubscriptionOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
