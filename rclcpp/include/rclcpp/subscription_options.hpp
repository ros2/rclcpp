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

#include "rclcpp/callback_group.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Non-template base class for subscription options.
struct SubscriptionOptionsBase
{
  /// Callbacks for events related to this subscription.
  SubscriptionEventCallbacks event_callbacks;
  /// True to ignore local publications.
  bool ignore_local_publications = false;
  /// The callback group for this subscription. NULL to use the default callback group.
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group = nullptr;
  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;
};

/// Structure containing optional configuration for Subscriptions.
template<typename Allocator>
struct SubscriptionOptionsWithAllocator : public SubscriptionOptionsBase
{
  /// Optional custom allocator.
  std::shared_ptr<Allocator> allocator = nullptr;

  SubscriptionOptionsWithAllocator<Allocator>() {}

  /// Constructor using base class as input.
  explicit SubscriptionOptionsWithAllocator(
    const SubscriptionOptionsBase & subscription_options_base)
  : SubscriptionOptionsBase(subscription_options_base)
  {}

  /// Convert this class, with a rclcpp::QoS, into an rcl_subscription_options_t.
  template<typename MessageT>
  rcl_subscription_options_t
  to_rcl_subscription_options(const rclcpp::QoS & qos) const
  {
    rcl_subscription_options_t result;
    using AllocatorTraits = std::allocator_traits<Allocator>;
    using MessageAllocatorT = typename AllocatorTraits::template rebind_alloc<MessageT>;
    auto message_alloc = std::make_shared<MessageAllocatorT>(*allocator.get());
    result.allocator = allocator::get_rcl_allocator<MessageT>(*message_alloc);
    result.ignore_local_publications = this->ignore_local_publications;
    result.qos = qos.get_rmw_qos_profile();
    return result;
  }
};

using SubscriptionOptions = SubscriptionOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
