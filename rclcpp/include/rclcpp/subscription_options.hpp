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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/detail/rmw_implementation_specific_subscription_payload.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/topic_statistics_state.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Non-template base class for subscription options.
struct SubscriptionOptionsBase
{
  /// Callbacks for events related to this subscription.
  SubscriptionEventCallbacks event_callbacks;

  /// Whether or not to use default callbacks when user doesn't supply any in event_callbacks
  bool use_default_callbacks = true;

  /// True to ignore local publications.
  bool ignore_local_publications = false;

  /// The callback group for this subscription. NULL to use the default callback group.
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;

  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;

  /// Setting the data-type stored in the intraprocess buffer
  IntraProcessBufferType intra_process_buffer_type = IntraProcessBufferType::CallbackDefault;

  /// Optional RMW implementation specific payload to be used during creation of the subscription.
  std::shared_ptr<rclcpp::detail::RMWImplementationSpecificSubscriptionPayload>
  rmw_implementation_payload = nullptr;

  // Options to configure topic statistics collector in the subscription.
  struct TopicStatisticsOptions
  {
    // Enable and disable topic statistics calculation and publication. Defaults to disabled.
    TopicStatisticsState state = TopicStatisticsState::NodeDefault;

    // Topic to which topic statistics get published when enabled. Defaults to /statistics.
    std::string publish_topic = "/statistics";

    // Topic statistics publication period in ms. Defaults to one second.
    // Only values greater than zero are allowed.
    std::chrono::milliseconds publish_period{std::chrono::seconds(1)};
  };

  TopicStatisticsOptions topic_stats_options;
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
  /**
   * \param qos QoS profile for subcription.
   * \return rcl_subscription_options_t structure based on the rclcpp::QoS
   */
  template<typename MessageT>
  rcl_subscription_options_t
  to_rcl_subscription_options(const rclcpp::QoS & qos) const
  {
    rcl_subscription_options_t result = rcl_subscription_get_default_options();
    using AllocatorTraits = std::allocator_traits<Allocator>;
    using MessageAllocatorT = typename AllocatorTraits::template rebind_alloc<MessageT>;
    auto message_alloc = std::make_shared<MessageAllocatorT>(*allocator.get());
    result.allocator = allocator::get_rcl_allocator<MessageT>(*message_alloc);
    result.qos = qos.get_rmw_qos_profile();
    result.rmw_subscription_options.ignore_local_publications = this->ignore_local_publications;

    // Apply payload to rcl_subscription_options if necessary.
    if (rmw_implementation_payload && rmw_implementation_payload->has_been_customized()) {
      rmw_implementation_payload->modify_rmw_subscription_options(result.rmw_subscription_options);
    }

    return result;
  }

  /// Get the allocator, creating one if needed.
  std::shared_ptr<Allocator>
  get_allocator() const
  {
    if (!this->allocator) {
      return std::make_shared<Allocator>();
    }
    return this->allocator;
  }
};

using SubscriptionOptions = SubscriptionOptionsWithAllocator<std::allocator<void>>;
}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
