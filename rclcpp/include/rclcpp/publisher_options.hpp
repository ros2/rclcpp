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

#include "rclcpp/callback_group.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl/publisher.h"

namespace rclcpp
{

/// Non-templated part of PublisherOptionsWithAllocator<Allocator>.
struct PublisherOptionsBase
{
  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;

  /// Callbacks for various events related to publishers.
  PublisherEventCallbacks event_callbacks;

  /// Callback group in which the waitable items from the publisher should be placed.
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
};

/// Structure containing optional configuration for Publishers.
template<typename Allocator>
struct PublisherOptionsWithAllocator : public PublisherOptionsBase
{
  /// Optional custom allocator.
  std::shared_ptr<Allocator> allocator = nullptr;

  PublisherOptionsWithAllocator<Allocator>() {}

  /// Constructor using base class as input.
  explicit PublisherOptionsWithAllocator(const PublisherOptionsBase & publisher_options_base)
  : PublisherOptionsBase(publisher_options_base)
  {}

  /// Convert this class, and a rclcpp::QoS, into an rcl_publisher_options_t.
  template<typename MessageT>
  rcl_publisher_options_t
  to_rcl_publisher_options(const rclcpp::QoS & qos) const
  {
    rcl_publisher_options_t result;
    using AllocatorTraits = std::allocator_traits<Allocator>;
    using MessageAllocatorT = typename AllocatorTraits::template rebind_alloc<MessageT>;
    auto message_alloc = std::make_shared<MessageAllocatorT>(*allocator.get());
    result.allocator = allocator::get_rcl_allocator<MessageT>(*message_alloc);
    result.qos = qos.get_rmw_qos_profile();
    return result;
  }
};

using PublisherOptions = PublisherOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_
