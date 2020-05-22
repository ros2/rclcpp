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

#include "rcl/publisher.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/rmw_implementation_specific_publisher_payload.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"

namespace rclcpp
{

class CallbackGroup;

/// Non-templated part of PublisherOptionsWithAllocator<Allocator>.
struct PublisherOptionsBase
{
  /// Setting to explicitly set intraprocess communications.
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;

  /// Callbacks for various events related to publishers.
  PublisherEventCallbacks event_callbacks;

  /// Whether or not to use default callbacks when user doesn't supply any in event_callbacks
  bool use_default_callbacks = true;

  /// Callback group in which the waitable items from the publisher should be placed.
  std::shared_ptr<rclcpp::CallbackGroup> callback_group;

  /// Optional RMW implementation specific payload to be used during creation of the publisher.
  std::shared_ptr<rclcpp::detail::RMWImplementationSpecificPublisherPayload>
  rmw_implementation_payload = nullptr;
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
    rcl_publisher_options_t result = rcl_publisher_get_default_options();
    using AllocatorTraits = std::allocator_traits<Allocator>;
    using MessageAllocatorT = typename AllocatorTraits::template rebind_alloc<MessageT>;
    auto message_alloc = std::make_shared<MessageAllocatorT>(*this->get_allocator().get());
    result.allocator = rclcpp::allocator::get_rcl_allocator<MessageT>(*message_alloc);
    result.qos = qos.get_rmw_qos_profile();

    // Apply payload to rcl_publisher_options if necessary.
    if (rmw_implementation_payload && rmw_implementation_payload->has_been_customized()) {
      rmw_implementation_payload->modify_rmw_publisher_options(result.rmw_publisher_options);
    }

    return result;
  }


  /// Get the allocator, creating one if needed.
  std::shared_ptr<Allocator>
  get_allocator() const
  {
    if (!this->allocator) {
      // TODO(wjwwood): I would like to use the commented line instead, but
      //   cppcheck 1.89 fails with:
      //     Syntax Error: AST broken, binary operator '>' doesn't have two operands.
      // return std::make_shared<Allocator>();
      std::shared_ptr<Allocator> tmp(new Allocator());
      return tmp;
    }
    return this->allocator;
  }
};

using PublisherOptions = PublisherOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_
