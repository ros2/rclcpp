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
#include <type_traits>
#include <vector>

#include "rcl/publisher.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/rmw_implementation_specific_publisher_payload.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/qos_overriding_options.hpp"

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

  /// Require middleware to generate unique network flow endpoints
  /// Disabled by default
  rmw_unique_network_flow_endpoints_requirement_t require_unique_network_flow_endpoints =
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED;

  /// Callback group in which the waitable items from the publisher should be placed.
  std::shared_ptr<rclcpp::CallbackGroup> callback_group;

  /// Optional RMW implementation specific payload to be used during creation of the publisher.
  std::shared_ptr<rclcpp::detail::RMWImplementationSpecificPublisherPayload>
  rmw_implementation_payload = nullptr;

  QosOverridingOptions qos_overriding_options;
};

/// Structure containing optional configuration for Publishers.
template<typename Allocator>
struct PublisherOptionsWithAllocator : public PublisherOptionsBase
{
  static_assert(
    std::is_void_v<typename std::allocator_traits<Allocator>::value_type>,
    "Publisher allocator value type must be void");

  /// Optional custom allocator.
  std::shared_ptr<Allocator> allocator = nullptr;

  PublisherOptionsWithAllocator() {}

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
    result.allocator = this->get_rcl_allocator();
    result.qos = qos.get_rmw_qos_profile();
    result.rmw_publisher_options.require_unique_network_flow_endpoints =
      this->require_unique_network_flow_endpoints;

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
      if (!allocator_storage_) {
        allocator_storage_ = std::make_shared<Allocator>();
      }
      return allocator_storage_;
    }
    return this->allocator;
  }

private:
  using PlainAllocator =
    typename std::allocator_traits<Allocator>::template rebind_alloc<char>;

  rcl_allocator_t
  get_rcl_allocator() const
  {
    if (!plain_allocator_storage_) {
      plain_allocator_storage_ =
        std::make_shared<PlainAllocator>(*this->get_allocator());
    }
    return rclcpp::allocator::get_rcl_allocator<char>(*plain_allocator_storage_);
  }

  // This is a temporal workaround, to make sure that get_allocator()
  // always returns a copy of the same allocator.
  mutable std::shared_ptr<Allocator> allocator_storage_;

  // This is a temporal workaround, to keep the plain allocator that backs
  // up the rcl allocator returned in rcl_publisher_options_t alive.
  mutable std::shared_ptr<PlainAllocator> plain_allocator_storage_;
};

using PublisherOptions = PublisherOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_
