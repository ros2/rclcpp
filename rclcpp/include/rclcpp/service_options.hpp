// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SERVICE_OPTIONS_HPP_
#define RCLCPP__SERVICE_OPTIONS_HPP_

#include <memory>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "rcl/service.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/event_handler.hpp"

namespace rclcpp
{

class CallbackGroup;

struct ServiceOptionsBase
{
  /// Whether or not to use default callbacks when user doesn't supply any in event_callbacks
  bool use_default_callbacks = true;

  QosOverridingOptions qos_overriding_options;

  /// Callback group in which the waitable items from the service should be placed
  std::shared_ptr<rclcpp::CallbackGroup> callback_group;
};


template<typename Allocator>
struct ServiceOptionsWithAllocator : public ServiceOptionsBase
{
  static_assert(
    std::is_void_v<typename std::allocator_traits<Allocator>::value_type>,
    "Service allocator value type must be void");

  /// Optional custom allocator
  std::shared_ptr<Allocator> allocator = nullptr;

  ServiceOptionsWithAllocator() {}

  /// Constructor using base class as an input
  explicit ServiceOptionsWithAllocator(const ServiceOptionsBase & service_options_base)
  : ServiceOptionsBase(service_options_base)
  {}

  /// Convert this class, and a rclcpp::QoS to the rcl_service_options_t for low overhead
  rcl_service_options_t
  to_rcl_service_options(const rclcpp::QoS & qos) const
  {
    /// Destructure our rcl_service_options_t to set it up with the users values
    rcl_service_options_t result = rcl_service_get_default_options();
    result.qos = qos.get_rmw_qos_profile();
    result.allocator = this->get_rcl_allocator();

    return result;
  }

  /// Get or create the allocator if needed
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

  // returning a copy of the allocator
  mutable std::shared_ptr<Allocator> allocator_storage_;

  // returning a copy of the plain allocator
  mutable std::shared_ptr<PlainAllocator> plain_allocator_storage_;
};

using ServiceOptions = ServiceOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_OPTIONS_HPP_
