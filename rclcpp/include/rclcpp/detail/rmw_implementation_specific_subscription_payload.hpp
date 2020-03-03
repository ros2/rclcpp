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

#ifndef RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_SUBSCRIPTION_PAYLOAD_HPP_
#define RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_SUBSCRIPTION_PAYLOAD_HPP_

#include "rcl/subscription.h"

#include "rclcpp/detail/rmw_implementation_specific_payload.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{

/// Subscription payload that may be rmw implementation specific.
class RCLCPP_PUBLIC RMWImplementationSpecificSubscriptionPayload
  : public RMWImplementationSpecificPayload
{
public:
  ~RMWImplementationSpecificSubscriptionPayload() override = default;

  /// Opportunity for a derived class to inject information into the rcl options.
  /**
   * This is called after the rcl_subscription_options_t has been prepared by
   * rclcpp, but before rcl_subscription_init() is called.
   *
   * The passed option is the rmw_subscription_options field of the
   * rcl_subscription_options_t that will be passed to rcl_subscription_init().
   *
   * By default the options are unmodified.
   */
  virtual
  void
  modify_rmw_subscription_options(rmw_subscription_options_t & rmw_subscription_options) const;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_SUBSCRIPTION_PAYLOAD_HPP_
