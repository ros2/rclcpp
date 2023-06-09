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

#ifndef RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_
#define RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_

#include "rcl/publisher.h"

#include "rclcpp/detail/rmw_implementation_specific_payload.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{

class RCLCPP_PUBLIC RMWImplementationSpecificPublisherPayload
  : public RMWImplementationSpecificPayload
{
public:
  ~RMWImplementationSpecificPublisherPayload() override = default;

  /// Opportunity for a derived class to inject information into the rcl options.
  /**
   * This is called after the rcl_publisher_options_t has been prepared by
   * rclcpp, but before rcl_publisher_init() is called.
   *
   * The passed option is the rmw_publisher_options field of the
   * rcl_publisher_options_t that will be passed to rcl_publisher_init().
   *
   * By default the options are unmodified.
   */
  virtual
  void
  modify_rmw_publisher_options(rmw_publisher_options_t & rmw_publisher_options) const;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_
