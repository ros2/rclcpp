// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUBSCRIPTION_STATISTICS_MONITOR_HPP_
#define RCLCPP__SUBSCRIPTION_STATISTICS_MONITOR_HPP_

#include <memory>

#include "rmw/types.h"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/**
 * Pure virtual interface that allows external code to monitor message reception
 * for a single subscription.
 *
 * Implementations are responsible for any collection, aggregation, and
 * publication of statistics. rclcpp calls before_message_dispatch() before
 * the user callback and after_message_dispatch() after the user callback for
 * each received message.
 */
class SubscriptionStatisticsMonitor
{
public:
  using SharedPtr = std::shared_ptr<SubscriptionStatisticsMonitor>;
  using WeakPtr = std::weak_ptr<SubscriptionStatisticsMonitor>;

  RCLCPP_PUBLIC
  virtual ~SubscriptionStatisticsMonitor() = default;

  /// Called once before the user callback is dispatched for a received message.
  /**
   * \param[in] message_info RMW message information for the received message.
   */
  RCLCPP_PUBLIC
  virtual void before_message_dispatch(const rmw_message_info_t & message_info) = 0;

  /// Called once after the user callback has been dispatched for a received message.
  /**
   * \param[in] message_info RMW message information for the received message.
   */
  RCLCPP_PUBLIC
  virtual void after_message_dispatch(const rmw_message_info_t & message_info) = 0;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_STATISTICS_MONITOR_HPP_
