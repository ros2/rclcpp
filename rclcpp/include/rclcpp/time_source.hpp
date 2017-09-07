// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIME_SOURCE_HPP_
#define RCLCPP__TIME_SOURCE_HPP_

#include <memory>
#include <vector>

#include "rcl/time.h"

#include "rclcpp/node.hpp"


namespace rclcpp
{
class Clock;

class TimeSource
{
public:
  RCLCPP_PUBLIC
  explicit TimeSource(std::shared_ptr<rclcpp::node::Node> node);

  RCLCPP_PUBLIC
  TimeSource();

  RCLCPP_PUBLIC
  void attachNode(std::shared_ptr<rclcpp::node::Node> node);

  RCLCPP_PUBLIC
  void detachNode();

  RCLCPP_PUBLIC
  void attachClock(std::shared_ptr<rclcpp::Clock> clock);

  RCLCPP_PUBLIC
  void detachClock(std::shared_ptr<rclcpp::Clock> clock);

  RCLCPP_PUBLIC
  ~TimeSource();

  // TODO(tfoote) add register callback for time jumps

private:
  // Preserve the node reference
  std::shared_ptr<rclcpp::node::Node> node_;

  // The subscription for the clock callback
  using MessageT = builtin_interfaces::msg::Time;
  using Alloc = std::allocator<void>;
  using SubscriptionT = rclcpp::subscription::Subscription<MessageT, Alloc>;
  std::shared_ptr<SubscriptionT> clock_subscription_;

  // The clock callback itself
  void clock_cb(const builtin_interfaces::msg::Time::SharedPtr msg);


  // An internal method to use in the clock callback that iterates and enables all clocks
  void enableROSTime();
  // An internal method to use in the clock callback that iterates and disables all clocks
  void disableROSTime();

  // Internal helper functions used inside iterators
  static void enableROSTime(std::shared_ptr<rclcpp::Clock> clock);
  static void disableROSTime(std::shared_ptr<rclcpp::Clock> clock);
  static void setClock(const builtin_interfaces::msg::Time::SharedPtr msg,
    std::shared_ptr<rclcpp::Clock> clock);

  // Local storage of validity of ROS time
  // This is needed when new clocks are added.
  bool ros_time_valid_;

  // A lock to protect iterating the associated_clocks_ field.
  std::mutex clock_list_lock_;
  // A vector to store references to associated clocks.
  std::vector<std::shared_ptr<rclcpp::Clock>> associated_clocks_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_
