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

  using MessageT = builtin_interfaces::msg::Time;
  using Alloc = std::allocator<void>;
  using SubscriptionT = rclcpp::subscription::Subscription<MessageT, Alloc>;
  // The subscription for the clock callback
  std::shared_ptr<SubscriptionT> clock_subscription_;

  // The clock callback itself
  void clock_cb(const builtin_interfaces::msg::Time::SharedPtr msg);
  void enableROSTime();
  void disableROSTime();

  // Internal helper functions used inside iterators
  void enableROSTime(std::shared_ptr<rclcpp::Clock> clock);
  void disableROSTime(std::shared_ptr<rclcpp::Clock> clock);
  void setClock(const builtin_interfaces::msg::Time::SharedPtr msg,
    std::shared_ptr<rclcpp::Clock> clock);

  bool ros_time_valid_;

  std::mutex clock_list_lock_;
  std::vector<std::shared_ptr<rclcpp::Clock>> associated_clocks_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_
