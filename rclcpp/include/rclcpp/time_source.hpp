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

#include "rcl_interfaces/msg/time.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_events_filter.hpp"


namespace rclcpp
{
class Clock;

class TimeSource
{
public:
  RCLCPP_PUBLIC
  explicit TimeSource(rclcpp::Node::SharedPtr node);

  RCLCPP_PUBLIC
  TimeSource();

  RCLCPP_PUBLIC
  void attachNode(rclcpp::Node::SharedPtr node);

  RCLCPP_PUBLIC
  void attachNode(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface);

  RCLCPP_PUBLIC
  void detachNode();

  /// Attach a clock to the time source to be updated
  /**
   * \throws std::invalid_argument if node is nullptr
   */
  RCLCPP_PUBLIC
  void attachClock(rclcpp::Clock::SharedPtr clock);

  RCLCPP_PUBLIC
  void detachClock(rclcpp::Clock::SharedPtr clock);

  RCLCPP_PUBLIC
  ~TimeSource();

private:
  // Preserve the node reference
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;

  // The subscription for the clock callback
  using MessageT = rcl_interfaces::msg::Time;
  using Alloc = std::allocator<void>;
  using SubscriptionT = rclcpp::Subscription<MessageT, Alloc>;
  std::shared_ptr<SubscriptionT> clock_subscription_;

  // The clock callback itself
  void clock_cb(const rcl_interfaces::msg::Time::SharedPtr msg);

  // Parameter Client pointer
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;

  // Parameter Event subscription
  using ParamMessageT = rcl_interfaces::msg::ParameterEvent;
  using ParamSubscriptionT = rclcpp::Subscription<ParamMessageT, Alloc>;
  std::shared_ptr<ParamSubscriptionT> parameter_subscription_;

  // Callback for parameter updates
  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  // An enum to hold the parameter state
  enum UseSimTimeParameterState {UNSET, SET_TRUE, SET_FALSE};
  UseSimTimeParameterState parameter_state_;

  // An internal method to use in the clock callback that iterates and enables all clocks
  void enable_ros_time();
  // An internal method to use in the clock callback that iterates and disables all clocks
  void disable_ros_time();

  // Internal helper functions used inside iterators
  static void enable_ros_time(rclcpp::Clock::SharedPtr clock);
  static void disable_ros_time(rclcpp::Clock::SharedPtr clock);
  static void set_clock(
    const rcl_interfaces::msg::Time::SharedPtr msg,
    bool set_ros_time_enabled,
    rclcpp::Clock::SharedPtr clock);

  // Local storage of validity of ROS time
  // This is needed when new clocks are added.
  bool ros_time_active_;
  // Last set message to be passed to newly registered clocks
  rcl_interfaces::msg::Time::SharedPtr last_msg_set_;

  // A lock to protect iterating the associated_clocks_ field.
  std::mutex clock_list_lock_;
  // A vector to store references to associated clocks.
  std::vector<rclcpp::Clock::SharedPtr> associated_clocks_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_
