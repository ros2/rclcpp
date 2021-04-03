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

#include "builtin_interfaces/msg/time.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"


namespace rclcpp
{
class Clock;

/**
 * Time source that will drive the attached clocks.
 *
 * If the attached node `use_sim_time` parameter is `true`, the attached clocks will
 * be updated based on messages received.
 *
 * The subscription to the clock topic created by the time source can have it's qos reconfigured
 * using parameter overrides, particularly the following ones are accepted:
 *
 * - qos_overrides./clock.depth
 * - qos_overrides./clock.durability
 * - qos_overrides./clock.history
 * - qos_overrides./clock.reliability
 */
class TimeSource
{
public:
  /// Constructor
  /**
   * The node will be attached to the time source.
   *
   * \param node std::shared pointer to a initialized node
   * \param qos QoS that will be used when creating a `/clock` subscription.
   */
  RCLCPP_PUBLIC
  explicit TimeSource(
    rclcpp::Node::SharedPtr node,
    const rclcpp::QoS & qos = rclcpp::ClockQoS(),
    bool use_clock_thread = true);

  /// Empty constructor
  /**
   * An Empty TimeSource class
   *
   * \param qos QoS that will be used when creating a `/clock` subscription.
   */
  RCLCPP_PUBLIC
  explicit TimeSource(
    const rclcpp::QoS & qos = rclcpp::ClockQoS(),
    bool use_clock_thread = true);

  /// Attack node to the time source.
  /**
   * \param node std::shared pointer to a initialized node
   */
  RCLCPP_PUBLIC
  void attachNode(rclcpp::Node::SharedPtr node);

  /// Attack node to the time source.
  /**
   * If the parameter `use_sim_time` is `true` then the source time is the simulation time,
   * otherwise the source time is defined by the system.
   *
   * \param node_base_interface Node base interface.
   * \param node_topics_interface Node topic base interface.
   * \param node_graph_interface Node graph interface.
   * \param node_services_interface Node service interface.
   * \param node_logging_interface Node logging interface.
   * \param node_clock_interface Node clock interface.
   * \param node_parameters_interface Node parameters interface.
   */
  RCLCPP_PUBLIC
  void attachNode(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface);

  /// Detach the node from the time source
  RCLCPP_PUBLIC
  void detachNode();

  /// Attach a clock to the time source to be updated
  /**
   * \param[in] clock to attach to the time source
   * \throws std::invalid_argument the time source must be a RCL_ROS_TIME otherwise throws an exception
   */
  RCLCPP_PUBLIC
  void attachClock(rclcpp::Clock::SharedPtr clock);

  /// Detach a clock to the time source
  RCLCPP_PUBLIC
  void detachClock(rclcpp::Clock::SharedPtr clock);

  /// TimeSource Destructor
  RCLCPP_PUBLIC
  ~TimeSource();

protected:
  // Dedicated thread for clock subscription.
  bool use_clock_thread_;
  std::thread clock_executor_thread_;

private:
  // Preserve the node reference
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  // Store (and update on node attach) logger for logging.
  Logger logger_;

  // QoS of the clock subscription.
  rclcpp::QoS qos_;

  // The subscription for the clock callback
  using MessageT = rosgraph_msgs::msg::Clock;
  using Alloc = std::allocator<void>;
  using SubscriptionT = rclcpp::Subscription<MessageT, Alloc>;
  std::shared_ptr<SubscriptionT> clock_subscription_{nullptr};
  std::mutex clock_sub_lock_;
  rclcpp::CallbackGroup::SharedPtr clock_callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr clock_executor_;
  std::promise<void> cancel_clock_executor_promise_;

  // The clock callback itself
  void clock_cb(std::shared_ptr<const rosgraph_msgs::msg::Clock> msg);

  // Create the subscription for the clock topic
  void create_clock_sub();

  // Destroy the subscription for the clock topic
  void destroy_clock_sub();

  // Parameter Event subscription
  using ParamMessageT = rcl_interfaces::msg::ParameterEvent;
  using ParamSubscriptionT = rclcpp::Subscription<ParamMessageT, Alloc>;
  std::shared_ptr<ParamSubscriptionT> parameter_subscription_;

  // Callback for parameter updates
  void on_parameter_event(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);

  // An enum to hold the parameter state
  enum UseSimTimeParameterState {UNSET, SET_TRUE, SET_FALSE};
  UseSimTimeParameterState parameter_state_;

  // An internal method to use in the clock callback that iterates and enables all clocks
  void enable_ros_time();
  // An internal method to use in the clock callback that iterates and disables all clocks
  void disable_ros_time();

  // Internal helper functions used inside iterators
  static void set_clock(
    const builtin_interfaces::msg::Time::SharedPtr msg,
    bool set_ros_time_enabled,
    rclcpp::Clock::SharedPtr clock);

  // Local storage of validity of ROS time
  // This is needed when new clocks are added.
  bool ros_time_active_{false};
  // Last set message to be passed to newly registered clocks
  std::shared_ptr<const rosgraph_msgs::msg::Clock> last_msg_set_;

  // A lock to protect iterating the associated_clocks_ field.
  std::mutex clock_list_lock_;
  // A vector to store references to associated clocks.
  std::vector<rclcpp::Clock::SharedPtr> associated_clocks_;
  // A handler for the use_sim_time parameter callback.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr sim_time_cb_handler_{nullptr};
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_
