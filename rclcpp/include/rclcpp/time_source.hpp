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
   * \param use_clock_thread whether to spin the attached node in a separate thread
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
   * \param use_clock_thread whether to spin the attached node in a separate thread.
   */
  RCLCPP_PUBLIC
  explicit TimeSource(
    const rclcpp::QoS & qos = rclcpp::ClockQoS(),
    bool use_clock_thread = true);

  // The TimeSource is uncopyable
  TimeSource(const TimeSource &) = delete;
  TimeSource & operator=(const TimeSource &) = delete;

  // The TimeSource is moveable
  TimeSource(TimeSource &&) = default;
  TimeSource & operator=(TimeSource &&) = default;

  /// Attach node to the time source.
  /**
   * \param node std::shared pointer to a initialized node
   */
  RCLCPP_PUBLIC
  void attachNode(rclcpp::Node::SharedPtr node);

  /// Attach node to the time source.
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

  /// Detach a clock from the time source
  RCLCPP_PUBLIC
  void detachClock(rclcpp::Clock::SharedPtr clock);

  /// Get whether a separate clock thread is used or not
  RCLCPP_PUBLIC
  bool get_use_clock_thread();

  /// Set whether to use a separate clock thread or not
  RCLCPP_PUBLIC
  void set_use_clock_thread(bool use_clock_thread);

  /// Check if the clock thread is joinable
  RCLCPP_PUBLIC
  bool clock_thread_is_joinable();

  /// TimeSource Destructor
  RCLCPP_PUBLIC
  ~TimeSource();

private:
  class NodeState;
  std::shared_ptr<NodeState> node_state_;

  // Preserve the arguments received by the constructor for reuse at runtime
  bool constructed_use_clock_thread_;
  rclcpp::QoS constructed_qos_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_
