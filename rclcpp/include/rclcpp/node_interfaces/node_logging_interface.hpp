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

#ifndef RCLCPP__NODE_INTERFACES__NODE_LOGGING_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_LOGGING_INTERFACE_HPP_

#include <memory>

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeLogging part of the Node API.
class NodeLoggingInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeLoggingInterface)

  RCLCPP_PUBLIC
  virtual
  ~NodeLoggingInterface() = default;

  /// Return the logger of the node.
  /** \return The logger of the node. */
  RCLCPP_PUBLIC
  virtual
  rclcpp::Logger
  get_logger() const = 0;

  /// Return the logger name associated with the node.
  /** \return The logger name associated with the node. */
  RCLCPP_PUBLIC
  virtual
  const char *
  get_logger_name() const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_LOGGING_INTERFACE_HPP_
