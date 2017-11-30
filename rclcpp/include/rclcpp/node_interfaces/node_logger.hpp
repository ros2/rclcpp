// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_LOGGER_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_LOGGER_HPP_

#include <memory>

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logger_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeLogger part of the Node API.
class NodeLogger : public NodeLoggerInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeLoggerInterface)

  RCLCPP_PUBLIC
  explicit NodeLogger(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeLogger();

  RCLCPP_PUBLIC
  virtual
  rclcpp::Logger
  get_logger() const;

private:
  RCLCPP_DISABLE_COPY(NodeLogger)

  /// Handle to the NodeBaseInterface given in the constructor.
  rclcpp::node_interfaces::NodeBaseInterface * node_base_;

  rclcpp::Logger logger_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_LOGGER_HPP_
