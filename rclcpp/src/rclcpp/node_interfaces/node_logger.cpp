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

#include "rclcpp/node_interfaces/node_logger.hpp"

using rclcpp::node_interfaces::NodeLogger;

NodeLogger::NodeLogger(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{
  // TODO(dhood): use the namespace (slashes converted to dots)
  logger_ = rclcpp::get_logger(node_base_->get_name());
}

NodeLogger::~NodeLogger()
{
}

rclcpp::Logger
NodeLogger::get_logger() const
{
  return logger_;
}
