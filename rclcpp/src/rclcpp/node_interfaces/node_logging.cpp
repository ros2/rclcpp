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

#include "rclcpp/node_interfaces/node_logging.hpp"

using rclcpp::node_interfaces::NodeLogging;

NodeLogging::NodeLogging(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{
  logger_ = rclcpp::get_logger(this->get_logger_name());
}

NodeLogging::~NodeLogging()
{
}

rclcpp::Logger
NodeLogging::get_logger() const
{
  return logger_;
}

const char *
NodeLogging::get_logger_name() const
{
  return rcl_node_get_logger_name(node_base_->get_rcl_node_handle());
}
