// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef NODE_INTERFACES__NODE_WRAPPER_HPP_
#define NODE_INTERFACES__NODE_WRAPPER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class NodeWrapper
{
public:
  explicit NodeWrapper(const std::string & name)
  : node(std::make_shared<rclcpp::Node>(name))
  {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() {return this->node->get_node_base_interface();}

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface() {return this->node->get_node_clock_interface();}

  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface() {return this->node->get_node_graph_interface();}

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
  get_node_logging_interface() {return this->node->get_node_logging_interface();}

  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface() {return this->node->get_node_timers_interface();}

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface() {return this->node->get_node_topics_interface();}

  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface() {return this->node->get_node_services_interface();}

  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface() {return this->node->get_node_waitables_interface();}

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface() {return this->node->get_node_parameters_interface();}

  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
  get_node_time_source_interface() {return this->node->get_node_time_source_interface();}

private:
  rclcpp::Node::SharedPtr node;
};

#endif  // NODE_INTERFACES__NODE_WRAPPER_HPP_
