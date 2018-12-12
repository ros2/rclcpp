// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_time_source.hpp"

#include <memory>
#include <string>

using rclcpp::node_interfaces::NodeTimeSource;

NodeTimeSource::NodeTimeSource(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters)
: node_base_(node_base),
  node_topics_(node_topics),
  node_graph_(node_graph),
  node_services_(node_services),
  node_logging_(node_logging),
  node_clock_(node_clock),
  node_parameters_(node_parameters)
{
  time_source_.attachNode(
    node_base_,
    node_topics_,
    node_graph_,
    node_services_,
    node_logging_,
    node_clock_,
    node_parameters_);
  time_source_.attachClock(node_clock_->get_clock());
}

NodeTimeSource::~NodeTimeSource()
{}
