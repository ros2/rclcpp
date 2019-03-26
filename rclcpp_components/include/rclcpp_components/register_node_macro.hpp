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

#ifndef RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__
#define RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__

#include "class_loader/class_loader.hpp"
#include "rclcpp_components/node_factory_template.hpp"

#define RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass) \
  CLASS_LOADER_REGISTER_CLASS(rclcpp_components::NodeFactoryTemplate<NodeClass>, \
    rclcpp_components::NodeFactory)

#endif  // RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__
