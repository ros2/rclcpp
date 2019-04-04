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

#include "rclcpp/rclcpp.hpp"

namespace test_rclcpp_components
{
/// Simple test component
class TestComponentFoo : public rclcpp::Node
{
public:
  explicit TestComponentFoo(rclcpp::NodeOptions options)
  : rclcpp::Node("test_component_foo", options)
  {
  }
};

/// Simple test component
class TestComponentBar : public rclcpp::Node
{
public:
  explicit TestComponentBar(rclcpp::NodeOptions options)
  : rclcpp::Node("test_component_bar", options)
  {
  }
};

/// Simple test component that doesn't inherit from rclcpp::Node
class TestComponentNoNode
{
public:
  explicit TestComponentNoNode(rclcpp::NodeOptions options)
  : node_("test_component_no_node", options)
  {
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface()
  {
    return node_.get_node_base_interface();
  }

private:
  rclcpp::Node node_;
};


}  // namespace test_rclcpp_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(test_rclcpp_components::TestComponentFoo)
RCLCPP_COMPONENTS_REGISTER_NODE(test_rclcpp_components::TestComponentBar)
RCLCPP_COMPONENTS_REGISTER_NODE(test_rclcpp_components::TestComponentNoNode)
