// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

class TestNodeInterfaces : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNodeInterfaces, default_constructor) {
  auto node = std::make_shared<rclcpp::Node>("my_node");
  using rclcpp::node_interfaces::NodeInterfaces;
  using rclcpp::node_interfaces::NodeBaseInterface;
  using rclcpp::node_interfaces::NodeGraphInterface;
  NodeInterfaces<NodeBaseInterface, NodeGraphInterface> interfaces;
  interfaces = NodeInterfaces<NodeBaseInterface, NodeGraphInterface>(*node);
}

/*
   Testing NodeInterfaces construction from nodes.
 */
TEST_F(TestNodeInterfaces, node_interfaces_nominal) {
  auto node = std::make_shared<rclcpp::Node>("my_node");

  // Create a NodeInterfaces for base and graph using a rclcpp::Node.
  {
    using rclcpp::node_interfaces::NodeInterfaces;
    using rclcpp::node_interfaces::NodeBaseInterface;
    using rclcpp::node_interfaces::NodeGraphInterface;
    auto node_interfaces = NodeInterfaces<NodeBaseInterface, NodeGraphInterface>(*node);
  }

  // Implicit conversion of rclcpp::Node into function that uses NodeInterfaces of base.
  {
    using rclcpp::node_interfaces::NodeInterfaces;
    using rclcpp::node_interfaces::NodeBaseInterface;
    auto some_func = [](NodeInterfaces<NodeBaseInterface> ni) {
        auto base_interface = ni.get<NodeBaseInterface>();
      };

    some_func(*node);
  }

  // Implicit narrowing of NodeInterfaces into a new interface NodeInterfaces with fewer interfaces.
  {
    using rclcpp::node_interfaces::NodeInterfaces;
    using rclcpp::node_interfaces::NodeBaseInterface;
    using rclcpp::node_interfaces::NodeGraphInterface;
    auto some_func = [](NodeInterfaces<NodeBaseInterface> ni_with_one) {
        auto base_interface = ni_with_one.get<NodeBaseInterface>();
      };

    NodeInterfaces<NodeBaseInterface, NodeGraphInterface> ni_with_two(*node);

    some_func(ni_with_two);
  }

  // Create a NodeInterfaces via aggregation of interfaces in constructor.
  {
    using rclcpp::node_interfaces::NodeInterfaces;
    using rclcpp::node_interfaces::NodeBaseInterface;
    using rclcpp::node_interfaces::NodeGraphInterface;
    auto loose_node_base = node->get_node_base_interface();
    auto loose_node_graph = node->get_node_graph_interface();
    auto ni = NodeInterfaces<NodeBaseInterface, NodeGraphInterface>(
      loose_node_base,
      loose_node_graph);
  }
}

/*
   Test construction with all standard rclcpp::node_interfaces::Node*Interfaces.
 */
TEST_F(TestNodeInterfaces, node_interfaces_standard_interfaces) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  auto ni = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeGraphInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeTimersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeServicesInterface,
    rclcpp::node_interfaces::NodeWaitablesInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTimeSourceInterface
    >(*node);
}

/*
   Testing getters.
 */
TEST_F(TestNodeInterfaces, ni_init) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  using rclcpp::node_interfaces::NodeInterfaces;
  using rclcpp::node_interfaces::NodeBaseInterface;
  using rclcpp::node_interfaces::NodeClockInterface;
  using rclcpp::node_interfaces::NodeGraphInterface;
  using rclcpp::node_interfaces::NodeLoggingInterface;
  using rclcpp::node_interfaces::NodeTimersInterface;
  using rclcpp::node_interfaces::NodeTopicsInterface;
  using rclcpp::node_interfaces::NodeServicesInterface;
  using rclcpp::node_interfaces::NodeWaitablesInterface;
  using rclcpp::node_interfaces::NodeParametersInterface;
  using rclcpp::node_interfaces::NodeTimeSourceInterface;

  auto ni = NodeInterfaces<
    NodeBaseInterface,
    NodeClockInterface,
    NodeGraphInterface,
    NodeLoggingInterface,
    NodeTimersInterface,
    NodeTopicsInterface,
    NodeServicesInterface,
    NodeWaitablesInterface,
    NodeParametersInterface,
    NodeTimeSourceInterface
    >(*node);

  {
    auto base = ni.get<NodeBaseInterface>();
    base = ni.get_node_base_interface();
    EXPECT_STREQ(base->get_name(), "my_node");  // Test for functionality
  }
  {
    auto clock = ni.get<NodeClockInterface>();
    clock = ni.get_node_clock_interface();
    clock->get_clock();
  }
  {
    auto graph = ni.get<NodeGraphInterface>();
    graph = ni.get_node_graph_interface();
  }
  {
    auto logging = ni.get<NodeLoggingInterface>();
    logging = ni.get_node_logging_interface();
  }
  {
    auto timers = ni.get<NodeTimersInterface>();
    timers = ni.get_node_timers_interface();
  }
  {
    auto topics = ni.get<NodeTopicsInterface>();
    topics = ni.get_node_topics_interface();
  }
  {
    auto services = ni.get<NodeServicesInterface>();
    services = ni.get_node_services_interface();
  }
  {
    auto waitables = ni.get<NodeWaitablesInterface>();
    waitables = ni.get_node_waitables_interface();
  }
  {
    auto parameters = ni.get<NodeParametersInterface>();
    parameters = ni.get_node_parameters_interface();
  }
  {
    auto time_source = ni.get<NodeTimeSourceInterface>();
    time_source = ni.get_node_time_source_interface();
  }
}

/*
   Testing macro'ed getters.
 */
TEST_F(TestNodeInterfaces, ni_all_init) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  using rclcpp::node_interfaces::NodeInterfaces;
  using rclcpp::node_interfaces::NodeBaseInterface;
  using rclcpp::node_interfaces::NodeClockInterface;
  using rclcpp::node_interfaces::NodeGraphInterface;
  using rclcpp::node_interfaces::NodeLoggingInterface;
  using rclcpp::node_interfaces::NodeTimersInterface;
  using rclcpp::node_interfaces::NodeTopicsInterface;
  using rclcpp::node_interfaces::NodeServicesInterface;
  using rclcpp::node_interfaces::NodeWaitablesInterface;
  using rclcpp::node_interfaces::NodeParametersInterface;
  using rclcpp::node_interfaces::NodeTimeSourceInterface;

  auto ni = rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>(*node);

  {
    auto base = ni.get<NodeBaseInterface>();
    base = ni.get_node_base_interface();
    EXPECT_STREQ(base->get_name(), "my_node");  // Test for functionality
  }
  {
    auto clock = ni.get<NodeClockInterface>();
    clock = ni.get_node_clock_interface();
    clock->get_clock();
  }
  {
    auto graph = ni.get<NodeGraphInterface>();
    graph = ni.get_node_graph_interface();
  }
  {
    auto logging = ni.get<NodeLoggingInterface>();
    logging = ni.get_node_logging_interface();
  }
  {
    auto timers = ni.get<NodeTimersInterface>();
    timers = ni.get_node_timers_interface();
  }
  {
    auto topics = ni.get<NodeTopicsInterface>();
    topics = ni.get_node_topics_interface();
  }
  {
    auto services = ni.get<NodeServicesInterface>();
    services = ni.get_node_services_interface();
  }
  {
    auto waitables = ni.get<NodeWaitablesInterface>();
    waitables = ni.get_node_waitables_interface();
  }
  {
    auto parameters = ni.get<NodeParametersInterface>();
    parameters = ni.get_node_parameters_interface();
  }
  {
    auto time_source = ni.get<NodeTimeSourceInterface>();
    time_source = ni.get_node_time_source_interface();
  }
}
