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
#include "rclcpp/node_handle.hpp"

class TestNodeInterfaceHandle : public ::testing::Test
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

/*
   Testing node handle construction.
 */
TEST_F(TestNodeInterfaceHandle, nh_init) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto empty_nh = std::make_shared<rclcpp::NodeInterfaceHandle<>>();
    EXPECT_EQ(nullptr, empty_nh->get_node_base_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_clock_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_graph_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_logging_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_timers_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_topics_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_services_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_waitables_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_parameters_interface());
    EXPECT_EQ(nullptr, empty_nh->get_node_time_source_interface());

    auto empty_nh_from_standalone = rclcpp::get_node_handle();
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_base_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_clock_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_graph_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_logging_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_timers_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_topics_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_services_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_waitables_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_parameters_interface());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->get_node_time_source_interface());

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::node_interfaces::NodeBaseInterface
        >>(node);
    EXPECT_NE(nullptr, base_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh->get_node_base_interface()->get_name());
    EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_graph_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_logging_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_timers_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_topics_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_services_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_waitables_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_parameters_interface());
    EXPECT_EQ(nullptr, base_nh->get_node_time_source_interface());

    auto base_nh_from_standalone = rclcpp::get_node_handle<
      rclcpp::node_interfaces::NodeBaseInterface
      >(node);
    EXPECT_NE(nullptr, base_nh_from_standalone->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh_from_standalone->get_node_base_interface()->get_name());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_clock_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_graph_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_logging_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_timers_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_topics_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_services_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_waitables_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_parameters_interface());
    EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_time_source_interface());

    auto base_clock_nh =
      std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::node_interfaces::NodeBaseInterface,
          rclcpp::node_interfaces::NodeClockInterface
        >>(node);
    EXPECT_NE(nullptr, base_clock_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_clock_nh->get_node_base_interface()->get_name());
    EXPECT_NE(nullptr, base_clock_nh->get_node_clock_interface());
    EXPECT_TRUE(
      RCL_ROS_TIME == base_clock_nh->get_node_clock_interface()->get_clock()->get_clock_type());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_graph_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_logging_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_timers_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_topics_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_services_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_waitables_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_parameters_interface());
    EXPECT_EQ(nullptr, base_clock_nh->get_node_time_source_interface());

    auto sans_base_clock_nh = std::make_shared<
      rclcpp::NodeInterfaceHandle<
        rclcpp::node_interfaces::NodeGraphInterface,
        rclcpp::node_interfaces::NodeLoggingInterface,
        rclcpp::node_interfaces::NodeTimersInterface,
        rclcpp::node_interfaces::NodeTopicsInterface,
        rclcpp::node_interfaces::NodeServicesInterface,
        rclcpp::node_interfaces::NodeWaitablesInterface,
        rclcpp::node_interfaces::NodeParametersInterface,
        rclcpp::node_interfaces::NodeTimeSourceInterface
      >>(node);
    EXPECT_EQ(nullptr, sans_base_clock_nh->get_node_base_interface());
    EXPECT_EQ(nullptr, sans_base_clock_nh->get_node_clock_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_graph_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_logging_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_timers_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_topics_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_services_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_waitables_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_parameters_interface());
    EXPECT_NE(nullptr, sans_base_clock_nh->get_node_time_source_interface());

    auto empty_nh_from_node = std::make_shared<rclcpp::NodeInterfaceHandle<>>(node);
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_base_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_clock_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_graph_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_logging_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_timers_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_topics_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_services_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_waitables_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_parameters_interface());
    EXPECT_EQ(nullptr, empty_nh_from_node->get_node_time_source_interface());
  }
}

/*
   Testing node handle setters.
 */
TEST_F(TestNodeInterfaceHandle, nh_setters) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::node_interfaces::NodeBaseInterface
        >>(node);

    EXPECT_NE(nullptr, base_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh->get_node_base_interface()->get_name());

    EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());

    base_nh->set_node_clock_interface(node->get_node_clock_interface());
    EXPECT_NE(nullptr, base_nh->get_node_clock_interface());
    EXPECT_TRUE(RCL_ROS_TIME == base_nh->get_node_clock_interface()->get_clock()->get_clock_type());
  }
}
