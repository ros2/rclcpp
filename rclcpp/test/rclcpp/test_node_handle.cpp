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
    EXPECT_EQ(nullptr, empty_nh->base());
    EXPECT_EQ(nullptr, empty_nh->clock());
    EXPECT_EQ(nullptr, empty_nh->graph());
    EXPECT_EQ(nullptr, empty_nh->logging());
    EXPECT_EQ(nullptr, empty_nh->timers());
    EXPECT_EQ(nullptr, empty_nh->topics());
    EXPECT_EQ(nullptr, empty_nh->services());
    EXPECT_EQ(nullptr, empty_nh->waitables());
    EXPECT_EQ(nullptr, empty_nh->parameters());
    EXPECT_EQ(nullptr, empty_nh->time_source());

    auto empty_nh_from_standalone = rclcpp::get_node_handle();
    EXPECT_EQ(nullptr, empty_nh_from_standalone->base());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->clock());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->graph());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->logging());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->timers());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->topics());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->services());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->waitables());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->parameters());
    EXPECT_EQ(nullptr, empty_nh_from_standalone->time_source());

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<rclcpp::BaseInterface>>(node);
    EXPECT_NE(nullptr, base_nh->base());
    EXPECT_STREQ("my_node", base_nh->base()->get_name());
    EXPECT_EQ(nullptr, base_nh->clock());
    EXPECT_EQ(nullptr, base_nh->graph());
    EXPECT_EQ(nullptr, base_nh->logging());
    EXPECT_EQ(nullptr, base_nh->timers());
    EXPECT_EQ(nullptr, base_nh->topics());
    EXPECT_EQ(nullptr, base_nh->services());
    EXPECT_EQ(nullptr, base_nh->waitables());
    EXPECT_EQ(nullptr, base_nh->parameters());
    EXPECT_EQ(nullptr, base_nh->time_source());

    auto base_nh_from_standalone = rclcpp::get_node_handle<rclcpp::BaseInterface>(node);
    EXPECT_NE(nullptr, base_nh_from_standalone->base());
    EXPECT_STREQ("my_node", base_nh_from_standalone->base()->get_name());
    EXPECT_EQ(nullptr, base_nh_from_standalone->clock());
    EXPECT_EQ(nullptr, base_nh_from_standalone->graph());
    EXPECT_EQ(nullptr, base_nh_from_standalone->logging());
    EXPECT_EQ(nullptr, base_nh_from_standalone->timers());
    EXPECT_EQ(nullptr, base_nh_from_standalone->topics());
    EXPECT_EQ(nullptr, base_nh_from_standalone->services());
    EXPECT_EQ(nullptr, base_nh_from_standalone->waitables());
    EXPECT_EQ(nullptr, base_nh_from_standalone->parameters());
    EXPECT_EQ(nullptr, base_nh_from_standalone->time_source());

    auto base_clock_nh =
      std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::BaseInterface,
          rclcpp::ClockInterface
        >>(node);
    EXPECT_NE(nullptr, base_clock_nh->base());
    EXPECT_STREQ("my_node", base_clock_nh->base()->get_name());
    EXPECT_NE(nullptr, base_clock_nh->clock());
    EXPECT_TRUE(RCL_ROS_TIME == base_clock_nh->clock()->get_clock()->get_clock_type());
    EXPECT_EQ(nullptr, base_clock_nh->graph());
    EXPECT_EQ(nullptr, base_clock_nh->logging());
    EXPECT_EQ(nullptr, base_clock_nh->timers());
    EXPECT_EQ(nullptr, base_clock_nh->topics());
    EXPECT_EQ(nullptr, base_clock_nh->services());
    EXPECT_EQ(nullptr, base_clock_nh->waitables());
    EXPECT_EQ(nullptr, base_clock_nh->parameters());
    EXPECT_EQ(nullptr, base_clock_nh->time_source());

    auto sans_base_clock_nh = std::make_shared<
      rclcpp::NodeInterfaceHandle<
        rclcpp::GraphInterface,
        rclcpp::LoggingInterface,
        rclcpp::TimersInterface,
        rclcpp::TopicsInterface,
        rclcpp::ServicesInterface,
        rclcpp::WaitablesInterface,
        rclcpp::ParametersInterface,
        rclcpp::TimeSourceInterface
      >>(node);
    EXPECT_EQ(nullptr, sans_base_clock_nh->base());
    EXPECT_EQ(nullptr, sans_base_clock_nh->clock());
    EXPECT_NE(nullptr, sans_base_clock_nh->graph());
    EXPECT_NE(nullptr, sans_base_clock_nh->logging());
    EXPECT_NE(nullptr, sans_base_clock_nh->timers());
    EXPECT_NE(nullptr, sans_base_clock_nh->topics());
    EXPECT_NE(nullptr, sans_base_clock_nh->services());
    EXPECT_NE(nullptr, sans_base_clock_nh->waitables());
    EXPECT_NE(nullptr, sans_base_clock_nh->parameters());
    EXPECT_NE(nullptr, sans_base_clock_nh->time_source());

    auto all_nh = std::make_shared<rclcpp::NodeInterfaceHandle<rclcpp::AllInterfaces>>(node);
    EXPECT_NE(nullptr, all_nh->base());
    EXPECT_NE(nullptr, all_nh->clock());
    EXPECT_NE(nullptr, all_nh->graph());
    EXPECT_NE(nullptr, all_nh->logging());
    EXPECT_NE(nullptr, all_nh->timers());
    EXPECT_NE(nullptr, all_nh->topics());
    EXPECT_NE(nullptr, all_nh->services());
    EXPECT_NE(nullptr, all_nh->waitables());
    EXPECT_NE(nullptr, all_nh->parameters());
    EXPECT_NE(nullptr, all_nh->time_source());

    auto empty_nh_from_node = std::make_shared<rclcpp::NodeInterfaceHandle<>>(node);
    EXPECT_EQ(nullptr, empty_nh_from_node->base());
    EXPECT_EQ(nullptr, empty_nh_from_node->clock());
    EXPECT_EQ(nullptr, empty_nh_from_node->graph());
    EXPECT_EQ(nullptr, empty_nh_from_node->logging());
    EXPECT_EQ(nullptr, empty_nh_from_node->timers());
    EXPECT_EQ(nullptr, empty_nh_from_node->topics());
    EXPECT_EQ(nullptr, empty_nh_from_node->services());
    EXPECT_EQ(nullptr, empty_nh_from_node->waitables());
    EXPECT_EQ(nullptr, empty_nh_from_node->parameters());
    EXPECT_EQ(nullptr, empty_nh_from_node->time_source());
  }
}

/*
   Testing node handle construction with alternate getters.
 */
TEST_F(TestNodeInterfaceHandle, nh_init_alternate_getters) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<rclcpp::BaseInterface>>(node);

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

    auto base_clock_nh = std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::BaseInterface,
          rclcpp::ClockInterface
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

    auto sans_base_clock_nh = std::make_shared<rclcpp::NodeInterfaceHandle<
          rclcpp::GraphInterface,
          rclcpp::LoggingInterface,
          rclcpp::TimersInterface,
          rclcpp::TopicsInterface,
          rclcpp::ServicesInterface,
          rclcpp::WaitablesInterface,
          rclcpp::ParametersInterface,
          rclcpp::TimeSourceInterface
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
  }
}

/*
   Testing node handle setters.
 */
TEST_F(TestNodeInterfaceHandle, nh_setters) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<rclcpp::BaseInterface>>(node);

    EXPECT_NE(nullptr, base_nh->base());
    EXPECT_STREQ("my_node", base_nh->base()->get_name());

    EXPECT_EQ(nullptr, base_nh->clock());
    EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());

    base_nh->clock(node->get_node_clock_interface());
    EXPECT_NE(nullptr, base_nh->clock());
    EXPECT_NE(nullptr, base_nh->get_node_clock_interface());
    EXPECT_TRUE(RCL_ROS_TIME == base_nh->clock()->get_clock()->get_clock_type());
  }
}

/*
   Testing node handle alternate setters.
 */
TEST_F(TestNodeInterfaceHandle, nh_alternate_setters) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto base_nh = std::make_shared<rclcpp::NodeInterfaceHandle<rclcpp::BaseInterface>>(node);

    EXPECT_NE(nullptr, base_nh->base());
    EXPECT_STREQ("my_node", base_nh->base()->get_name());

    EXPECT_EQ(nullptr, base_nh->clock());
    EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());

    base_nh->set_node_clock_interface(node->get_node_clock_interface());
    EXPECT_NE(nullptr, base_nh->clock());
    EXPECT_NE(nullptr, base_nh->get_node_clock_interface());
    EXPECT_TRUE(RCL_ROS_TIME == base_nh->clock()->get_clock()->get_clock_type());
  }
}
