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

/*
   Testing node handle construction.
 */
TEST_F(TestNodeInterfaces, nh_init) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto empty_nh_from_standalone = rclcpp::node_interfaces::get_node_interfaces<
      rclcpp::node_interfaces::Base,
      rclcpp::node_interfaces::Clock,
      rclcpp::node_interfaces::Graph,
      rclcpp::node_interfaces::Logging,
      rclcpp::node_interfaces::Timers,
      rclcpp::node_interfaces::Topics,
      rclcpp::node_interfaces::Services,
      rclcpp::node_interfaces::Waitables,
      rclcpp::node_interfaces::Parameters,
      rclcpp::node_interfaces::TimeSource
      >();
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

    auto base_nh = std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::Base
        >>(node);
    EXPECT_NE(nullptr, base_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_nh->get_node_time_source_interface());

    auto base_nh_from_standalone = rclcpp::node_interfaces::get_node_interfaces<
      rclcpp::node_interfaces::Base
      >(node);
    EXPECT_NE(nullptr, base_nh_from_standalone->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh_from_standalone->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_nh_from_standalone->get_node_time_source_interface());

    auto base_clock_nh =
      std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::Base,
          rclcpp::node_interfaces::Clock
        >>(node);
    EXPECT_NE(nullptr, base_clock_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_clock_nh->get_node_base_interface()->get_name());
    EXPECT_NE(nullptr, base_clock_nh->get_node_clock_interface());
    EXPECT_TRUE(
      RCL_ROS_TIME == base_clock_nh->get_node_clock_interface()->get_clock()->get_clock_type());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_clock_nh->get_node_time_source_interface());

    auto sans_base_clock_nh = std::make_shared<
      rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::Graph,
        rclcpp::node_interfaces::Logging,
        rclcpp::node_interfaces::Timers,
        rclcpp::node_interfaces::Topics,
        rclcpp::node_interfaces::Services,
        rclcpp::node_interfaces::Waitables,
        rclcpp::node_interfaces::Parameters,
        rclcpp::node_interfaces::TimeSource
      >>(node);
    // The following will not be defined
    // EXPECT_EQ(nullptr, sans_base_clock_nh->get_node_base_interface());
    // EXPECT_EQ(nullptr, sans_base_clock_nh->get_node_clock_interface());
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
TEST_F(TestNodeInterfaces, nh_setters) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

    auto base_nh = std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::Base,
          rclcpp::node_interfaces::Clock
        >>(node);

    EXPECT_NE(nullptr, base_nh->get_node_base_interface());
    EXPECT_STREQ("my_node", base_nh->get_node_base_interface()->get_name());

    base_nh->set_node_clock_interface(nullptr);
    EXPECT_EQ(nullptr, base_nh->get_node_clock_interface());

    base_nh->set_node_clock_interface(node->get_node_clock_interface());
    EXPECT_NE(nullptr, base_nh->get_node_clock_interface());
    EXPECT_TRUE(RCL_ROS_TIME == base_nh->get_node_clock_interface()->get_clock()->get_clock_type());
  }
}
