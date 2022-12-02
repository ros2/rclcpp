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
   Testing NodeInterfaces construction from nodes.
 */
TEST_F(TestNodeInterfaces, test_node_construction) {
  {
    auto node = std::make_shared<rclcpp::Node>("node_test_node", "/ns");

    // Default constructing a NodeInterfaces object is not supported
    // auto invalid_interfaces = rclcpp::node_interfaces::get_node_interfaces<
    //   rclcpp::node_interfaces::Base
    // >();

    // TEST NODE CONSTRUCTORS ======================================================================

    using BaseInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base
    >;
    auto base_interfaces = std::make_shared<BaseInterfaces>(node);
    EXPECT_NE(nullptr, base_interfaces->get_node_base_interface());
    EXPECT_STREQ("node_test_node", base_interfaces->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_time_source_interface());

    using BaseInterfacesFromNodeObj = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base
    >;
    auto base_interfaces_from_node_obj = std::make_shared<BaseInterfacesFromNodeObj>(*node);
    EXPECT_NE(nullptr, base_interfaces_from_node_obj->get_node_base_interface());
    EXPECT_STREQ(
      "node_test_node",
      base_interfaces_from_node_obj->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_time_source_interface());

    auto base_interfaces_from_standalone = rclcpp::node_interfaces::get_node_interfaces<
      rclcpp::node_interfaces::Base
      >(node);
    EXPECT_NE(nullptr, base_interfaces_from_standalone->get_node_base_interface());
    EXPECT_STREQ(
      "node_test_node",
      base_interfaces_from_standalone->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_time_source_interface());

    using ClockInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base,
      rclcpp::node_interfaces::Clock
    >;
    auto base_clock_interfaces = std::make_shared<ClockInterfaces>(node);
    EXPECT_NE(nullptr, base_clock_interfaces->get_node_base_interface());
    EXPECT_STREQ("node_test_node", base_clock_interfaces->get_node_base_interface()->get_name());
    EXPECT_NE(nullptr, base_clock_interfaces->get_node_clock_interface());
    EXPECT_TRUE(
      RCL_ROS_TIME ==
      base_clock_interfaces->get_node_clock_interface()->get_clock()->get_clock_type());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_time_source_interface());

    using SansBaseInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Graph,
      rclcpp::node_interfaces::Logging,
      rclcpp::node_interfaces::Timers,
      rclcpp::node_interfaces::Topics,
      rclcpp::node_interfaces::Services,
      rclcpp::node_interfaces::Waitables,
      rclcpp::node_interfaces::Parameters,
      rclcpp::node_interfaces::TimeSource
    >;
    auto sans_base_clock_interfaces = std::make_shared<SansBaseInterfaces>(node);
    // The following will not be defined
    // EXPECT_EQ(nullptr, sans_base_clock_interfaces->get_node_base_interface());
    // EXPECT_EQ(nullptr, sans_base_clock_interfaces->get_node_clock_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_graph_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_logging_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_timers_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_topics_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_services_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_waitables_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_parameters_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_time_source_interface());
  }
}


/*
   Testing NodeInterfaces construction from aggregation.
 */
TEST_F(TestNodeInterfaces, test_aggregation) {
  {
    auto node = std::make_shared<rclcpp::Node>("aggregation_test_node", "/ns");

    // Default constructing a NodeInterfaces object is not supported
    // auto invalid_interfaces = rclcpp::node_interfaces::get_node_interfaces<
    //   rclcpp::node_interfaces::Base
    // >();

    using BaseInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base
    >;
    auto base_interfaces = std::make_shared<BaseInterfaces>(node->get_node_base_interface());
    EXPECT_NE(nullptr, base_interfaces->get_node_base_interface());
    EXPECT_STREQ("aggregation_test_node", base_interfaces->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces->get_node_time_source_interface());

    using BaseInterfacesFromNodeObj = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base
    >;
    auto base_interfaces_from_node_obj = std::make_shared<BaseInterfacesFromNodeObj>(
      (*node).get_node_base_interface()
    );
    EXPECT_NE(nullptr, base_interfaces_from_node_obj->get_node_base_interface());
    EXPECT_STREQ(
      "aggregation_test_node",
      base_interfaces_from_node_obj->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_node_obj->get_node_time_source_interface());

    auto base_interfaces_from_standalone = rclcpp::node_interfaces::get_node_interfaces<
      rclcpp::node_interfaces::Base
      >(node->get_node_base_interface());
    EXPECT_NE(nullptr, base_interfaces_from_standalone->get_node_base_interface());
    EXPECT_STREQ(
      "aggregation_test_node",
      base_interfaces_from_standalone->get_node_base_interface()->get_name());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_clock_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_interfaces_from_standalone->get_node_time_source_interface());

    using ClockInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Base,
      rclcpp::node_interfaces::Clock
    >;
    auto base_clock_interfaces = std::make_shared<ClockInterfaces>(
      node->get_node_base_interface(),
      node->get_node_clock_interface()
    );
    EXPECT_NE(nullptr, base_clock_interfaces->get_node_base_interface());
    EXPECT_STREQ(
      "aggregation_test_node",
      base_clock_interfaces->get_node_base_interface()->get_name());
    EXPECT_NE(nullptr, base_clock_interfaces->get_node_clock_interface());
    EXPECT_TRUE(
      RCL_ROS_TIME ==
      base_clock_interfaces->get_node_clock_interface()->get_clock()->get_clock_type());
    // The following will not be defined
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_graph_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_logging_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_timers_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_topics_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_services_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_waitables_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_parameters_interface());
    // EXPECT_EQ(nullptr, base_clock_interfaces->get_node_time_source_interface());

    using SansBaseInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Graph,
      rclcpp::node_interfaces::Logging,
      rclcpp::node_interfaces::Timers,
      rclcpp::node_interfaces::Topics,
      rclcpp::node_interfaces::Services,
      rclcpp::node_interfaces::Waitables,
      rclcpp::node_interfaces::Parameters,
      rclcpp::node_interfaces::TimeSource
    >;
    auto sans_base_clock_interfaces = std::make_shared<SansBaseInterfaces>(
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      node->get_node_waitables_interface(),
      node->get_node_parameters_interface(),
      node->get_node_time_source_interface()
    );
    // The following will not be defined
    // EXPECT_EQ(nullptr, sans_base_clock_interfaces->get_node_base_interface());
    // EXPECT_EQ(nullptr, sans_base_clock_interfaces->get_node_clock_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_graph_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_logging_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_timers_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_topics_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_services_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_waitables_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_parameters_interface());
    EXPECT_NE(nullptr, sans_base_clock_interfaces->get_node_time_source_interface());
  }
}


TEST_F(TestNodeInterfaces, test_conversions) {
  {
    auto node = std::make_shared<rclcpp::Node>("conversion_test_node", "/ns");

    using SansBaseInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::Graph,
      rclcpp::node_interfaces::Logging,
      rclcpp::node_interfaces::Timers,
      rclcpp::node_interfaces::Topics,
      rclcpp::node_interfaces::Services,
      rclcpp::node_interfaces::Waitables,
      rclcpp::node_interfaces::Parameters,
      rclcpp::node_interfaces::TimeSource
    >;
    auto sans_base_clock_interfaces = std::make_shared<SansBaseInterfaces>(
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      node->get_node_waitables_interface(),
      node->get_node_parameters_interface(),
      node->get_node_time_source_interface()
    );

    // Test conversions
    rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::Graph> subset_from_ptr =
      sans_base_clock_interfaces;

    rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::Graph> subset_from_obj =
      *sans_base_clock_interfaces;

    auto subset_from_standalone =
      rclcpp::node_interfaces::get_node_interfaces<rclcpp::node_interfaces::Graph>(
      sans_base_clock_interfaces);
    auto subset_from_standalone_obj =
      rclcpp::node_interfaces::get_node_interfaces<rclcpp::node_interfaces::Graph>(
      *sans_base_clock_interfaces);
  }
}
