// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <map>
#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/detail/qos_parameters.hpp"

TEST(TestQosParameters, test_overriding_options) {
  {
    rclcpp::QosOverridingOptions options{true};
    EXPECT_EQ(options.id, "");
    EXPECT_EQ(options.validation_callback, nullptr);
    EXPECT_THAT(
      options.policy_kinds, testing::ElementsAre(
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Reliability));
  }
}

TEST(TestQosParameters, declare) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.reliability", "best_effort"),
  }));

  rclcpp::QoS qos{rclcpp::KeepLast(10)};
  qos = rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions{true},
    node,
    "/my/fully/qualified/topic_name",
    qos,
    rclcpp::detail::PublisherQosParametersTraits{});

  EXPECT_EQ(
    node->get_parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.history").get_value<std::string>(),
    "keep_last");
  EXPECT_EQ(
    node->get_parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.depth").get_value<int64_t>(),
    10);
  EXPECT_EQ(
    node->get_parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.reliability"
    ).get_value<std::string>(),
    "best_effort");
  EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_LAST, qos.get_rmw_qos_profile().history);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, qos.get_rmw_qos_profile().reliability);
  EXPECT_EQ(10u, qos.get_rmw_qos_profile().depth);

  std::map<std::string, rclcpp::Parameter> qos_params;
  EXPECT_TRUE(
    node->get_node_parameters_interface()->get_parameters_by_prefix(
      "qos_overrides./my/fully/qualified/topic_name.publisher", qos_params));
  EXPECT_EQ(3u, qos_params.size());

  rclcpp::shutdown();
}

// TEST(TestQosParameters, declare_with_callback) {
//   rclcpp::init(0, nullptr);
//   auto node = std::make_shared<rclcpp::Node>(
//     "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
//   {
//     rclcpp::Parameter(
//       "qos_overrides./my/fully/qualified/topic_name.publisher.reliability", "best_effort"),
//   }));

//   rclcpp::QoS qos{rclcpp::KeepLast(10)};
//   qos = rclcpp::detail::declare_qos_parameters(
//     rclcpp::QosOverridingOptions{[](const )},
//     node,
//     "/my/fully/qualified/topic_name",
//     qos,
//     rclcpp::detail::PublisherQosParametersTraits{});

//   EXPECT_EQ(
//     node->get_parameter(
//       "qos_overrides./my/fully/qualified/topic_name.publisher.history").get_value<std::string>(),
//     "keep_last");
//   EXPECT_EQ(
//     node->get_parameter(
//       "qos_overrides./my/fully/qualified/topic_name.publisher.depth").get_value<int64_t>(),
//     10);
//   EXPECT_EQ(
//     node->get_parameter(
//       "qos_overrides./my/fully/qualified/topic_name.publisher.reliability"
//     ).get_value<std::string>(),
//     "best_effort");
//   EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_LAST, qos.get_rmw_qos_profile().history);
//   EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, qos.get_rmw_qos_profile().reliability);
//   EXPECT_EQ(10u, qos.get_rmw_qos_profile().depth);

//   std::map<std::string, rclcpp::Parameter> qos_params;
//   EXPECT_TRUE(
//     node->get_node_parameters_interface()->get_parameters_by_prefix(
//       "qos_overrides./my/fully/qualified/topic_name.publisher", qos_params));
//   EXPECT_EQ(3u, qos_params.size());

//   rclcpp::shutdown();
// }

TEST(TestQosParameters, qos_parameters_created_by_one_node) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns");

  // Total number of qos parameters created by a node
  // Up to now, /rosout and /parameter_events
  // aren't creating qos parameters.
  std::map<std::string, rclcpp::Parameter> qos_params;
  EXPECT_FALSE(
    node->get_node_parameters_interface()->get_parameters_by_prefix(
      "qos_overrides", qos_params));

  rclcpp::shutdown();
}

TEST(TestQosParameters, qos_parameters_created_by_one_node_with_use_sim_time) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("use_sim_time", true)
  }));

  // Total number of qos parameters for the /clock topic is 4.
  std::map<std::string, rclcpp::Parameter> qos_params;
  EXPECT_TRUE(
    node->get_node_parameters_interface()->get_parameters_by_prefix(
      "qos_overrides", qos_params));
  EXPECT_EQ(4u, qos_params.size());

  rclcpp::shutdown();
}
