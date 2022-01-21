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

TEST(TestQosParameters, declare) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.reliability", "best_effort"),
  }));

  for (size_t i = 0; i < 2; ++i) {
    // The first iteration will declare parameters, the second will get the previosuly declared
    // ones, check both have the same result.
    rclcpp::QoS qos{rclcpp::KeepLast(10)};
    qos = rclcpp::detail::declare_qos_parameters(
      rclcpp::QosOverridingOptions::with_default_policies(),
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
  }

  rclcpp::shutdown();
}

TEST(TestQosParameters, declare_with_callback) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.publisher.reliability", "best_effort"),
  }));

  rclcpp::QoS qos{rclcpp::KeepLast(10)};
  // *INDENT-OFF*, uncrustify suggestion makes the code unreadable
  EXPECT_THROW(
    rclcpp::detail::declare_qos_parameters(
      {
        {rclcpp::QosPolicyKind::Lifespan},
        [](const rclcpp::QoS &) {
          return rclcpp::QosCallbackResult{};
        }
      },
      node,
      "/my/fully/qualified/topic_name/fails_validation",
      qos,
      rclcpp::detail::PublisherQosParametersTraits{}),
    rclcpp::exceptions::InvalidQosOverridesException);

  rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions::with_default_policies([](const rclcpp::QoS &) {
      rclcpp::QosCallbackResult result;
      result.successful = true;
      return result;
    }),
    node,
    "/my/fully/qualified/topic_name",
    qos,
    rclcpp::detail::PublisherQosParametersTraits{});
  //  *INDENT-ON*

  rclcpp::shutdown();
}

constexpr int64_t kDuration{1000000};

TEST(TestQosParameters, declare_qos_subscription_parameters) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "my_node", "/ns", rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.subscription.reliability", "best_effort"),
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.subscription.deadline", kDuration),
    rclcpp::Parameter(
      "qos_overrides./my/fully/qualified/topic_name.subscription.liveliness_lease_duration",
      kDuration),
  }));

  for (size_t i = 0; i < 2; ++i) {
    // The first iteration will declare parameters, the second will get the previosuly declared
    // ones, check both have the same result.
    rclcpp::QoS qos{rclcpp::KeepLast(10)};
    qos = rclcpp::detail::declare_qos_parameters(
    {
      rclcpp::QosPolicyKind::AvoidRosNamespaceConventions, rclcpp::QosPolicyKind::Deadline,
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      // lifespan will be ignored
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Lifespan,
      rclcpp::QosPolicyKind::Liveliness, rclcpp::QosPolicyKind::LivelinessLeaseDuration,
      rclcpp::QosPolicyKind::Reliability
    },
      node,
      "/my/fully/qualified/topic_name",
      qos,
      rclcpp::detail::SubscriptionQosParametersTraits{});

    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.avoid_ros_namespace_conventions"
      ).get_value<bool>(), false);
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.deadline"
      ).get_value<int64_t>(), kDuration);
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.depth"
      ).get_value<int64_t>(), 10);
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.durability"
      ).get_value<std::string>(), "volatile");
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.history"
      ).get_value<std::string>(), "keep_last");
    EXPECT_FALSE(
      node->has_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.lifespan"));
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.liveliness"
      ).get_value<std::string>(), "system_default");
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.liveliness_lease_duration"
      ).get_value<int64_t>(), kDuration);
    EXPECT_EQ(
      node->get_parameter(
        "qos_overrides./my/fully/qualified/topic_name.subscription.reliability"
      ).get_value<std::string>(),
      "best_effort");

    std::map<std::string, rclcpp::Parameter> qos_params;
    EXPECT_TRUE(
      node->get_node_parameters_interface()->get_parameters_by_prefix(
        "qos_overrides./my/fully/qualified/topic_name.subscription", qos_params));
    EXPECT_EQ(8u, qos_params.size());
  }
  rclcpp::shutdown();
}

TEST(TestQosParameters, declare_with_id) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  qos = rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions::with_default_policies(nullptr, "my_id"),
    node,
    "/my/fully/qualified/topic_name",
    qos,
    rclcpp::detail::PublisherQosParametersTraits{});

  std::map<std::string, rclcpp::Parameter> qos_params;
  EXPECT_TRUE(
    node->get_node_parameters_interface()->get_parameters_by_prefix(
      "qos_overrides./my/fully/qualified/topic_name.publisher_my_id", qos_params));
  EXPECT_EQ(3u, qos_params.size());

  rclcpp::shutdown();
}

TEST(TestQosParameters, declare_no_parameters_interface) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  auto node_base_interface = node->get_node_base_interface();
  EXPECT_THROW(
    rclcpp::detail::declare_qos_parameters(
      rclcpp::QosOverridingOptions::with_default_policies(),
      node_base_interface,
      "/my/fully/qualified/topic_name",
      qos,
      rclcpp::detail::PublisherQosParametersTraits{}),
    std::runtime_error);

  qos = rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions{},
    node_base_interface,
    "/my/fully/qualified/topic_name",
    qos,
    rclcpp::detail::PublisherQosParametersTraits{});

  std::map<std::string, rclcpp::Parameter> qos_params;
  EXPECT_FALSE(
    node->get_node_parameters_interface()->get_parameters_by_prefix(
      "qos_overrides./my/fully/qualified/topic_name", qos_params));

  rclcpp::shutdown();
}

TEST(TestQosParameters, internal_functions_failure_modes) {
  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  EXPECT_THROW(
    rclcpp::detail::apply_qos_override(
      rclcpp::QosPolicyKind::Invalid, rclcpp::ParameterValue{}, qos),
    std::invalid_argument);
  EXPECT_THROW(
    rclcpp::detail::get_default_qos_param_value(
      rclcpp::QosPolicyKind::Invalid, qos),
    std::invalid_argument);
  EXPECT_THROW(
    rclcpp::detail::check_if_stringified_policy_is_null(
      nullptr, rclcpp::QosPolicyKind::Reliability),
    std::invalid_argument);
}
