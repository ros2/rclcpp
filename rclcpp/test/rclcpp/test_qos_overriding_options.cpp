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

#include "gmock/gmock.h"

#include "rclcpp/qos_overriding_options.hpp"

TEST(TestQosOverridingOptions, test_overriding_options) {
  auto options = rclcpp::QosOverridingOptions::with_default_policies();
  EXPECT_EQ(options.get_id(), "");
  EXPECT_EQ(options.get_validation_callback(), nullptr);
  EXPECT_THAT(
    options.get_policy_kinds(), testing::ElementsAre(
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Reliability));
}

TEST(TestQosOverridingOptions, test_qos_policy_kind_to_cstr) {
  EXPECT_THROW(
    rclcpp::qos_policy_kind_to_cstr(rclcpp::QosPolicyKind::Invalid),
    std::invalid_argument);
}
