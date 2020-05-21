// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rclcpp/qos.hpp"

TEST(TestQoS, equality_history) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  EXPECT_EQ(a, b);
  a.keep_last(5);
  EXPECT_NE(a, b);
  a.keep_all();
  b.keep_all();
  EXPECT_EQ(a, b);
}

TEST(TestQoS, equality_reliability) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  b.best_effort();
  EXPECT_NE(a, b);
}

TEST(TestQoS, equality_durability) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  a.transient_local();
  EXPECT_NE(a, b);
}

TEST(TestQoS, equality_deadline) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  rmw_time_t deadline{0, 1000};
  a.deadline(deadline);
  EXPECT_NE(a, b);
}

TEST(TestQoS, equality_lifespan) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  rmw_time_t lifespan{3, 0};
  a.lifespan(lifespan);
  EXPECT_NE(a, b);
}

TEST(TestQoS, equality_liveliness) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  rmw_time_t duration{0, 1000000};
  a.liveliness_lease_duration(duration);
  EXPECT_NE(a, b);
  b.liveliness_lease_duration(duration);
  EXPECT_EQ(a, b);
  a.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  EXPECT_NE(a, b);
}

TEST(TestQoS, equality_namespace) {
  rclcpp::QoS a(10);
  rclcpp::QoS b(10);
  a.avoid_ros_namespace_conventions(true);
  EXPECT_NE(a, b);
}
