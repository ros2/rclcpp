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

/*
  Test the equality operator for QoS profiles.
 */
TEST(TestQoS, equality_basic) {
  rclcpp::QoS profile_a(10);
  rclcpp::QoS profile_b(10);
  EXPECT_EQ(profile_a, profile_b);

  profile_a = rclcpp::QoS(5);
  EXPECT_NE(profile_a, profile_b);

  profile_a.keep_all().reliable();
  profile_b.keep_all().reliable();
  EXPECT_EQ(profile_a, profile_b);

  profile_b.transient_local();
  EXPECT_NE(profile_a, profile_b);

  profile_a.transient_local();
  EXPECT_EQ(profile_a, profile_b);

  rmw_time_t deadline{0, 1000};
  profile_a.deadline(deadline);
  EXPECT_NE(profile_a, profile_b);
}
