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

#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"
#include "rcl/logging_rosout.h"
#include "rcl/init.h"
#include "rmw/types.h"

bool operator==(
  const rmw_time_t & lhs,
  const rmw_time_t & rhs)
{
  return lhs.sec == rhs.sec && lhs.nsec == rhs.nsec;
}

bool operator==(
  const rmw_qos_profile_t & lhs,
  const rmw_qos_profile_t & rhs)
{
  return lhs.history == rhs.history &&
         lhs.depth == rhs.depth &&
         lhs.reliability == rhs.reliability &&
         lhs.durability == rhs.durability &&
         lhs.deadline == rhs.deadline &&
         lhs.lifespan == rhs.lifespan &&
         lhs.liveliness == rhs.liveliness &&
         lhs.liveliness_lease_duration == rhs.liveliness_lease_duration &&
         lhs.avoid_ros_namespace_conventions == rhs.avoid_ros_namespace_conventions;
}

bool operator!=(
  const rmw_qos_profile_t & lhs,
  const rmw_qos_profile_t & rhs)
{
  return !(lhs == rhs);
}

/*
   Test rosout_qos function with default value.
 */
TEST(TestRosoutQoS, test_rosout_qos_with_default_value) {
  rclcpp::NodeOptions node_options;
  rclcpp::QoS rosout_qos_profile = node_options.rosout_qos();
  rmw_qos_profile_t rmw_qos_profile = rosout_qos_profile.get_rmw_qos_profile();
  EXPECT_EQ(rcl_qos_profile_rosout_default, rmw_qos_profile);
  EXPECT_EQ(rcl_qos_profile_rosout_default, node_options.get_rcl_node_options()->rosout_qos);
}

/*
   Test `rosout_qos` function with custom value.
 */
TEST(TestRosoutQoS, test_rosout_qos_with_custom_value) {
  rmw_time_t life_span;
  life_span.sec = 10;
  life_span.nsec = 0;
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local().lifespan(life_span);
  auto options = rclcpp::NodeOptions().rosout_qos(qos_profile);
  rclcpp::QoS rosout_qos = options.rosout_qos();
  rmw_qos_profile_t rmw_qos_profile = rosout_qos.get_rmw_qos_profile();

  EXPECT_EQ(rmw_qos_profile, qos_profile.get_rmw_qos_profile());
  EXPECT_EQ(rmw_qos_profile, options.get_rcl_node_options()->rosout_qos);
}
