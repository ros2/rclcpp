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

class TestRosoutQoS : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_options = std::make_shared<rclcpp::NodeOptions>();
  }

  void InitializeNodeOptions(const rmw_qos_profile_t rmw_qos_options = rmw_qos_profile_default)
  {
    rclcpp::QoS qos_profiles(rmw_qos_options);
    node_options->rosout_qos(qos_profiles);
  }

  void TearDown()
  {
    node_options.reset();
  }

  std::shared_ptr<rclcpp::NodeOptions> node_options;
};

/*
   Test rosout_qos function with default value.
 */
TEST_F(TestRosoutQoS, test_rosout_qos_with_default_value) {
  rclcpp::QoS rosout_qos_profile = rclcpp::NodeOptions().rosout_qos();
  rmw_qos_profile_t rmw_qos_profile = rosout_qos_profile.get_rmw_qos_profile();
  EXPECT_EQ(rcl_qos_profile_rosout_default, rmw_qos_profile);
}

/*
   Test `rosout_qos` function with user-defined value.
 */
TEST_F(TestRosoutQoS, test_rosout_qos_with_custom_value) {
  rmw_time_t life_span;
  life_span.sec = 10;
  life_span.nsec = 0;
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local().lifespan(life_span);
  auto options = rclcpp::NodeOptions().rosout_qos(qos_profile);
  rclcpp::QoS rosout_qos = options.rosout_qos();
  rmw_qos_profile_t rmw_qos_profile = rosout_qos.get_rmw_qos_profile();

  EXPECT_EQ((size_t)1000, rmw_qos_profile.depth);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, rmw_qos_profile.durability);
  EXPECT_EQ(life_span, rmw_qos_profile.lifespan);
}

/*
   Test the QoS's constructor which has `rmw_qos_profile_t` as the parameter.
 */
TEST_F(TestRosoutQoS, test_qos_constructor) {
  rmw_qos_profile_t rosout_qos_profile = rcl_qos_profile_rosout_default;
  rclcpp::QoS qos = rclcpp::QoS(rosout_qos_profile);
  rmw_qos_profile_t rmw_qos_profile = qos.get_rmw_qos_profile();
  EXPECT_EQ(rcl_qos_profile_rosout_default, rmw_qos_profile);
}

/*
   Test `get_rcl_node_options` function.
 */
TEST_F(TestRosoutQoS, test_get_rcl_node_options) {
  rmw_qos_profile_t rosout_qos_profile = rcl_qos_profile_rosout_default;
  {
    InitializeNodeOptions(rosout_qos_profile);
    EXPECT_EQ(rcl_qos_profile_rosout_default, node_options->get_rcl_node_options()->rosout_qos);
  }

  {
    rosout_qos_profile.depth = 10;
    rosout_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    InitializeNodeOptions(rosout_qos_profile);
    EXPECT_NE(rcl_qos_profile_rosout_default, node_options->get_rcl_node_options()->rosout_qos);
  }
}

/*
   Test `rcl_node_init` function
 */
TEST_F(TestRosoutQoS, test_rcl_node_init) {
  rcl_ret_t ret;

  // Initialize rcl with rcl_init().
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_context_t context = rcl_get_zero_initialized_context();
  ret = rcl_init(0, nullptr, &init_options, &context);
  ASSERT_EQ(RCL_RET_OK, ret);

  rcl_node_options_t default_options = rcl_node_get_default_options();

  // First execute `rcl_node_init` use the default rosout qos configuration.
  {
    rcl_node_t node = rcl_get_zero_initialized_node();
    ret = rcl_node_init(&node, "node", "/ns", &context, &default_options);
    ASSERT_EQ(RCL_RET_OK, ret);

    const rcl_node_options_t * options = rcl_node_get_options(&node);
    EXPECT_EQ(rcl_qos_profile_rosout_default, options->rosout_qos);

    rcl_ret_t ret = rcl_node_fini(&node);
    EXPECT_EQ(RCL_RET_OK, ret);
  }

  // Then execute `rcl_node_init` use the custom rosout qos configuration.
  {
    static const rmw_qos_profile_t custom_rosout_qos_profile =
    {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      10,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      {1, 23456},
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    default_options.rosout_qos = custom_rosout_qos_profile;
    rcl_node_t node = rcl_get_zero_initialized_node();
    ret = rcl_node_init(&node, "node", "/ns", &context, &default_options);
    ASSERT_EQ(RCL_RET_OK, ret);

    const rcl_node_options_t * options = rcl_node_get_options(&node);
    EXPECT_EQ(custom_rosout_qos_profile, options->rosout_qos);
    EXPECT_NE(rcl_qos_profile_rosout_default, options->rosout_qos);

    rcl_ret_t ret = rcl_node_fini(&node);
    EXPECT_EQ(RCL_RET_OK, ret);
  }
}
