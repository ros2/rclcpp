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

#include <string>

#include "rclcpp/qos.hpp"

#include "rmw/types.h"

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

TEST(TestQoS, setters_and_getters) {
  rclcpp::QoS qos(10);

  qos.keep_all();
  EXPECT_EQ(rclcpp::HistoryPolicy::KeepAll, qos.history());

  qos.keep_last(20);
  EXPECT_EQ(rclcpp::HistoryPolicy::KeepLast, qos.history());
  EXPECT_EQ(20u, qos.depth());

  qos.reliable();
  EXPECT_EQ(rclcpp::ReliabilityPolicy::Reliable, qos.reliability());

  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(rclcpp::ReliabilityPolicy::BestEffort, qos.reliability());

  qos.durability_volatile();
  EXPECT_EQ(rclcpp::DurabilityPolicy::Volatile, qos.durability());

  qos.transient_local();
  EXPECT_EQ(rclcpp::DurabilityPolicy::TransientLocal, qos.durability());

  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  EXPECT_EQ(rclcpp::DurabilityPolicy::Volatile, qos.durability());

  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
  EXPECT_EQ(rclcpp::HistoryPolicy::KeepAll, qos.history());

  qos.history(rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(rclcpp::HistoryPolicy::KeepLast, qos.history());

  constexpr rcl_duration_value_t duration_ns = 12345;
  constexpr std::chrono::nanoseconds duration(duration_ns);
  qos.deadline(duration);
  EXPECT_EQ(duration_ns, qos.deadline().nanoseconds());

  const rmw_time_t rmw_time {0, 54321};
  qos.deadline(rmw_time);
  EXPECT_EQ(rmw_time.sec, qos.get_rmw_qos_profile().deadline.sec);
  EXPECT_EQ(rmw_time.nsec, qos.get_rmw_qos_profile().deadline.nsec);

  qos.lifespan(duration);
  EXPECT_EQ(duration_ns, qos.lifespan().nanoseconds());

  qos.lifespan(rmw_time);
  EXPECT_EQ(rmw_time.sec, qos.get_rmw_qos_profile().lifespan.sec);
  EXPECT_EQ(rmw_time.nsec, qos.get_rmw_qos_profile().lifespan.nsec);

  qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  EXPECT_EQ(rclcpp::LivelinessPolicy::ManualByTopic, qos.liveliness());

  qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  EXPECT_EQ(rclcpp::LivelinessPolicy::Automatic, qos.liveliness());

  qos.liveliness_lease_duration(duration);
  EXPECT_EQ(duration_ns, qos.liveliness_lease_duration().nanoseconds());

  qos.liveliness_lease_duration(rmw_time);
  EXPECT_EQ(rmw_time.sec, qos.get_rmw_qos_profile().liveliness_lease_duration.sec);
  EXPECT_EQ(rmw_time.nsec, qos.get_rmw_qos_profile().liveliness_lease_duration.nsec);

  qos.avoid_ros_namespace_conventions(true);
  EXPECT_TRUE(qos.avoid_ros_namespace_conventions());
  qos.avoid_ros_namespace_conventions(false);
  EXPECT_FALSE(qos.avoid_ros_namespace_conventions());
}

bool operator==(const rmw_qos_profile_t & lhs, const rmw_qos_profile_t & rhs)
{
  if (lhs.history != rhs.history) {
    return false;
  }
  switch (lhs.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return true;
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      return lhs.depth == rhs.depth;
  }
  throw std::runtime_error("This line shouldn't be reached");
}

TEST(TestQoS, DerivedTypes) {
  rclcpp::SensorDataQoS sensor_data_qos;
  EXPECT_EQ(rmw_qos_profile_sensor_data, sensor_data_qos.get_rmw_qos_profile());

  rclcpp::ParametersQoS parameter_qos;
  EXPECT_EQ(rmw_qos_profile_parameters, parameter_qos.get_rmw_qos_profile());

  rclcpp::ServicesQoS services_qos;
  EXPECT_EQ(rmw_qos_profile_services_default, services_qos.get_rmw_qos_profile());

  rclcpp::ParameterEventsQoS parameter_events_qos;
  EXPECT_EQ(rmw_qos_profile_parameter_events, parameter_events_qos.get_rmw_qos_profile());

  rclcpp::RosoutQoS rosout_qos;
  EXPECT_EQ(rcl_qos_profile_rosout_default, rosout_qos.get_rmw_qos_profile());

  rclcpp::SystemDefaultsQoS system_default_qos;
  const rclcpp::KeepLast expected_initialization(RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT);
  const rclcpp::QoS expected_default(expected_initialization);
  EXPECT_EQ(expected_default.get_rmw_qos_profile(), system_default_qos.get_rmw_qos_profile());
}

TEST(TestQoS, policy_name_from_kind) {
  EXPECT_EQ(
    "DURABILITY_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_DURABILITY));

  EXPECT_EQ(
    "DEADLINE_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_DEADLINE));

  EXPECT_EQ(
    "LIVELINESS_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_LIVELINESS));

  EXPECT_EQ(
    "RELIABILITY_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_RELIABILITY));

  EXPECT_EQ(
    "HISTORY_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_HISTORY));

  EXPECT_EQ(
    "LIFESPAN_QOS_POLICY",
    rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_LIFESPAN));
}

TEST(TestQoS, qos_check_compatible)
{
  // Compatible
  {
    rclcpp::QoS qos = rclcpp::QoS(1)
      .reliable()
      .durability_volatile()
      .deadline(rclcpp::Duration(1, 0u))
      .lifespan(rclcpp::Duration(1, 0u))
      .liveliness(rclcpp::LivelinessPolicy::Automatic)
      .liveliness_lease_duration(rclcpp::Duration(1, 0u));
    rclcpp::QoSCheckCompatibleResult ret = rclcpp::qos_check_compatible(qos, qos);
    EXPECT_EQ(ret.compatibility, rclcpp::QoSCompatibility::Ok);
    EXPECT_EQ(ret.reason, std::string(""));
  }

  // Note, the following incompatible tests assume we are using a DDS middleware,
  // and may not be valid for other RMWs.
  // TODO(jacobperron): programmatically check if current RMW is one of the officially
  //                    supported DDS middlewares before running the following tests

  // Incompatible
  {
    rclcpp::QoS pub_qos = rclcpp::QoS(1).best_effort();
    rclcpp::QoS sub_qos = rclcpp::QoS(1).reliable();
    rclcpp::QoSCheckCompatibleResult ret = rclcpp::qos_check_compatible(pub_qos, sub_qos);
    EXPECT_EQ(ret.compatibility, rclcpp::QoSCompatibility::Error);
    EXPECT_FALSE(ret.reason.empty());
  }

  // Warn of possible incompatibility
  {
    rclcpp::SystemDefaultsQoS pub_qos;
    rclcpp::QoS sub_qos = rclcpp::QoS(1).reliable();
    rclcpp::QoSCheckCompatibleResult ret = rclcpp::qos_check_compatible(pub_qos, sub_qos);
    EXPECT_EQ(ret.compatibility, rclcpp::QoSCompatibility::Warning);
    EXPECT_FALSE(ret.reason.empty());
  }
}
