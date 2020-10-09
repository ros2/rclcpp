// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <limits>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

namespace
{

bool logical_eq(const bool a, const bool b)
{
  return (a && b) || ((!a) && !(b));
}

}  // namespace


class TestTime : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestTime, clock_type_access) {
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  EXPECT_EQ(RCL_ROS_TIME, ros_clock.get_clock_type());

  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  EXPECT_EQ(RCL_SYSTEM_TIME, system_clock.get_clock_type());

  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  EXPECT_EQ(RCL_STEADY_TIME, steady_clock.get_clock_type());
}

// Check that the clock may go out of the scope before the jump callback without leading in UB.
TEST_F(TestTime, clock_jump_callback_destruction_order) {
  rclcpp::JumpHandler::SharedPtr handler;
  {
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rcl_jump_threshold_t threshold;
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    handler = ros_clock.create_jump_callback([]() {}, [](const rcl_time_jump_t &) {}, threshold);
  }
}

TEST_F(TestTime, time_sources) {
  using builtin_interfaces::msg::Time;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  Time ros_now = ros_clock.now();
  EXPECT_NE(0, ros_now.sec);
  EXPECT_NE(0u, ros_now.nanosec);

  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  Time system_now = system_clock.now();
  EXPECT_NE(0, system_now.sec);
  EXPECT_NE(0u, system_now.nanosec);

  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  Time steady_now = steady_clock.now();
  EXPECT_NE(0, steady_now.sec);
  EXPECT_NE(0u, steady_now.nanosec);
}

static const int64_t HALF_SEC_IN_NS = 500 * 1000 * 1000;
static const int64_t ONE_SEC_IN_NS = 1000 * 1000 * 1000;
static const int64_t ONE_AND_HALF_SEC_IN_NS = 3 * HALF_SEC_IN_NS;

TEST_F(TestTime, conversions) {
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

  {
    rclcpp::Time now = system_clock.now();
    builtin_interfaces::msg::Time now_msg = now;

    rclcpp::Time now_again = now_msg;
    EXPECT_EQ(now.nanoseconds(), now_again.nanoseconds());
  }

  {
    rclcpp::Time positive_time = rclcpp::Time(12345, 67890u);

    builtin_interfaces::msg::Time msg = positive_time;
    EXPECT_EQ(msg.sec, 12345);
    EXPECT_EQ(msg.nanosec, 67890u);

    rclcpp::Time time = msg;
    EXPECT_EQ(time.nanoseconds(), positive_time.nanoseconds());
    EXPECT_EQ(
      RCL_S_TO_NS(static_cast<int64_t>(msg.sec)) + static_cast<int64_t>(msg.nanosec),
      time.nanoseconds());
    EXPECT_EQ(static_cast<int64_t>(msg.sec), RCL_NS_TO_S(time.nanoseconds()));
  }

  // throw on construction/assignment of negative times
  {
    builtin_interfaces::msg::Time negative_time_msg;
    negative_time_msg.sec = -1;
    negative_time_msg.nanosec = 1;

    EXPECT_ANY_THROW(
    {
      rclcpp::Time negative_time = negative_time_msg;
    });

    EXPECT_ANY_THROW(rclcpp::Time(-1, 1));

    EXPECT_ANY_THROW(
    {
      rclcpp::Time assignment(1, 2);
      assignment = negative_time_msg;
    });
  }

  {
    const rclcpp::Time time(HALF_SEC_IN_NS);
    const auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, 0);
    EXPECT_EQ(time_msg.nanosec, HALF_SEC_IN_NS);
    EXPECT_EQ(rclcpp::Time(time_msg).nanoseconds(), HALF_SEC_IN_NS);
  }

  {
    const rclcpp::Time time(ONE_SEC_IN_NS);
    const auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, 1);
    EXPECT_EQ(time_msg.nanosec, 0u);
    EXPECT_EQ(rclcpp::Time(time_msg).nanoseconds(), ONE_SEC_IN_NS);
  }

  {
    const rclcpp::Time time(ONE_AND_HALF_SEC_IN_NS);
    auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, 1);
    EXPECT_EQ(time_msg.nanosec, HALF_SEC_IN_NS);
    EXPECT_EQ(rclcpp::Time(time_msg).nanoseconds(), ONE_AND_HALF_SEC_IN_NS);
  }

  {
    // Can rclcpp::Time be negative or not? The following constructor works:
    rclcpp::Time time(-HALF_SEC_IN_NS);
    auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, -1);
    EXPECT_EQ(time_msg.nanosec, HALF_SEC_IN_NS);

    // The opposite conversion throws...
    EXPECT_ANY_THROW(
    {
      rclcpp::Time negative_time(time_msg);
    });
  }

  {
    // Can rclcpp::Time be negative or not? The following constructor works:
    rclcpp::Time time(-ONE_SEC_IN_NS);
    auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, -1);
    EXPECT_EQ(time_msg.nanosec, 0u);

    // The opposite conversion throws...
    EXPECT_ANY_THROW(
    {
      rclcpp::Time negative_time(time_msg);
    });
  }

  {
    // Can rclcpp::Time be negative or not? The following constructor works:
    rclcpp::Time time(-ONE_AND_HALF_SEC_IN_NS);
    auto time_msg = static_cast<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(time_msg.sec, -2);
    EXPECT_EQ(time_msg.nanosec, HALF_SEC_IN_NS);

    // The opposite conversion throws...
    EXPECT_ANY_THROW(
    {
      rclcpp::Time negative_time(time_msg);
    });
  }
}

TEST_F(TestTime, operators) {
  rclcpp::Time old(1, 0);
  rclcpp::Time young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);
  EXPECT_TRUE(young != old);

  rclcpp::Duration sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), (young.nanoseconds() - old.nanoseconds()));
  EXPECT_EQ(sub, young - old);

  rclcpp::Time young_changed(young);
  young_changed -= rclcpp::Duration(old.nanoseconds());
  EXPECT_EQ(sub.nanoseconds(), young_changed.nanoseconds());

  rclcpp::Time system_time(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time steady_time(0, 0, RCL_STEADY_TIME);

  EXPECT_ANY_THROW((void)(system_time == steady_time));
  EXPECT_ANY_THROW((void)(system_time != steady_time));
  EXPECT_ANY_THROW((void)(system_time <= steady_time));
  EXPECT_ANY_THROW((void)(system_time >= steady_time));
  EXPECT_ANY_THROW((void)(system_time < steady_time));
  EXPECT_ANY_THROW((void)(system_time > steady_time));
  EXPECT_ANY_THROW((void)(system_time - steady_time));

  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  rclcpp::Time now = system_clock.now();
  rclcpp::Time later = steady_clock.now();

  EXPECT_ANY_THROW((void)(now == later));
  EXPECT_ANY_THROW((void)(now != later));
  EXPECT_ANY_THROW((void)(now <= later));
  EXPECT_ANY_THROW((void)(now >= later));
  EXPECT_ANY_THROW((void)(now < later));
  EXPECT_ANY_THROW((void)(now > later));
  EXPECT_ANY_THROW((void)(now - later));

  for (auto time_source : {RCL_ROS_TIME, RCL_SYSTEM_TIME, RCL_STEADY_TIME}) {
    rclcpp::Time time = rclcpp::Time(0, 0, time_source);
    rclcpp::Time copy_constructor_time(time);
    rclcpp::Time assignment_op_time = rclcpp::Time(1, 0, time_source);
    assignment_op_time = time;

    EXPECT_TRUE(time == copy_constructor_time);
    EXPECT_TRUE(time == assignment_op_time);
  }
}

TEST_F(TestTime, overflow_detectors) {
  /////////////////////////////////////////////////////////////////////////////
  // Test logical_eq call first:
  EXPECT_TRUE(logical_eq(false, false));
  EXPECT_FALSE(logical_eq(false, true));
  EXPECT_FALSE(logical_eq(true, false));
  EXPECT_TRUE(logical_eq(true, true));

  /////////////////////////////////////////////////////////////////////////////
  // Exhaustive test of all int8_t values
  using test_type_t = int8_t;
  // big_type_t encompasses test_type_t:
  //  big_type_t::min < test_type_t::min
  //  big_type_t::max > test_type_t::max
  using big_type_t = int16_t;
  const big_type_t min_val = std::numeric_limits<test_type_t>::min();
  const big_type_t max_val = std::numeric_limits<test_type_t>::max();
  // 256 * 256 = 64K total loops, should be pretty fast on everything
  for (big_type_t y = min_val; y <= max_val; ++y) {
    for (big_type_t x = min_val; x <= max_val; ++x) {
      const big_type_t sum = static_cast<big_type_t>(x + y);
      const big_type_t diff = static_cast<big_type_t>(x - y);

      const bool add_will_overflow =
        rclcpp::add_will_overflow(test_type_t(x), test_type_t(y));
      const bool add_did_overflow = sum > max_val;
      EXPECT_TRUE(logical_eq(add_will_overflow, add_did_overflow));

      const bool add_will_underflow =
        rclcpp::add_will_underflow(test_type_t(x), test_type_t(y));
      const bool add_did_underflow = sum < min_val;
      EXPECT_TRUE(logical_eq(add_will_underflow, add_did_underflow));

      const bool sub_will_overflow =
        rclcpp::sub_will_overflow(test_type_t(x), test_type_t(y));
      const bool sub_did_overflow = diff > max_val;
      EXPECT_TRUE(logical_eq(sub_will_overflow, sub_did_overflow));

      const bool sub_will_underflow =
        rclcpp::sub_will_underflow(test_type_t(x), test_type_t(y));
      const bool sub_did_underflow = diff < min_val;
      EXPECT_TRUE(logical_eq(sub_will_underflow, sub_did_underflow));
    }
  }

  // Few selected tests for int64_t
  EXPECT_TRUE(rclcpp::add_will_overflow<int64_t>(INT64_MAX, 1));
  EXPECT_FALSE(rclcpp::add_will_overflow<int64_t>(INT64_MAX, -1));
  EXPECT_TRUE(rclcpp::add_will_underflow<int64_t>(INT64_MIN, -1));
  EXPECT_FALSE(rclcpp::add_will_underflow<int64_t>(INT64_MIN, 1));

  EXPECT_FALSE(rclcpp::sub_will_overflow<int64_t>(INT64_MAX, 1));
  EXPECT_TRUE(rclcpp::sub_will_overflow<int64_t>(INT64_MAX, -1));
  EXPECT_FALSE(rclcpp::sub_will_underflow<int64_t>(INT64_MIN, -1));
  EXPECT_TRUE(rclcpp::sub_will_underflow<int64_t>(INT64_MIN, 1));
}

TEST_F(TestTime, overflows) {
  rclcpp::Time max_time(std::numeric_limits<rcl_time_point_value_t>::max());
  rclcpp::Time min_time(std::numeric_limits<rcl_time_point_value_t>::min());
  rclcpp::Duration one(1);
  rclcpp::Duration two(2);

  // Cross min/max
  EXPECT_THROW(max_time + one, std::overflow_error);
  EXPECT_THROW(min_time - one, std::underflow_error);
  EXPECT_THROW(max_time - min_time, std::overflow_error);
  EXPECT_THROW(min_time - max_time, std::underflow_error);
  EXPECT_THROW(rclcpp::Time(max_time) += one, std::overflow_error);
  EXPECT_THROW(rclcpp::Time(min_time) -= one, std::underflow_error);
  EXPECT_NO_THROW(max_time - max_time);
  EXPECT_NO_THROW(min_time - min_time);

  // Cross zero in both directions
  rclcpp::Time one_time(1);
  EXPECT_NO_THROW(one_time - two);
  EXPECT_NO_THROW(rclcpp::Time(one_time) -= two);

  rclcpp::Time minus_one_time(-1);
  EXPECT_NO_THROW(minus_one_time + two);
  EXPECT_NO_THROW(rclcpp::Time(minus_one_time) += two);

  EXPECT_NO_THROW(one_time - minus_one_time);
  EXPECT_NO_THROW(minus_one_time - one_time);

  rclcpp::Time two_time(2);
  EXPECT_NO_THROW(one_time - two_time);
}

TEST_F(TestTime, seconds) {
  EXPECT_DOUBLE_EQ(0.0, rclcpp::Time(0, 0).seconds());
  EXPECT_DOUBLE_EQ(4.5, rclcpp::Time(4, 500000000).seconds());
  EXPECT_DOUBLE_EQ(2.5, rclcpp::Time(0, 2500000000).seconds());
}

TEST_F(TestTime, test_max) {
  const rclcpp::Time time_max = rclcpp::Time::max();
  const rclcpp::Time max_time(std::numeric_limits<int32_t>::max(), 999999999);
  EXPECT_DOUBLE_EQ(max_time.seconds(), time_max.seconds());
  EXPECT_EQ(max_time.nanoseconds(), time_max.nanoseconds());
}

TEST_F(TestTime, test_constructor_from_rcl_time_point) {
  const rcl_time_point_value_t test_nano_seconds = 555;
  const rcl_clock_type_t test_clock_type = RCL_ROS_TIME;
  rcl_time_point_t test_time_point;
  test_time_point.nanoseconds = test_nano_seconds;
  test_time_point.clock_type = test_clock_type;

  const rclcpp::Time time_max = rclcpp::Time(test_time_point);

  EXPECT_EQ(test_nano_seconds, time_max.nanoseconds());
  EXPECT_EQ(test_nano_seconds, test_time_point.nanoseconds);
  EXPECT_EQ(test_clock_type, time_max.get_clock_type());
  EXPECT_EQ(test_clock_type, test_time_point.clock_type);
}

TEST_F(TestTime, test_assignment_operator_from_builtin_msg_time) {
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  const builtin_interfaces::msg::Time ros_now = ros_clock.now();
  EXPECT_NE(0, ros_now.sec);
  EXPECT_NE(0u, ros_now.nanosec);

  rclcpp::Time test_time(0u, RCL_CLOCK_UNINITIALIZED);
  EXPECT_EQ(0u, test_time.nanoseconds());
  EXPECT_EQ(RCL_CLOCK_UNINITIALIZED, test_time.get_clock_type());

  test_time = ros_now;
  EXPECT_NE(0, test_time.nanoseconds());
  // The clock type is hardcoded internally
  EXPECT_EQ(RCL_ROS_TIME, test_time.get_clock_type());
}

TEST_F(TestTime, test_sum_operator) {
  const rclcpp::Duration one(1);
  const rclcpp::Time test_time(0u);
  EXPECT_EQ(0u, test_time.nanoseconds());

  const rclcpp::Time new_time = one + test_time;
  EXPECT_EQ(1, new_time.nanoseconds());
}

TEST_F(TestTime, test_overflow_underflow_throws) {
  rclcpp::Time test_time(0u);

  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Time(INT64_MAX) + rclcpp::Duration(1),
    std::overflow_error("addition leads to int64_t overflow"));
  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Time(INT64_MIN) + rclcpp::Duration(-1),
    std::underflow_error("addition leads to int64_t underflow"));

  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Time(INT64_MAX) - rclcpp::Duration(-1),
    std::overflow_error("time subtraction leads to int64_t overflow"));
  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Time(INT64_MIN) - rclcpp::Duration(1),
    std::underflow_error("time subtraction leads to int64_t underflow"));

  test_time = rclcpp::Time(INT64_MAX);
  RCLCPP_EXPECT_THROW_EQ(
    test_time += rclcpp::Duration(1),
    std::overflow_error("addition leads to int64_t overflow"));
  test_time = rclcpp::Time(INT64_MIN);
  RCLCPP_EXPECT_THROW_EQ(
    test_time += rclcpp::Duration(-1),
    std::underflow_error("addition leads to int64_t underflow"));

  test_time = rclcpp::Time(INT64_MAX);
  RCLCPP_EXPECT_THROW_EQ(
    test_time -= rclcpp::Duration(-1),
    std::overflow_error("time subtraction leads to int64_t overflow"));
  test_time = rclcpp::Time(INT64_MIN);
  RCLCPP_EXPECT_THROW_EQ(
    test_time -= rclcpp::Duration(1),
    std::underflow_error("time subtraction leads to int64_t underflow"));

  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Duration(INT64_MAX) + rclcpp::Time(1),
    std::overflow_error("addition leads to int64_t overflow"));
  RCLCPP_EXPECT_THROW_EQ(
    test_time = rclcpp::Duration(INT64_MIN) + rclcpp::Time(-1),
    std::underflow_error("addition leads to int64_t underflow"));
}
