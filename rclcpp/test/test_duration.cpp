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

#include <chrono>
#include <limits>

#include "rcl/time.h"
#include "rclcpp/duration.hpp"
#include "builtin_interfaces/msg/duration.hpp"


using namespace std::chrono_literals;

class TestDuration : public ::testing::Test
{
};

TEST(TestDuration, conversions) {
  // pull up the message type
  using msg_t = builtin_interfaces::msg::Duration;
  using sec_t = typename msg_t::_sec_type;
  using nano_t = typename msg_t::_nanosec_type;
  using full_t = rcl_duration_value_t;

  {
    SCOPED_TRACE("test from builtin_interface");
    // setup a ill-formed message
    msg_t ill_formed;
    ill_formed.sec = std::numeric_limits<sec_t>::max();
    ill_formed.nanosec = std::numeric_limits<nano_t>::max();

    // create a rclcpp::Duration, convert it twice - first round from an ill
    // defined input and second from properly controller input.
    rclcpp::Duration duration1 = ill_formed;
    msg_t msg1 = duration1;
    rclcpp::Duration duration2 = msg1;
    msg_t msg2 = duration2;

    // check that the conversion bears no side effects
    EXPECT_EQ(duration1, duration2);
    EXPECT_EQ(msg1, msg2);

    // check that the conversion is correct
    const auto raw_nano = RCL_S_TO_NS(static_cast<full_t>(msg1.sec)) +
      static_cast<full_t>(msg1.nanosec);
    EXPECT_EQ(raw_nano, duration1.nanoseconds());
  }

  {
    SCOPED_TRACE("test from rclcpp::Duration");

    // setup a rclcpp::Duration, which receives ill input (too much nano)
    // and convert it back to message and back to rclcpp::Duration;
    // The conversion shall have no side effects
    rclcpp::Duration duration1(std::numeric_limits<sec_t>::max(),
      std::numeric_limits<nano_t>::max());
    msg_t msg1 = duration1;
    rclcpp::Duration duration2 = msg1;
    msg_t msg2 = duration2;

    // check that the conversion bears no side effects
    EXPECT_EQ(duration1, duration2);
    EXPECT_EQ(msg1, msg2);

    // check that the conversion is correct
    const auto raw_nano = RCL_S_TO_NS(static_cast<full_t>(msg1.sec)) +
      static_cast<full_t>(msg1.nanosec);
    EXPECT_EQ(raw_nano, duration1.nanoseconds());
  }

  {
    SCOPED_TRACE("test with negative numbers");
    // setup a ill-formed message
    msg_t ill_formed;
    ill_formed.sec = std::numeric_limits<sec_t>::min();
    ill_formed.nanosec = std::numeric_limits<nano_t>::max();

    // create a rclcpp::Duration, convert it twice - first round from an ill
    // defined input and second from properly controller input.
    rclcpp::Duration duration1 = ill_formed;
    msg_t msg1 = duration1;
    rclcpp::Duration duration2 = msg1;
    msg_t msg2 = duration2;

    // check that the conversion bears no side effects
    EXPECT_EQ(duration1, duration2);
    EXPECT_EQ(msg1, msg2);
  }
}

TEST(TestDuration, operators) {
  rclcpp::Duration old(1, 0);
  rclcpp::Duration young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);

  rclcpp::Duration add = old + young;
  EXPECT_EQ(add.nanoseconds(), (rcl_duration_value_t)(old.nanoseconds() + young.nanoseconds()));
  EXPECT_EQ(add, old + young);

  rclcpp::Duration sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), (rcl_duration_value_t)(young.nanoseconds() - old.nanoseconds()));
  EXPECT_EQ(sub, young - old);

  rclcpp::Duration scale = old * 3;
  EXPECT_EQ(scale.nanoseconds(), (rcl_duration_value_t)(old.nanoseconds() * 3));

  rclcpp::Duration time = rclcpp::Duration(0, 0);
  rclcpp::Duration copy_constructor_duration(time);
  rclcpp::Duration assignment_op_duration = rclcpp::Duration(1, 0);
  assignment_op_duration = time;

  EXPECT_TRUE(time == copy_constructor_duration);
  EXPECT_TRUE(time == assignment_op_duration);
}

TEST(TestDuration, chrono_overloads) {
  int64_t ns = 123456789l;
  auto chrono_ns = std::chrono::nanoseconds(ns);
  auto d1 = rclcpp::Duration(ns);
  auto d2 = rclcpp::Duration(chrono_ns);
  auto d3 = rclcpp::Duration(123456789ns);
  EXPECT_EQ(d1, d2);
  EXPECT_EQ(d1, d3);
  EXPECT_EQ(d2, d3);
}

TEST(TestDuration, overflows) {
  rclcpp::Duration max_(std::numeric_limits<rcl_duration_value_t>::max());
  rclcpp::Duration min_(std::numeric_limits<rcl_duration_value_t>::min());

  rclcpp::Duration one(1);
  rclcpp::Duration negative_one(-1);

  EXPECT_THROW(max_ + one, std::overflow_error);
  EXPECT_THROW(min_ - one, std::underflow_error);
  EXPECT_THROW(negative_one + min_, std::underflow_error);
  // the min is defined as -max - 1, hence the expression -1 -max is not
  // sufficient to provoke and underflow
  EXPECT_THROW((negative_one + negative_one) - max_, std::underflow_error);

  rclcpp::Duration base_d = max_ * 0.3;
  EXPECT_THROW(base_d * 4, std::overflow_error);
  EXPECT_THROW(base_d * (-4), std::underflow_error);

  rclcpp::Duration base_d_neg = max_ * (-0.3);
  EXPECT_THROW(base_d_neg * (-4), std::overflow_error);
  EXPECT_THROW(base_d_neg * 4, std::underflow_error);
}

TEST(TestDuration, negative_duration) {
  rclcpp::Duration assignable_duration = rclcpp::Duration(0) - rclcpp::Duration(5, 0);

  {
    // avoid windows converting a literal number less than -INT_MAX to unsigned int C4146
    int64_t expected_value = -5000;
    expected_value *= 1000 * 1000;
    EXPECT_EQ(expected_value, assignable_duration.nanoseconds());
  }

  {
    builtin_interfaces::msg::Duration duration_msg;
    duration_msg.sec = -4;
    duration_msg.nanosec = 250000000;

    assignable_duration = duration_msg;
    // avoid windows converting a literal number less than -INT_MAX to unsigned int C4146
    int64_t expected_value = -4250;
    expected_value *= 1000 * 1000;
    EXPECT_EQ(expected_value, assignable_duration.nanoseconds());
  }
}

TEST(TestDuration, maximum_duration) {
  rclcpp::Duration max_duration = rclcpp::Duration::max();
  rclcpp::Duration max_(std::numeric_limits<int32_t>::max(), 999999999);

  EXPECT_EQ(max_duration, max_);
}
