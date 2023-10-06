// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <stdexcept>
#include <string>

#include "rclcpp/rate.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

class TestRate : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

/*
   Basic tests for the Rate and WallRate classes.
 */
TEST_F(TestRate, rate_basics) {
  auto period = std::chrono::milliseconds(1000);
  auto offset = std::chrono::milliseconds(500);
  auto epsilon = std::chrono::milliseconds(100);
  double overrun_ratio = 1.5;

  auto start = std::chrono::system_clock::now();
  rclcpp::Rate r(period);
  EXPECT_EQ(rclcpp::Duration(period), r.period());
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  ASSERT_FALSE(r.is_steady());
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
  ASSERT_EQ(RCL_SYSTEM_TIME, r.get_type());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::system_clock::now();
  auto delta = one - start;
  EXPECT_LT(period, delta + epsilon);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::system_clock::now();
  delta = two - start;
  EXPECT_LT(2 * period, delta);
  EXPECT_GT(2 * period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::system_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::system_clock::now();
  delta = three - two_offset;
  EXPECT_LT(period, delta + epsilon);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset + period);
  auto four = std::chrono::system_clock::now();
  ASSERT_FALSE(r.sleep());
  auto five = std::chrono::system_clock::now();
  delta = five - four;
  ASSERT_TRUE(epsilon > delta);
}

TEST_F(TestRate, wall_rate_basics) {
  auto period = std::chrono::milliseconds(100);
  auto offset = std::chrono::milliseconds(50);
  auto epsilon = std::chrono::milliseconds(1);
  double overrun_ratio = 1.5;

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate r(period);
  EXPECT_EQ(rclcpp::Duration(period), r.period());
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  ASSERT_TRUE(r.is_steady());
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  ASSERT_EQ(RCL_STEADY_TIME, r.get_type());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::steady_clock::now();
  auto delta = one - start;
  EXPECT_LT(period, delta);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::steady_clock::now();
  delta = two - start;
  EXPECT_LT(2 * period, delta + epsilon);
  EXPECT_GT(2 * period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::steady_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::steady_clock::now();
  delta = three - two_offset;
  EXPECT_LT(period, delta);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset + period);
  auto four = std::chrono::steady_clock::now();
  ASSERT_FALSE(r.sleep());
  auto five = std::chrono::steady_clock::now();
  delta = five - four;
  EXPECT_GT(epsilon, delta);
}

/*
   Basic test for the deprecated GenericRate class.
 */
TEST_F(TestRate, generic_rate) {
  auto period = std::chrono::milliseconds(100);
  auto offset = std::chrono::milliseconds(50);
  auto epsilon = std::chrono::milliseconds(1);
  double overrun_ratio = 1.5;

  {
    auto start = std::chrono::system_clock::now();

// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

    rclcpp::GenericRate<std::chrono::system_clock> gr(period);
    ASSERT_FALSE(gr.is_steady());

// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

    ASSERT_EQ(gr.get_type(), RCL_SYSTEM_TIME);
    EXPECT_EQ(period, gr.period());
    ASSERT_TRUE(gr.sleep());
    auto one = std::chrono::system_clock::now();
    auto delta = one - start;
    EXPECT_LT(period, delta + epsilon);
    EXPECT_GT(period * overrun_ratio, delta);

    rclcpp::sleep_for(offset);
    ASSERT_TRUE(gr.sleep());
    auto two = std::chrono::system_clock::now();
    delta = two - start;
    EXPECT_LT(2 * period, delta);
    EXPECT_GT(2 * period * overrun_ratio, delta);

    rclcpp::sleep_for(offset);
    auto two_offset = std::chrono::system_clock::now();
    gr.reset();
    ASSERT_TRUE(gr.sleep());
    auto three = std::chrono::system_clock::now();
    delta = three - two_offset;
    EXPECT_LT(period, delta + epsilon);
    EXPECT_GT(period * overrun_ratio, delta);

    rclcpp::sleep_for(offset + period);
    auto four = std::chrono::system_clock::now();
    ASSERT_FALSE(gr.sleep());
    auto five = std::chrono::system_clock::now();
    delta = five - four;
    ASSERT_TRUE(epsilon > delta);
  }

  {
    auto start = std::chrono::steady_clock::now();

// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

    rclcpp::GenericRate<std::chrono::steady_clock> gr(period);
    ASSERT_TRUE(gr.is_steady());

// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

    ASSERT_EQ(gr.get_type(), RCL_STEADY_TIME);
    EXPECT_EQ(period, gr.period());
    ASSERT_TRUE(gr.sleep());
    auto one = std::chrono::steady_clock::now();
    auto delta = one - start;
    EXPECT_LT(period, delta);
    EXPECT_GT(period * overrun_ratio, delta);

    rclcpp::sleep_for(offset);
    ASSERT_TRUE(gr.sleep());
    auto two = std::chrono::steady_clock::now();
    delta = two - start;
    EXPECT_LT(2 * period, delta + epsilon);
    EXPECT_GT(2 * period * overrun_ratio, delta);

    rclcpp::sleep_for(offset);
    auto two_offset = std::chrono::steady_clock::now();
    gr.reset();
    ASSERT_TRUE(gr.sleep());
    auto three = std::chrono::steady_clock::now();
    delta = three - two_offset;
    EXPECT_LT(period, delta);
    EXPECT_GT(period * overrun_ratio, delta);

    rclcpp::sleep_for(offset + period);
    auto four = std::chrono::steady_clock::now();
    ASSERT_FALSE(gr.sleep());
    auto five = std::chrono::steady_clock::now();
    delta = five - four;
    EXPECT_GT(epsilon, delta);
  }
}

TEST_F(TestRate, from_double) {
  {
    rclcpp::Rate rate(1.0);
    EXPECT_EQ(rclcpp::Duration(std::chrono::seconds(1)), rate.period());
  }
  {
    rclcpp::Rate rate(2.0);
    EXPECT_EQ(rclcpp::Duration(std::chrono::milliseconds(500)), rate.period());
  }
  {
    rclcpp::WallRate rate(0.5);
    EXPECT_EQ(rclcpp::Duration(std::chrono::seconds(2)), rate.period());
  }
  {
    rclcpp::WallRate rate(4.0);
    EXPECT_EQ(rclcpp::Duration(std::chrono::milliseconds(250)), rate.period());
  }
}

TEST_F(TestRate, clock_types) {
  {
    rclcpp::Rate rate(1.0, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    EXPECT_FALSE(rate.is_steady());
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
    EXPECT_EQ(RCL_SYSTEM_TIME, rate.get_type());
  }
  {
    rclcpp::Rate rate(1.0, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    EXPECT_TRUE(rate.is_steady());
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
    EXPECT_EQ(RCL_STEADY_TIME, rate.get_type());
  }
  {
    rclcpp::Rate rate(1.0, std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
// suppress deprecated warnings
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    EXPECT_FALSE(rate.is_steady());
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
    EXPECT_EQ(RCL_ROS_TIME, rate.get_type());
  }
}

TEST_F(TestRate, incorrect_constuctor) {
  // Constructor with 0-frequency
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::Rate rate(0.0),
    std::invalid_argument("rate must be greater than 0"));

  // Constructor with negative frequency
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::Rate rate(-1.0),
    std::invalid_argument("rate must be greater than 0"));

  // Constructor with 0-duration
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::Rate rate(rclcpp::Duration(0, 0)),
    std::invalid_argument("period must be greater than 0"));

  // Constructor with negative duration
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::Rate rate(rclcpp::Duration(-1, 0)),
    std::invalid_argument("period must be greater than 0"));
}
