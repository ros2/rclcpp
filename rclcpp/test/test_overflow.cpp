// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <limits>
#include <string>
#include <tuple>  // JUST FOR CPPLINT

using testing::TestWithParam;
using testing::ValuesIn;
using testing::Combine;
using testing::tuple;
using testing::get;

#define RCLCPP__CUSTOM_OVERFLOW
#include "rclcpp/overflow.hpp"

#ifdef RCLCPP__BUILTIN_OVERFLOW

const int signed_values[] = {
  std::numeric_limits<int>::min(),
  std::numeric_limits<int>::min() / 2,
  -2,
  -1,
  0,
  1,
  2,
  std::numeric_limits<int>::max() / 2,
  std::numeric_limits<int>::max()
};

struct __fixture_signed_int : public TestWithParam<tuple<int, int>>
{
  __fixture_signed_int()
  : param_(GetParam()) {}
  using p_signed_int_t = tuple<int, int>;
  p_signed_int_t param_;
};

using add_signed_int = __fixture_signed_int;

INSTANTIATE_TEST_CASE_P(/**/, add_signed_int,
  Combine(ValuesIn(signed_values), ValuesIn(signed_values)), );

TEST_P(add_signed_int, generic)
{
  int res = 0, com = 0;
  EXPECT_EQ(custom_add_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_add_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_add_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

using sub_signed_int = __fixture_signed_int;

INSTANTIATE_TEST_CASE_P(/**/, sub_signed_int,
  Combine(ValuesIn(signed_values), ValuesIn(signed_values)), );

TEST_P(sub_signed_int, generic)
{
  int res = 0, com = 0;
  EXPECT_EQ(custom_sub_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_sub_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_sub_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

using mul_signed_int = __fixture_signed_int;

INSTANTIATE_TEST_CASE_P(/**/, mul_signed_int,
  Combine(ValuesIn(signed_values), ValuesIn(signed_values)), );

TEST_P(mul_signed_int, generic)
{
  int res = 0, com = 0;
  EXPECT_EQ(custom_mul_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_mul_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_mul_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

const unsigned int unsigned_values[] = {
  std::numeric_limits<unsigned int>::min(),
  0,
  1,
  2,
  std::numeric_limits<unsigned int>::max()
};

struct __fixture_unsigned_int : public TestWithParam<tuple<unsigned int, unsigned int>>
{
  __fixture_unsigned_int()
  : param_(GetParam()) {}
  using p_unsigned_int_t = tuple<unsigned int, unsigned int>;
  p_unsigned_int_t param_;
};

using add_unsigned_int = __fixture_unsigned_int;

INSTANTIATE_TEST_CASE_P(/**/, add_unsigned_int,
  Combine(ValuesIn(unsigned_values), ValuesIn(unsigned_values)), );

TEST_P(add_unsigned_int, generic)
{
  unsigned int res = 0, com = 0;
  EXPECT_EQ(custom_add_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_add_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_add_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

using sub_unsigned_int = __fixture_unsigned_int;

INSTANTIATE_TEST_CASE_P(/**/, sub_unsigned_int,
  Combine(ValuesIn(unsigned_values), ValuesIn(unsigned_values)), );

TEST_P(sub_unsigned_int, generic)
{
  unsigned int res = 0, com = 0;
  EXPECT_EQ(custom_sub_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_sub_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_sub_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

using mul_unsigned_int = __fixture_unsigned_int;

INSTANTIATE_TEST_CASE_P(/**/, mul_unsigned_int,
  Combine(ValuesIn(unsigned_values), ValuesIn(unsigned_values)), );

TEST_P(mul_unsigned_int, generic)
{
  unsigned int res = 0, com = 0;
  EXPECT_EQ(custom_mul_overflow(get<0>(param_), get<1>(param_), &res),
    __builtin_mul_overflow(get<0>(param_), get<1>(param_), &com));
  if (!__builtin_mul_overflow(get<0>(param_), get<1>(param_), &com)) {
    EXPECT_EQ(res, com);
  }
}

#endif  // RCLCPP__BUILTIN_OVERFLOW

TEST(rclcpp_overflow, signed_test)
{
  {
    SCOPED_TRACE("rclcpp_add_overflow failed with signed");
    int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_add_overflow(1, 1, &res));
    EXPECT_FALSE(__rclcpp_add_overflow(std::numeric_limits<int>::max(), -1, &res));
    EXPECT_FALSE(__rclcpp_add_overflow(std::numeric_limits<int>::min(), 1, &res));
    EXPECT_TRUE(__rclcpp_add_overflow(std::numeric_limits<int>::max(), 1, &res));
    EXPECT_TRUE(__rclcpp_add_overflow(std::numeric_limits<int>::min(), -1, &res));
  }

  {
    SCOPED_TRACE("rclcpp_sub_overflow failed with signed");
    int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_sub_overflow(1, 1, &res));
    EXPECT_FALSE(__rclcpp_sub_overflow(std::numeric_limits<int>::max(), 1, &res));
    EXPECT_FALSE(__rclcpp_sub_overflow(std::numeric_limits<int>::min(), -1, &res));
    EXPECT_TRUE(__rclcpp_sub_overflow(std::numeric_limits<int>::max(), -1, &res));
    EXPECT_TRUE(__rclcpp_sub_overflow(std::numeric_limits<int>::min(), 1, &res));
  }

  {
    SCOPED_TRACE("rclcpp_mul_overflow failed with signed");
    int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_mul_overflow(1, 1, &res));
    EXPECT_FALSE(__rclcpp_mul_overflow(std::numeric_limits<int>::max(), 1, &res));
    EXPECT_FALSE(__rclcpp_mul_overflow(std::numeric_limits<int>::min(), 1, &res));
    EXPECT_FALSE(__rclcpp_mul_overflow(std::numeric_limits<int>::max(), -1, &res));
    EXPECT_TRUE(__rclcpp_mul_overflow(std::numeric_limits<int>::min(), -1, &res));
    EXPECT_TRUE(__rclcpp_mul_overflow(std::numeric_limits<int>::max(), 2, &res));
  }
}

TEST(rclcpp_overflow, unsigned_test)
{
  {
    SCOPED_TRACE("rclcpp_add_overflow failed with unsigned");
    unsigned int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_add_overflow(1u, 1u, &res));
    EXPECT_FALSE(__rclcpp_add_overflow(std::numeric_limits<unsigned int>::min(), 1u, &res));
    EXPECT_TRUE(__rclcpp_add_overflow(std::numeric_limits<unsigned int>::max(), 1u, &res));
  }

  {
    SCOPED_TRACE("rclcpp_sub_overflow failed with unsigned");
    unsigned int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_sub_overflow(1u, 1u, &res));
    EXPECT_FALSE(__rclcpp_sub_overflow(std::numeric_limits<unsigned int>::max(), 1u, &res));
    EXPECT_TRUE(__rclcpp_sub_overflow(std::numeric_limits<unsigned int>::min(), 1u, &res));
  }

  {
    SCOPED_TRACE("rclcpp_mul_overflow failed with unsigned");
    unsigned int res = 0;
    // check the rclcpp_add_overflow function
    EXPECT_FALSE(__rclcpp_mul_overflow(1u, 1u, &res));
    EXPECT_FALSE(__rclcpp_mul_overflow(std::numeric_limits<unsigned int>::max(), 1u, &res));
    EXPECT_FALSE(__rclcpp_mul_overflow(std::numeric_limits<unsigned int>::min(), 1u, &res));
    EXPECT_TRUE(__rclcpp_mul_overflow(std::numeric_limits<unsigned int>::max(), 2u, &res));
  }
}

TEST(rclcpp_overflow, exhaustive_uint8)
{
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
      test_type_t res = 0;

      {
        SCOPED_TRACE("failed with " + std::to_string(x) + " + " + std::to_string(y));
        const big_type_t sum = x + y;

        const bool expected = sum > max_val || sum < min_val;
        EXPECT_EQ(expected, __rclcpp_add_overflow(static_cast<test_type_t>(x),
          static_cast<test_type_t>(y),
          &res));
        if (!expected) {
          EXPECT_EQ(sum, res);
        }
      }

      {
        SCOPED_TRACE("failed with " + std::to_string(x) + " - " + std::to_string(y));
        const big_type_t diff = x - y;

        const bool expected = diff > max_val || diff < min_val;
        EXPECT_EQ(expected, __rclcpp_sub_overflow(static_cast<test_type_t>(x),
          static_cast<test_type_t>(y),
          &res));
        if (!expected) {
          EXPECT_EQ(diff, res);
        }
      }

      {
        SCOPED_TRACE("failed with " + std::to_string(x) + " * " + std::to_string(y));
        const big_type_t prod = x * y;

        const bool expected = prod > max_val || prod < min_val;
        EXPECT_EQ(expected, __rclcpp_mul_overflow(static_cast<test_type_t>(x),
          static_cast<test_type_t>(y),
          &res));
        if (!expected) {
          EXPECT_EQ(prod, res);
        }
      }
    }
  }
}
