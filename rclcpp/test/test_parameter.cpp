// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

class TestParameter : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST(TestParameter, not_set_variant) {
  // Direct instantiation
  rclcpp::parameter::ParameterVariant not_set_variant;
  EXPECT_EQ(not_set_variant.get_type(), rclcpp::parameter::PARAMETER_NOT_SET);
  EXPECT_EQ(not_set_variant.get_type_name(), "not set");

  EXPECT_THROW(not_set_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_int(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_double(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_string(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(not_set_variant.as_string_array(), std::runtime_error);

  rcl_interfaces::msg::Parameter not_set_param = not_set_variant.to_parameter();
  EXPECT_EQ(not_set_param.name, "");
  EXPECT_EQ(not_set_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);

  // From parameter message
  EXPECT_THROW(rclcpp::parameter::ParameterVariant::from_parameter(not_set_param),
    std::runtime_error);
}

TEST(TestParameter, bool_variant) {
  // Direct instantiation
  rclcpp::parameter::ParameterVariant bool_variant_true("bool_param", true);
  EXPECT_EQ(bool_variant_true.get_name(), "bool_param");
  EXPECT_EQ(bool_variant_true.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BOOL);
  EXPECT_EQ(bool_variant_true.get_type_name(), "bool");
  EXPECT_TRUE(bool_variant_true.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL>());
  EXPECT_TRUE(bool_variant_true.get_parameter_value().bool_value);
  EXPECT_EQ(bool_variant_true.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
  EXPECT_TRUE(bool_variant_true.as_bool());

  EXPECT_THROW(bool_variant_true.as_int(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_double(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_string(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_byte_array(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_bool_array(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_integer_array(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_double_array(), std::runtime_error);
  EXPECT_THROW(bool_variant_true.as_string_array(), std::runtime_error);

  EXPECT_EQ(bool_variant_true.value_to_string(), "true");

  rclcpp::parameter::ParameterVariant bool_variant_false("bool_param", false);
  EXPECT_FALSE(bool_variant_false.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL>());
  EXPECT_FALSE(bool_variant_false.get_parameter_value().bool_value);
  EXPECT_EQ(bool_variant_false.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);

  rcl_interfaces::msg::Parameter bool_param = bool_variant_true.to_parameter();
  EXPECT_EQ(bool_param.name, "bool_param");
  EXPECT_EQ(bool_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
  EXPECT_EQ(bool_param.value.bool_value, true);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg_true =
    rclcpp::parameter::ParameterVariant::from_parameter(bool_param);
  EXPECT_EQ(from_msg_true.get_name(), "bool_param");
  EXPECT_EQ(from_msg_true.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BOOL);
  EXPECT_EQ(from_msg_true.get_type_name(), "bool");
  EXPECT_TRUE(from_msg_true.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL>());
  EXPECT_TRUE(from_msg_true.get_parameter_value().bool_value);
  EXPECT_EQ(bool_variant_false.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);

  bool_param.value.bool_value = false;
  rclcpp::parameter::ParameterVariant from_msg_false =
    rclcpp::parameter::ParameterVariant::from_parameter(bool_param);
  EXPECT_FALSE(from_msg_false.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL>());
  EXPECT_FALSE(from_msg_false.get_parameter_value().bool_value);
  EXPECT_EQ(bool_variant_false.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
}

TEST(TestParameter, integer_variant) {
  const int TEST_VALUE {42};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant integer_variant("integer_param", TEST_VALUE);
  EXPECT_EQ(integer_variant.get_name(), "integer_param");
  EXPECT_EQ(integer_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(integer_variant.get_type_name(), "integer");
  EXPECT_EQ(integer_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER>(),
    TEST_VALUE);
  EXPECT_EQ(integer_variant.get_parameter_value().integer_value, TEST_VALUE);
  EXPECT_EQ(integer_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(integer_variant.as_int(), TEST_VALUE);

  EXPECT_THROW(integer_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_double(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_string(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(integer_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(integer_variant.value_to_string(), "42");

  const int64_t TEST_VALUE_L {std::numeric_limits<int64_t>::max()};

  rclcpp::parameter::ParameterVariant long_variant("integer_param", TEST_VALUE_L);
  EXPECT_EQ(long_variant.get_name(), "integer_param");
  EXPECT_EQ(long_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(long_variant.get_type_name(), "integer");
  EXPECT_EQ(long_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER>(),
    TEST_VALUE_L);
  EXPECT_EQ(long_variant.get_parameter_value().integer_value, TEST_VALUE_L);
  EXPECT_EQ(long_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);

  rcl_interfaces::msg::Parameter integer_param = long_variant.to_parameter();
  EXPECT_EQ(integer_param.name, "integer_param");
  EXPECT_EQ(integer_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(integer_param.value.integer_value, TEST_VALUE_L);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(integer_param);
  EXPECT_EQ(from_msg.get_name(), "integer_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(from_msg.get_type_name(), "integer");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER>(),
    TEST_VALUE_L);
  EXPECT_EQ(from_msg.get_parameter_value().integer_value, TEST_VALUE_L);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
}

TEST(TestParameter, double_variant) {
  const double TEST_VALUE {-42.1};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant double_variant("double_param", TEST_VALUE);
  EXPECT_EQ(double_variant.get_name(), "double_param");
  EXPECT_EQ(double_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(double_variant.get_type_name(), "double");
  EXPECT_EQ(double_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE>(),
    TEST_VALUE);
  EXPECT_EQ(double_variant.get_parameter_value().double_value, TEST_VALUE);
  EXPECT_EQ(double_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(double_variant.as_double(), TEST_VALUE);

  EXPECT_THROW(double_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(double_variant.as_int(), std::runtime_error);
  EXPECT_THROW(double_variant.as_string(), std::runtime_error);
  EXPECT_THROW(double_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(double_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(double_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(double_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(double_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(double_variant.value_to_string(), "-42.100000");

  const float TEST_VALUE_F {static_cast<float>(-TEST_VALUE)};
  rclcpp::parameter::ParameterVariant float_variant("float_param", TEST_VALUE_F);
  EXPECT_EQ(float_variant.get_name(), "float_param");
  EXPECT_EQ(float_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(float_variant.get_type_name(), "double");
  EXPECT_EQ(float_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE>(),
    TEST_VALUE_F);
  EXPECT_EQ(float_variant.get_parameter_value().double_value, TEST_VALUE_F);
  EXPECT_EQ(float_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  rcl_interfaces::msg::Parameter double_param = double_variant.to_parameter();
  EXPECT_EQ(double_param.name, "double_param");
  EXPECT_EQ(double_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(double_param.value.double_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(double_param);
  EXPECT_EQ(from_msg.get_name(), "double_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(from_msg.get_type_name(), "double");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE>(), TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().double_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
}

TEST(TestParameter, string_variant) {
  const std::string TEST_VALUE {"ROS2"};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant string_variant("string_param", TEST_VALUE);
  EXPECT_EQ(string_variant.get_name(), "string_param");
  EXPECT_EQ(string_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(string_variant.get_type_name(), "string");
  EXPECT_EQ(string_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_STRING>(),
    TEST_VALUE);
  EXPECT_EQ(string_variant.get_parameter_value().string_value, TEST_VALUE);
  EXPECT_EQ(string_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(string_variant.as_string(), TEST_VALUE);

  EXPECT_THROW(string_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(string_variant.as_int(), std::runtime_error);
  EXPECT_THROW(string_variant.as_double(), std::runtime_error);
  EXPECT_THROW(string_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(string_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(string_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(string_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(string_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(string_variant.value_to_string(), TEST_VALUE);

  rcl_interfaces::msg::Parameter string_param = string_variant.to_parameter();
  EXPECT_EQ(string_param.name, "string_param");
  EXPECT_EQ(string_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(string_param.value.string_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(string_param);
  EXPECT_EQ(from_msg.get_name(), "string_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(from_msg.get_type_name(), "string");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_STRING>(), TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().string_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
}

TEST(TestParameter, byte_array_variant) {
  const std::vector<uint8_t> TEST_VALUE {0x52, 0x4f, 0x53, 0x32};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant byte_array_variant("byte_array_param", TEST_VALUE);
  EXPECT_EQ(byte_array_variant.get_name(), "byte_array_param");
  EXPECT_EQ(byte_array_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BYTE_ARRAY);
  EXPECT_EQ(byte_array_variant.get_type_name(), "byte_array");
  EXPECT_EQ(byte_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_BYTE_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(byte_array_variant.get_parameter_value().byte_array_value, TEST_VALUE);
  EXPECT_EQ(byte_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY);
  EXPECT_EQ(byte_array_variant.as_byte_array(), TEST_VALUE);

  EXPECT_THROW(byte_array_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_int(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_double(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_string(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(byte_array_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(byte_array_variant.value_to_string(), "[0x52, 0x4f, 0x53, 0x32]");

  rcl_interfaces::msg::Parameter byte_array_param = byte_array_variant.to_parameter();
  EXPECT_EQ(byte_array_param.name, "byte_array_param");
  EXPECT_EQ(byte_array_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY);
  EXPECT_EQ(byte_array_param.value.byte_array_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(byte_array_param);
  EXPECT_EQ(from_msg.get_name(), "byte_array_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BYTE_ARRAY);
  EXPECT_EQ(from_msg.get_type_name(), "byte_array");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_BYTE_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().byte_array_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY);
}

TEST(TestParameter, bool_array_variant) {
  const std::vector<bool> TEST_VALUE {false, true, true, false, false, true};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant bool_array_variant("bool_array_param", TEST_VALUE);
  EXPECT_EQ(bool_array_variant.get_name(), "bool_array_param");
  EXPECT_EQ(bool_array_variant.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BOOL_ARRAY);
  EXPECT_EQ(bool_array_variant.get_type_name(), "bool_array");
  EXPECT_EQ(bool_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(bool_array_variant.get_parameter_value().bool_array_value, TEST_VALUE);
  EXPECT_EQ(bool_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY);
  EXPECT_EQ(bool_array_variant.as_bool_array(), TEST_VALUE);

  EXPECT_THROW(bool_array_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_int(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_double(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_string(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(bool_array_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(bool_array_variant.value_to_string(), "[false, true, true, false, false, true]");

  rcl_interfaces::msg::Parameter bool_array_param = bool_array_variant.to_parameter();
  EXPECT_EQ(bool_array_param.name, "bool_array_param");
  EXPECT_EQ(bool_array_param.value.type, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY);
  EXPECT_EQ(bool_array_param.value.bool_array_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(bool_array_param);
  EXPECT_EQ(from_msg.get_name(), "bool_array_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_BOOL_ARRAY);
  EXPECT_EQ(from_msg.get_type_name(), "bool_array");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_BOOL_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().bool_array_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY);
}

TEST(TestParameter, integer_array_variant) {
  const std::vector<int> TEST_VALUE
  {42, -99, std::numeric_limits<int>::max(), std::numeric_limits<int>::lowest(), 0};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant integer_array_variant("integer_array_param", TEST_VALUE);
  EXPECT_EQ(integer_array_variant.get_name(), "integer_array_param");
  EXPECT_EQ(integer_array_variant.get_type(),
    rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(integer_array_variant.get_type_name(), "integer_array");

  // No direct comparison of vectors of ints and long ints
  const auto & param_value_ref =
    integer_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY>();
  const auto mismatches_get_val = std::mismatch(TEST_VALUE.begin(), TEST_VALUE.end(),
      param_value_ref.begin());
  EXPECT_EQ(mismatches_get_val.first, TEST_VALUE.end());
  EXPECT_EQ(mismatches_get_val.second, param_value_ref.end());

  const auto mismatches_get_param = std::mismatch(TEST_VALUE.begin(), TEST_VALUE.end(),
      integer_array_variant.get_parameter_value().integer_array_value.begin());
  EXPECT_EQ(mismatches_get_param.first, TEST_VALUE.end());
  EXPECT_EQ(mismatches_get_param.second,
    integer_array_variant.get_parameter_value().integer_array_value.end());

  EXPECT_EQ(integer_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);

  const std::vector<int64_t> TEST_VALUE_L
  {42, -99, std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::lowest(), 0};

  rclcpp::parameter::ParameterVariant long_array_variant("integer_array_param", TEST_VALUE_L);
  EXPECT_EQ(long_array_variant.get_name(), "integer_array_param");
  EXPECT_EQ(long_array_variant.get_type(),
    rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(long_array_variant.get_type_name(), "integer_array");
  EXPECT_EQ(
    long_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY>(),
    TEST_VALUE_L);
  EXPECT_EQ(long_array_variant.get_parameter_value().integer_array_value, TEST_VALUE_L);
  EXPECT_EQ(long_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(long_array_variant.as_integer_array(), TEST_VALUE_L);

  EXPECT_THROW(long_array_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_int(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_double(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_string(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_double_array(), std::runtime_error);
  EXPECT_THROW(long_array_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(
    long_array_variant.value_to_string(),
    "[42, -99, 9223372036854775807, -9223372036854775808, 0]");

  rcl_interfaces::msg::Parameter integer_array_param = long_array_variant.to_parameter();
  EXPECT_EQ(integer_array_param.name, "integer_array_param");
  EXPECT_EQ(integer_array_param.value.type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(integer_array_param.value.integer_array_value, TEST_VALUE_L);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(integer_array_param);
  EXPECT_EQ(from_msg.get_name(), "integer_array_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(from_msg.get_type_name(), "integer_array");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_INTEGER_ARRAY>(),
    TEST_VALUE_L);
  EXPECT_EQ(from_msg.get_parameter_value().integer_array_value, TEST_VALUE_L);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
}

TEST(TestParameter, double_array_variant) {
  const std::vector<double> TEST_VALUE
  {42.1, -99.1, std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(), 0.1};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant double_array_variant("double_array_param", TEST_VALUE);
  EXPECT_EQ(double_array_variant.get_name(), "double_array_param");
  EXPECT_EQ(double_array_variant.get_type(),
    rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(double_array_variant.get_type_name(), "double_array");
  EXPECT_EQ(
    double_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(double_array_variant.get_parameter_value().double_array_value, TEST_VALUE);
  EXPECT_EQ(double_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(double_array_variant.as_double_array(), TEST_VALUE);

  EXPECT_THROW(double_array_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_int(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_double(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_string(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(double_array_variant.as_string_array(), std::runtime_error);

  EXPECT_EQ(
    double_array_variant.value_to_string(), "[42.1, -99.1, 1.79769e+308, -1.79769e+308, 0.1]");

  const std::vector<float> TEST_VALUE_F
  {42.1f, -99.1f, std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(), 0.1f};

  rclcpp::parameter::ParameterVariant float_array_variant("double_array_param", TEST_VALUE_F);
  EXPECT_EQ(float_array_variant.get_name(), "double_array_param");
  EXPECT_EQ(float_array_variant.get_type(),
    rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(float_array_variant.get_type_name(), "double_array");

  // No direct comparison of vectors of doubles and floats
  const auto & param_value_ref =
    float_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY>();
  const auto mismatches_get_val = std::mismatch(TEST_VALUE_F.begin(), TEST_VALUE_F.end(),
      param_value_ref.begin());
  EXPECT_EQ(mismatches_get_val.first, TEST_VALUE_F.end());
  EXPECT_EQ(mismatches_get_val.second, param_value_ref.end());

  const auto mismatches_get_param = std::mismatch(TEST_VALUE_F.begin(), TEST_VALUE_F.end(),
      float_array_variant.get_parameter_value().double_array_value.begin());
  EXPECT_EQ(mismatches_get_param.first, TEST_VALUE_F.end());
  EXPECT_EQ(mismatches_get_param.second,
    double_array_variant.get_parameter_value().double_array_value.end());

  EXPECT_EQ(float_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);

  rcl_interfaces::msg::Parameter double_array_param = double_array_variant.to_parameter();
  EXPECT_EQ(double_array_param.name, "double_array_param");
  EXPECT_EQ(double_array_param.value.type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(double_array_param.value.double_array_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(double_array_param);
  EXPECT_EQ(from_msg.get_name(), "double_array_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(from_msg.get_type_name(), "double_array");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_DOUBLE_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().double_array_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
}

TEST(TestParameter, string_array_variant) {
  const std::vector<std::string> TEST_VALUE {"R", "O", "S2"};

  // Direct instantiation
  rclcpp::parameter::ParameterVariant string_array_variant("string_array_param", TEST_VALUE);
  EXPECT_EQ(string_array_variant.get_name(), "string_array_param");
  EXPECT_EQ(string_array_variant.get_type(),
    rclcpp::parameter::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(string_array_variant.get_type_name(), "string_array");
  EXPECT_EQ(
    string_array_variant.get_value<rclcpp::parameter::ParameterType::PARAMETER_STRING_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(string_array_variant.get_parameter_value().string_array_value, TEST_VALUE);
  EXPECT_EQ(string_array_variant.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(string_array_variant.as_string_array(), TEST_VALUE);

  EXPECT_THROW(string_array_variant.as_bool(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_int(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_double(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_string(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_byte_array(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_bool_array(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_integer_array(), std::runtime_error);
  EXPECT_THROW(string_array_variant.as_double_array(), std::runtime_error);

  EXPECT_EQ(string_array_variant.value_to_string(), "[R, O, S2]");

  rcl_interfaces::msg::Parameter string_array_param = string_array_variant.to_parameter();
  EXPECT_EQ(string_array_param.name, "string_array_param");
  EXPECT_EQ(string_array_param.value.type,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(string_array_param.value.string_array_value, TEST_VALUE);

  // From parameter message
  rclcpp::parameter::ParameterVariant from_msg =
    rclcpp::parameter::ParameterVariant::from_parameter(string_array_param);
  EXPECT_EQ(from_msg.get_name(), "string_array_param");
  EXPECT_EQ(from_msg.get_type(), rclcpp::parameter::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(from_msg.get_type_name(), "string_array");
  EXPECT_EQ(from_msg.get_value<rclcpp::parameter::ParameterType::PARAMETER_STRING_ARRAY>(),
    TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().string_array_value, TEST_VALUE);
  EXPECT_EQ(from_msg.get_parameter_value().type,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
}
