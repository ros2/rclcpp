// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>

#include "rcpputils/shared_library.hpp"

#include "rclcpp/typesupport_helpers.hpp"

using namespace ::testing;  // NOLINT

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_no_type) {
  EXPECT_ANY_THROW(
    rclcpp::get_typesupport_library("just_a_package_name", "rosidl_typesupport_cpp"));
}

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_slash_at_the_start_only) {
  EXPECT_ANY_THROW(
    rclcpp::get_typesupport_library("/name_with_slash_at_start", "rosidl_typesupport_cpp"));
}

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_slash_at_the_end_only) {
  EXPECT_ANY_THROW(
    rclcpp::get_typesupport_library("name_with_slash_at_end/", "rosidl_typesupport_cpp"));
}

TEST(TypesupportHelpersTest, throws_exception_if_library_cannot_be_found) {
  EXPECT_THROW(
    rclcpp::get_typesupport_library("invalid/message", "rosidl_typesupport_cpp"),
    std::runtime_error);
}

TEST(TypesupportHelpersTest, returns_c_type_info_for_valid_legacy_library) {
  try {
    auto library = rclcpp::get_typesupport_library(
      "test_msgs/BasicTypes", "rosidl_typesupport_cpp");
    auto string_typesupport = rclcpp::get_message_typesupport_handle(
      "test_msgs/BasicTypes", "rosidl_typesupport_cpp", *library);

    EXPECT_THAT(
      std::string(string_typesupport->typesupport_identifier),
      ContainsRegex("rosidl_typesupport"));
  } catch (const std::exception & e) {
    FAIL() << e.what();
  }
}

TEST(TypesupportHelpersTest, returns_c_type_info_for_valid_library) {
  try {
    auto library = rclcpp::get_typesupport_library(
      "test_msgs/msg/BasicTypes", "rosidl_typesupport_cpp");
    auto string_typesupport = rclcpp::get_message_typesupport_handle(
      "test_msgs/msg/BasicTypes", "rosidl_typesupport_cpp", *library);

    EXPECT_THAT(
      std::string(string_typesupport->typesupport_identifier),
      ContainsRegex("rosidl_typesupport"));
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  }
}

TEST(TypesupportHelpersTest, returns_service_type_info_for_valid_legacy_library) {
  try {
    auto library = rclcpp::get_typesupport_library(
      "test_msgs/Empty", "rosidl_typesupport_cpp");
    auto empty_typesupport = rclcpp::get_service_typesupport_handle(
      "test_msgs/Empty", "rosidl_typesupport_cpp", *library);

    EXPECT_THAT(
      std::string(empty_typesupport->typesupport_identifier),
      ContainsRegex("rosidl_typesupport"));
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  }
}

TEST(TypesupportHelpersTest, returns_service_type_info_for_valid_library) {
  try {
    auto library = rclcpp::get_typesupport_library(
      "test_msgs/srv/Empty", "rosidl_typesupport_cpp");
    auto empty_typesupport = rclcpp::get_service_typesupport_handle(
      "test_msgs/srv/Empty", "rosidl_typesupport_cpp", *library);

    EXPECT_THAT(
      std::string(empty_typesupport->typesupport_identifier),
      ContainsRegex("rosidl_typesupport"));
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  }
}

TEST(TypesupportHelpersTest, test_throw_exception_with_invalid_type) {
  // message
  std::string invalid_type = "test_msgs/msg/InvalidType";
  auto library = rclcpp::get_typesupport_library(invalid_type, "rosidl_typesupport_cpp");
  EXPECT_THROW(
    rclcpp::get_message_typesupport_handle(invalid_type, "rosidl_typesupport_cpp", *library),
    std::runtime_error);
  EXPECT_THROW(
    rclcpp::get_service_typesupport_handle(invalid_type, "rosidl_typesupport_cpp", *library),
    std::runtime_error);

  // service
  invalid_type = "test_msgs/srv/InvalidType";
  library = rclcpp::get_typesupport_library(invalid_type, "rosidl_typesupport_cpp");
  EXPECT_THROW(
    rclcpp::get_service_typesupport_handle(invalid_type, "rosidl_typesupport_cpp", *library),
    std::runtime_error);
}
