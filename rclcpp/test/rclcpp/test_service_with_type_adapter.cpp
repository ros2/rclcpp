// Copyright 2023 Open Source Robotics Foundation, Inc.
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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

#include "rclcpp/msg/empty.hpp"
#include "rclcpp/msg/string.hpp"
#include "rclcpp/msg/bool.hpp"

#include "rclcpp/srv/set_bool.hpp"

using namespace std::chrono_literals;

class TestService: public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

template<>
struct rclcpp::TypeAdapter<bool, rclcpp::msg::Bool>
{
  using is_specialized = std::true_type;
  using custom_type = bool;
  using ros_message_type = rclcpp::msg::Bool;

  static void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

template<>
struct rclcpp::TypeAdapter<std::string, rclcpp::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = rclcpp::msg::String;

  static void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

// Throws in conversion
template<>
struct rclcpp::TypeAdapter<int, rclcpp::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = int;
  using ros_message_type = rclcpp::msg::String;

  static void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    (void) source;
    (void) destination;
    throw std::runtime_error("This should not happen");
  }

  static void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    (void) source;
    (void) destination;
  }
};

/*
 * Testing the basic creation of services with a TypeAdapter for both Request and Response
 */
TEST_F(TestService, total_type_adaption_service_creation)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using AdaptedRequestType = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>>;
  using AdaptedResponseType = rclcpp::TypeAdapter<bool, rclcpp::msg::Bool>>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = AdaptedResponseType;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const std::string & req, const bool & res) {});

  /// Now try to adapt the type with the `as` metafunction
  (void)service;

  using AdaptedRequestType = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;
  using AdaptedResponseType = rclcpp::adapt_type<bool>::as<rclcpp::msg::Bool>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = AdaptedResponseType;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const std::string & req, const bool & res) {});
  (void)service;
}

TEST_F(TestService, request_type_adaption_service_creation)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using AdaptedRequestType = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = rclcpp::srv::SetBool::Response;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const std::string & req, rclcpp::srv::SetBool::Response & res) {});

  /// Now try to adapt the type with the `as` metafunction
  (void)service;

  using AdaptedRequestType = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = rclcpp::srv::SetBool::Response;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const std::string & req, rclcpp::srv::SetBool::Response & res) {});
  (void)service;
}

TEST_F(TestService, response_type_adaption_service_creation)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using AdaptedResponseType = rclcpp::TypeAdapter<bool, rclcpp::msg::Bool>>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = AdaptedResponseType;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const rclcpp::msg::String & req, const bool & res) {});

  /// Now try to adapt the type with the `as` metafunction
  (void)service;

  using AdaptedResponseType = rclcpp::adapt_type<bool>::as<rclcpp::msg::Bool>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = AdaptedResponseType;
  };

  auto service = rclcpp::create_service<AdaptedTypeStruct>(
      "service",
      [](const rclcpp::msg::String & req, const bool & res) {});
  (void)service;
}

/// Testing that conversion errors are passed up
TEST_F(TestService, conversion_exception_is_passed_up)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using BadAdaptedResponseType = rclcpp::TypeAdapter<int, rclcpp::msg::String>>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = BadAdaptedResponseType;
  };

  auto service = rclcpp::create_service<BadAdaptedTypeStruct>(
      "service",
      [](const rclcpp::msg::String & req) {});
}
