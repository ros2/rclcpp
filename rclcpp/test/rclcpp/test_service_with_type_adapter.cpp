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

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "rclcpp/msg/empty.hpp"
#include "rclcpp/msg/string.hpp"
#include "rclcpp/msg/bool.hpp"

#include "rclcpp/srv/set_bool.hpp"

using namespace std::chrono_literals;

class TestService : public ::testing::Test
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

struct CustomBool
{
  struct SetBoolResponse
  {
    bool success;
    std::string message;
  };

  using Request = bool;
  using Response = SetBoolResponse;
};

struct CustomIncorrectBool
{
  struct SetBoolResponse
  {
    int success;
    std::string message;
  };

  using Request = bool;
  using Response = SetBoolResponse;
};

template<>
struct rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>
{
  using is_specialized = std::true_type;
  using custom_type = CustomBool;
  using ros_message_type = rclcpp::srv::SetBool;

  static void
  convert_to_ros_service_request(
    const custom_type::Request & source,
    ros_message_type::Request & destination)
  {
    destination.data = source;
  }

  static void
  convert_to_custom_service_request(
    const ros_message_type::Request & source,
    custom_type::Request & destination)
  {
    destination = source.data;
  }

  static void
  convert_to_ros_service_response(
    const custom_type::Response & source,
    ros_message_type::Response & destination)
  {
    destination.success = source.success;
    destination.message = source.message;
  }

  static void
  convert_to_custom_service_response(
    const ros_message_type::Response & source,
    custom_type::Response & destination)
  {
    destination.success = source.success;
    destination.message = source.message;
  }
};

// Throws in conversion
template<>
struct rclcpp::TypeAdapter<CustomIncorrectBool, rclcpp::srv::SetBool>
{
  using is_specialized = std::true_type;
  using custom_type = CustomIncorrectBool;
  using ros_message_type = rclcpp::srv::SetBool;

  static void
  convert_to_ros_service_request(
    const custom_type::Request & source,
    ros_message_type::Request & destination)
  {
    (void) source;
    (void) destination;
    throw std::runtime_error("This should not happen");
  }

  static void
  convert_to_custom_service_request(
    const ros_message_type::Request & source,
    custom_type::Request & destination)
  {
    (void) source;
    (void) destination;
  }

  static void
  convert_to_ros_service_response(
    const custom_type::Response & source,
    ros_message_type::Response & destination)
  {
    (void) source;
    (void) destination;
    throw std::runtime_error("This should not happen");
  }

  static void
  convert_to_custom_service_response(
    const ros_message_type::Response & source,
    custom_type::Response & destination)
  {
    (void) source;
    (void) destination;
  }
};

void custom_total_callback(
  const std::shared_ptr<CustomBool::Request> req,
  const std::shared_ptr<CustomBool::Response> res)
{
  (void)req;
  (void)res;
}

void custom_ros_callback(
  const std::shared_ptr<CustomBool::Request> req,
  const std::shared_ptr<rclcpp::srv::SetBool::Response> res)
{
  (void)req;
  (void)res;
}

void ros_custom_callback(
  const std::shared_ptr<rclcpp::srv::SetBool::Request> req,
  const std::shared_ptr<CustomBool::Response> res)
{
  (void)req;
  (void)res;
}

void incorrect_callback(
  const std::shared_ptr<rclcpp::srv::SetBool::Request> req,
  const std::shared_ptr<rclcpp::srv::SetBool::Response> res)
{
  (void)req;
  (void)res;
}

/*
 * Testing the basic creation of services with a TypeAdapter for both Request and Response
 */
TEST_F(TestService, various_creation_signatures)
{
  {
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

    using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;
    auto service = node->create_service<AdaptedTypeStruct>("service", &custom_total_callback);
    (void)service;
  }
  {
    /// Now try to adapt the type with the `as` metafunction
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

    using AdaptedTypeStruct = rclcpp::adapt_type<CustomBool>::as<rclcpp::srv::SetBool>;
    auto service = node->create_service<AdaptedTypeStruct>("service", &custom_total_callback);
    (void)service;
  }
  {
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

    using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;
    auto service = node->create_service<AdaptedTypeStruct>("service", &custom_ros_callback);
    (void)service;
  }
  {
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

    using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;
    auto service = node->create_service<AdaptedTypeStruct>("service", &ros_custom_callback);
    (void)service;
  }
}

/// Testing that conversion errors are passed up
TEST_F(TestService, conversion_exception_is_passed_up)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using BadAdaptedTypeStruct = rclcpp::TypeAdapter<CustomIncorrectBool, rclcpp::srv::SetBool>;

  auto service = node->create_service<BadAdaptedTypeStruct>("service", &incorrect_callback);
}

TEST_F(TestService, send_adapted_response) {
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");
  using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;

  auto server = node->create_service<AdaptedTypeStruct>("service", &custom_total_callback);
  {
    auto request_id = server->create_request_header();
    CustomBool::Response response;
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_OK);
    EXPECT_NO_THROW(server->send_response(*request_id.get(), response));
  }
  {
    auto request_id = server->create_request_header();
    CustomBool::Response response;
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_ERROR);
    EXPECT_THROW(
      server->send_response(*request_id.get(), response),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestService, take_adapted_request) {
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");
  using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;

  auto server = node->create_service<AdaptedTypeStruct>("service", &custom_total_callback);
  {
    auto request_id = server->create_request_header();
    CustomBool::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_OK);
    EXPECT_TRUE(server->take_request(request, *request_id.get()));
  }
  {
    auto request_id = server->create_request_header();
    CustomBool::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_SERVICE_TAKE_FAILED);
    EXPECT_FALSE(server->take_request(request, *request_id.get()));
  }
  {
    auto request_id = server->create_request_header();
    CustomBool::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_ERROR);
    EXPECT_THROW(server->take_request(request, *request_id.get()), rclcpp::exceptions::RCLError);
  }
}
