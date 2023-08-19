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

#include "test_msgs/msg/empty.hpp"
#include "rclcpp/msg/string.hpp"
#include "rclcpp/msg/bool.hpp"

#include "rclcpp/srv/set_bool.hpp"

using namespace std::chrono_literals;

static const int g_max_loops = 200;
static const std::chrono::milliseconds g_sleep_per_loop(10);


class TestClient: public ::testing::Test
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

namespace rclcpp
{
template<>
struct TypeAdapter<bool, rclcpp::msg::Bool>
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
struct TypeAdapter<std::string, rclcpp::msg::String>
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
struct TypeAdapter<int, rclcpp::msg::String>
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

}  // namespace rclcpp

/*
 * Testing the basic creation of clients with a TypeAdapter for both Request and Response
 */
TEST_F(TestClient, total_type_adaption_client_creation)
{
  using AdaptedRequestType = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>>;
  using AdaptedResponseType = rclcpp::TypeAdapter<bool, rclcpp::msg::Bool>>;

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = AdaptedResponseType;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");

  /// Now try to adapt the type with the `as` metafunction
  (void)client;

  using AdaptedRequestType = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;
  using AdaptedResponseType = rclcpp::adapt_type<bool>::as<rclcpp::msg::Bool>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = AdaptedResponseType;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");
  (void)client;
}

TEST_F(TestClient, request_type_adaption_client_creation)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using AdaptedRequestType = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = rclcpp::srv::SetBool::Response;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");

  /// Now try to adapt the type with the `as` metafunction
  (void)client;

  using AdaptedRequestType = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;

  struct AdaptedTypeStruct {
    using Request = AdaptedRequestType;
    using Response = rclcpp::srv::SetBool::Response;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");
  (void)client;
}

TEST_F(TestClient, response_type_adaption_client_creation)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using AdaptedResponseType = rclcpp::TypeAdapter<bool, rclcpp::msg::Bool>>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = AdaptedResponseType;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");

  /// Now try to adapt the type with the `as` metafunction
  (void)client;

  using AdaptedResponseType = rclcpp::adapt_type<bool>::as<rclcpp::msg::Bool>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = AdaptedResponseType;
  };

  auto client = node->create_client<AdaptedTypeStruct>("client");
  (void)client;
}

/// Testing that conversion errors are passed up
TEST_F(TestClient, conversion_exception_is_passed_up)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("my_node");

  using BadAdaptedResponseType = rclcpp::TypeAdapter<int, rclcpp::msg::String>>;

  struct AdaptedTypeStruct {
    using Request = rclcpp::msg::String;
    using Response = BadAdaptedResponseType;
  };

  auto client = node->create_client<BadAdaptedTypeStruct>("client");
}
