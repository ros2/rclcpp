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

class TestClient : public ::testing::Test
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

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestClientWithServer : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("node", "ns");

    auto callback = [](
      const std::shared_ptr<CustomBool::Request>,
      const std::shared_ptr<rclcpp::srv::SetBool::Response>) {};

    service = node->create_service<AdaptedTypeStruct>(service_name, std::move(callback));
  }

  ::testing::AssertionResult SendBoolRequestAndWait(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    using SharedFuture = rclcpp::Client<AdaptedTypeStruct>::CustomSharedFuture;

    auto client = node->create_client<AdaptedTypeStruct>(service_name);
    if (!client->wait_for_service()) {
      return ::testing::AssertionFailure() << "Waiting for service failed";
    }

    auto request = std::make_shared<CustomBool::Request>();
    bool received_response = false;
    ::testing::AssertionResult request_result = ::testing::AssertionSuccess();
    auto callback = [&received_response, &request_result](SharedFuture future_response) {
        if (nullptr == future_response.get()) {
          request_result = ::testing::AssertionFailure() << "Future response was null";
        }
        received_response = true;
      };

    auto req_id = client->async_send_request(request, std::move(callback));

    auto start = std::chrono::steady_clock::now();
    while (!received_response &&
      (std::chrono::steady_clock::now() - start) < timeout)
    {
      rclcpp::spin_some(node);
    }

    if (!received_response) {
      return ::testing::AssertionFailure() << "Waiting for response timed out";
    }
    if (client->remove_pending_request(req_id)) {
      return ::testing::AssertionFailure() << "Should not be able to remove a finished request";
    }

    return request_result;
  }

  using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Service<AdaptedTypeStruct>> service;
  const std::string service_name{"empty_service"};
};

/*
 * Testing the basic creation of clients with a TypeAdapter for both Request and Response
 */
TEST_F(TestClient, various_creation_signatures)
{
  {
    using AdaptedTypeStruct = rclcpp::TypeAdapter<CustomBool, rclcpp::srv::SetBool>;
    auto client = node->create_client<AdaptedTypeStruct>("client");

    (void)client;
  }
  {
    /// Now try to adapt the type with the `as` metafunction
    using AdaptedTypeStruct = rclcpp::adapt_type<CustomBool>::as<rclcpp::srv::SetBool>;

    auto client = node->create_client<AdaptedTypeStruct>("client");
    (void)client;
  }
}

/// Testing that conversion errors are passed up
TEST_F(TestClient, conversion_exception_is_passed_up)
{
  using BadAdaptedTypeStruct = rclcpp::TypeAdapter<CustomIncorrectBool, rclcpp::srv::SetBool>;

  auto client = node->create_client<BadAdaptedTypeStruct>("client");
}

TEST_F(TestClientWithServer, test_adapted_client_remove_pending_request) {
  auto client = node->create_client<AdaptedTypeStruct>("no_service_server_available_here");

  auto request = std::make_shared<CustomBool::Request>();
  auto future = client->async_send_request(request);

  EXPECT_TRUE(client->remove_pending_request(future));
}

TEST_F(TestClientWithServer, take_adapted_response)
{
  auto client = node->create_client<AdaptedTypeStruct>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<CustomBool::Request>();
  auto request_header = client->create_request_header();
  CustomBool::Response response;

  client->async_send_request(request);
  EXPECT_FALSE(client->take_response(response, *request_header.get()));

  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_OK);
    EXPECT_TRUE(client->take_response(response, *request_header.get()));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_CLIENT_TAKE_FAILED);
    EXPECT_FALSE(client->take_response(response, *request_header.get()));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_ERROR);
    EXPECT_THROW(
      client->take_response(response, *request_header.get()),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestClientWithServer, async_send_request) {
  EXPECT_TRUE(SendBoolRequestAndWait());
}

TEST_F(TestClientWithServer, async_send_request_callback_with_request) {
  using SharedFutureWithRequest =
    rclcpp::Client<AdaptedTypeStruct>::CustomTotalSharedFutureWithRequest;

  auto client = node->create_client<AdaptedTypeStruct>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<CustomBool::Request>();
  bool received_response = false;
  auto callback = [&request, &received_response](SharedFutureWithRequest future) {
      auto request_response_pair = future.get();
      EXPECT_EQ(request, request_response_pair.first);
      EXPECT_NE(nullptr, request_response_pair.second);
      received_response = true;
    };
  auto req_id = client->async_send_request(request, std::move(callback));

  auto start = std::chrono::steady_clock::now();
  while (!received_response &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1))
  {
    rclcpp::spin_some(node);
  }
  EXPECT_TRUE(received_response);
  EXPECT_FALSE(client->remove_pending_request(req_id));
}
