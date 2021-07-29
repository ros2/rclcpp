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

#include <string>
#include <memory>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

#include "../mocking_utils/patch.hpp"

#include "test_msgs/srv/empty.hpp"

class TestClient : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestClientSub : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

/*
   Testing client construction and destruction.
 */
TEST_F(TestClient, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = node->create_client<ListParameters>("service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "service",
      rmw_qos_profile_services_default,
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?service",
        rmw_qos_profile_services_default,
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construct_with_rcl_error) {
  {
    // reset() is not necessary for this exception, but handles unused return value warning
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_init, RCL_RET_ERROR);
    EXPECT_THROW(
      node->create_client<test_msgs::srv::Empty>("service").reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(node->create_client<test_msgs::srv::Empty>("service").reset());
  }
}

TEST_F(TestClient, wait_for_service) {
  const std::string service_name = "service";
  auto client = node->create_client<test_msgs::srv::Empty>(service_name);
  EXPECT_FALSE(client->wait_for_service(std::chrono::nanoseconds(0)));
  EXPECT_FALSE(client->wait_for_service(std::chrono::milliseconds(10)));

  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {};

  auto service =
    node->create_service<test_msgs::srv::Empty>(service_name, std::move(callback));

  EXPECT_TRUE(client->wait_for_service(std::chrono::nanoseconds(-1)));
  EXPECT_TRUE(client->service_is_ready());
}

/*
   Testing client construction and destruction for subnodes.
 */
TEST_F(TestClientSub, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = subnode->create_client<ListParameters>("service");
    EXPECT_STREQ(client->get_service_name(), "/ns/sub_ns/service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

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
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};

    service = node->create_service<test_msgs::srv::Empty>(service_name, std::move(callback));
  }

  ::testing::AssertionResult SendEmptyRequestAndWait(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    using SharedFuture = rclcpp::Client<test_msgs::srv::Empty>::SharedFuture;

    auto client = node->create_client<test_msgs::srv::Empty>(service_name);
    if (!client->wait_for_service()) {
      return ::testing::AssertionFailure() << "Waiting for service failed";
    }

    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    bool received_response = false;
    ::testing::AssertionResult request_result = ::testing::AssertionSuccess();
    auto callback = [&received_response, &request_result](SharedFuture future_response) {
        if (nullptr == future_response.get()) {
          request_result = ::testing::AssertionFailure() << "Future response was null";
        }
        received_response = true;
      };

    auto future = client->async_send_request(request, std::move(callback));

    auto start = std::chrono::steady_clock::now();
    while (!received_response &&
      (std::chrono::steady_clock::now() - start) < timeout)
    {
      rclcpp::spin_some(node);
    }

    if (!received_response) {
      return ::testing::AssertionFailure() << "Waiting for response timed out";
    }

    return request_result;
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Service<test_msgs::srv::Empty>> service;
  const std::string service_name{"empty_service"};
};

TEST_F(TestClientWithServer, async_send_request) {
  EXPECT_TRUE(SendEmptyRequestAndWait());
}

TEST_F(TestClientWithServer, async_send_request_callback_with_request) {
  using SharedFutureWithRequest =
    rclcpp::Client<test_msgs::srv::Empty>::SharedFutureWithRequest;

  auto client = node->create_client<test_msgs::srv::Empty>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  bool received_response = false;
  auto callback = [&request, &received_response](SharedFutureWithRequest future) {
      auto request_response_pair = future.get();
      EXPECT_EQ(request, request_response_pair.first);
      EXPECT_NE(nullptr, request_response_pair.second);
      received_response = true;
    };
  auto future = client->async_send_request(request, std::move(callback));

  auto start = std::chrono::steady_clock::now();
  while (!received_response &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1))
  {
    rclcpp::spin_some(node);
  }
  EXPECT_TRUE(received_response);
}

TEST_F(TestClientWithServer, async_send_request_rcl_send_request_error) {
  // Checking rcl_send_request in rclcpp::Client::async_send_request()
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_request, RCL_RET_ERROR);
  EXPECT_THROW(SendEmptyRequestAndWait(), rclcpp::exceptions::RCLError);
}

TEST_F(TestClientWithServer, async_send_request_rcl_service_server_is_available_error) {
  {
    // Checking rcl_service_server_is_available in rclcpp::ClientBase::service_is_ready
    auto client = node->create_client<test_msgs::srv::Empty>(service_name);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_NODE_INVALID);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = node->create_client<test_msgs::srv::Empty>(service_name);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = node->create_client<test_msgs::srv::Empty>(service_name);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestClientWithServer, take_response) {
  auto client = node->create_client<test_msgs::srv::Empty>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto request_header = client->create_request_header();
  test_msgs::srv::Empty::Response response;

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
