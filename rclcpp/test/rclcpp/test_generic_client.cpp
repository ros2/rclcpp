// Copyright 2023 Sony Group Corporation.
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

#include "rclcpp/create_generic_client.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/srv/empty.hpp"

using namespace std::chrono_literals;

// All tests are from test_client

class TestGenericClient : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestGenericClientSub : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_node", "/ns");
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
TEST_F(TestGenericClient, construction_and_destruction) {
  {
    auto client = node->create_generic_client("test_service", "test_msgs/srv/Empty");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("invalid_test_service?", "test_msgs/srv/Empty");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("test_service", "test_msgs/srv/InvalidType");
    }, std::runtime_error);
  }
}

TEST_F(TestGenericClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_generic_client(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "test_service",
      "test_msgs/srv/Empty",
      rclcpp::ServicesQoS(),
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?test_service",
        "test_msgs/srv/Empty",
        rclcpp::ServicesQoS(),
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "test_service",
        "test_msgs/srv/InvalidType",
        rclcpp::ServicesQoS(),
        nullptr);
    }, std::runtime_error);
  }
}

TEST_F(TestGenericClient, construct_with_rcl_error) {
  {
    // reset() is not necessary for this exception, but handles unused return value warning
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_init, RCL_RET_ERROR);
    EXPECT_THROW(
      node->create_generic_client("test_service", "test_msgs/srv/Empty").reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(
      node->create_generic_client("test_service", "test_msgs/srv/Empty").reset());
  }
}

TEST_F(TestGenericClient, wait_for_service) {
  const std::string service_name = "test_service";
  auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
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
   Testing generic client construction and destruction for subnodes.
 */
TEST_F(TestGenericClientSub, construction_and_destruction) {
  {
    auto client = subnode->create_generic_client("test_service", "test_msgs/srv/Empty");
    EXPECT_STREQ(client->get_service_name(), "/ns/test_service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("invalid_service?", "test_msgs/srv/Empty");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

class TestGenericClientWithServer : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_node", "ns");

    auto callback = [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};

    service = node->create_service<test_msgs::srv::Empty>(service_name, std::move(callback));
  }

  ::testing::AssertionResult SendEmptyRequestAndWait(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
    if (!client->wait_for_service()) {
      return ::testing::AssertionFailure() << "Service is not available yet";
    }

    auto request = std::make_shared<test_msgs::srv::Empty::Request>();

    auto future_and_req_id = client->async_send_request(request.get());

    auto ret = rclcpp::spin_until_future_complete(node, future_and_req_id, timeout);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      return ::testing::AssertionFailure() << "Waiting for response timed out";
    }

    if (client->remove_pending_request(future_and_req_id.request_id)) {
      return ::testing::AssertionFailure() << "Should not be able to remove a finished request";
    }

    return ::testing::AssertionSuccess();
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Service<test_msgs::srv::Empty>> service;
  const std::string service_name{"empty_service"};
};

TEST_F(TestGenericClientWithServer, async_send_request) {
  EXPECT_TRUE(SendEmptyRequestAndWait());
}

TEST_F(TestGenericClientWithServer, test_client_remove_pending_request) {
  auto client =
    node->create_generic_client("no_service_server_available_here", "test_msgs/srv/Empty");
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto future_and_req_id = client->async_send_request(request.get());

  EXPECT_TRUE(client->remove_pending_request(future_and_req_id.request_id));
}

TEST_F(TestGenericClientWithServer, prune_requests_older_than_no_pruned) {
  auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto future = client->async_send_request(request.get());
  auto time = std::chrono::system_clock::now() + 1s;

  EXPECT_EQ(1u, client->prune_requests_older_than(time));
}

TEST_F(TestGenericClientWithServer, prune_requests_older_than_with_pruned) {
  auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto future = client->async_send_request(request.get());
  auto time = std::chrono::system_clock::now() + 1s;

  std::vector<int64_t> pruned_requests;
  EXPECT_EQ(1u, client->prune_requests_older_than(time, &pruned_requests));
  ASSERT_EQ(1u, pruned_requests.size());
  EXPECT_EQ(future.request_id, pruned_requests[0]);
}

TEST_F(TestGenericClientWithServer, async_send_request_rcl_send_request_error) {
  // Checking rcl_send_request in rclcpp::Client::async_send_request()
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_request, RCL_RET_ERROR);
  EXPECT_THROW(SendEmptyRequestAndWait(), rclcpp::exceptions::RCLError);
}

TEST_F(TestGenericClientWithServer, async_send_request_rcl_service_server_is_available_error) {
  {
    // Checking rcl_service_server_is_available in rclcpp::ClientBase::service_is_ready
    auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_NODE_INVALID);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestGenericClientWithServer, take_response) {
  auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto request_header = client->create_request_header();
  test_msgs::srv::Empty::Response response;

  rclcpp::Serialization<test_msgs::srv::Empty::Request> serializer;
  client->async_send_request(request.get());
  EXPECT_FALSE(client->take_response(static_cast<void *>(&response), *request_header.get()));

  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_OK);
    EXPECT_TRUE(client->take_response(static_cast<void *>(&response), *request_header.get()));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_CLIENT_TAKE_FAILED);
    EXPECT_FALSE(client->take_response(static_cast<void *>(&response), *request_header.get()));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_ERROR);
    EXPECT_THROW(
      client->take_response(static_cast<void *>(&response), *request_header.get()),
      rclcpp::exceptions::RCLError);
  }
}

/*
   Testing on_new_response callbacks.
 */
TEST_F(TestGenericClient, on_new_response_callback) {
  auto client_node = std::make_shared<rclcpp::Node>("test_client_node", "ns");
  auto server_node = std::make_shared<rclcpp::Node>("test_server_node", "ns");

  rclcpp::ServicesQoS client_qos;
  client_qos.keep_last(3);
  auto client =
    client_node->create_generic_client("test_service", "test_msgs/srv/Empty", client_qos);
  std::atomic<size_t> server_requests_count {0};
  auto server_callback = [&server_requests_count](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {server_requests_count++;};
  auto server = server_node->create_service<test_msgs::srv::Empty>(
    "test_service", server_callback, client_qos);
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  std::atomic<size_t> c1 {0};
  auto increase_c1_cb = [&c1](size_t count_msgs) {c1 += count_msgs;};
  client->set_on_new_response_callback(increase_c1_cb);

  client->async_send_request(request.get());
  auto start = std::chrono::steady_clock::now();
  while (server_requests_count == 0 &&
    (std::chrono::steady_clock::now() - start) < 10s)
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 1u);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c1 == 0 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);

  std::atomic<size_t> c2 {0};
  auto increase_c2_cb = [&c2](size_t count_msgs) {c2 += count_msgs;};
  client->set_on_new_response_callback(increase_c2_cb);

  client->async_send_request(request.get());
  start = std::chrono::steady_clock::now();
  while (server_requests_count == 1 &&
    (std::chrono::steady_clock::now() - start) < 10s)
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 2u);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c1 == 0 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);

  client->clear_on_new_response_callback();

  client->async_send_request(request.get());
  client->async_send_request(request.get());
  client->async_send_request(request.get());
  start = std::chrono::steady_clock::now();
  while (server_requests_count < 5 &&
    (std::chrono::steady_clock::now() - start) < 10s)
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 5u);

  std::atomic<size_t> c3 {0};
  auto increase_c3_cb = [&c3](size_t count_msgs) {c3 += count_msgs;};
  client->set_on_new_response_callback(increase_c3_cb);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c3 < 3 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);
  EXPECT_EQ(c3.load(), 3u);

  std::function<void(size_t)> invalid_cb = nullptr;
  EXPECT_THROW(client->set_on_new_response_callback(invalid_cb), std::invalid_argument);
}

TEST_F(TestGenericClient, client_qos) {
  rclcpp::ServicesQoS qos_profile;
  qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
  rclcpp::Duration duration(std::chrono::nanoseconds(1));
  qos_profile.deadline(duration);
  qos_profile.lifespan(duration);
  qos_profile.liveliness_lease_duration(duration);

  auto client =
    node->create_generic_client("test_client", "test_msgs/srv/Empty", qos_profile);

  auto rp_qos = client->get_request_publisher_actual_qos();
  auto rs_qos = client->get_response_subscription_actual_qos();

  EXPECT_EQ(qos_profile, rp_qos);
  // Lifespan has no meaning for subscription/readers
  rs_qos.lifespan(qos_profile.lifespan());
  EXPECT_EQ(qos_profile, rs_qos);
}
