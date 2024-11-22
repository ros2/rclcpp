// Copyright 2024 Sony Group Corporation.
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

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "rclcpp/create_generic_client.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/srv/empty.hpp"

template<typename T>
class TestAllClientTypesWithServer : public ::testing::Test
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

  template<typename ClientType>
  auto SendEmptyRequestAndWait(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    if constexpr (std::is_same_v<ClientType, rclcpp::GenericClient>) {
      return GenericClientSendEmptyRequestAndWait(timeout);
    } else if constexpr (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
      return ClientSendEmptyRequestAndWait(timeout);
    } else {
      return ::testing::AssertionFailure() << "No test for this client type";
    }
  }

  ::testing::AssertionResult GenericClientSendEmptyRequestAndWait(
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

  ::testing::AssertionResult ClientSendEmptyRequestAndWait(
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

  template<typename ClientType>
  auto create_client(
    rclcpp::Node::SharedPtr node,
    const std::string service_name = "empty_service",
    const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  {
    if constexpr (std::is_same_v<ClientType, rclcpp::GenericClient>) {
      return node->create_generic_client(service_name, "test_msgs/srv/Empty", qos);
    } else if constexpr (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
      return node->template create_client<test_msgs::srv::Empty>(service_name, qos);
    } else {
      ASSERT_TRUE(false) << "Not know how to create this kind of client";
    }
  }

  template<typename ClientType, typename RequestType>
  auto async_send_request(std::shared_ptr<ClientType> client, std::shared_ptr<RequestType> request)
  {
    if constexpr (std::is_same_v<ClientType, rclcpp::GenericClient>) {
      return client->async_send_request(request.get());
    } else if constexpr (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
      return client->async_send_request(request);
    } else {
      ASSERT_TRUE(false) << "Not know how to send request for this kind of client";
    }
  }

  template<typename ClientType, typename ResponseType>
  auto take_response(
    std::shared_ptr<ClientType> client,
    ResponseType & response,
    std::shared_ptr<rmw_request_id_t> request_header)
  {
    if constexpr (std::is_same_v<ClientType, rclcpp::GenericClient>) {
      return client->take_response(static_cast<void *>(&response), *request_header.get());
    } else if constexpr (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
      return client->take_response(response, *request_header.get());
    } else {
      ASSERT_TRUE(false) << "Not know how to take response for this kind of client";
    }
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Service<test_msgs::srv::Empty>> service;
  const std::string service_name{"empty_service"};
};

using ClientType =
  ::testing::Types<
  rclcpp::Client<test_msgs::srv::Empty>,
  rclcpp::GenericClient>;

class ClientTypeNames
{
public:
  template<typename T>
  static std::string GetName(int idx)
  {
    (void)idx;
    if (std::is_same_v<T, rclcpp::Client<test_msgs::srv::Empty>>) {
      return "Client";
    }

    if (std::is_same_v<T, rclcpp::GenericClient>) {
      return "GenericClient";
    }

    return "";
  }
};

TYPED_TEST_SUITE(TestAllClientTypesWithServer, ClientType, ClientTypeNames);

TYPED_TEST(TestAllClientTypesWithServer, async_send_request)
{
  using ClientType = TypeParam;
  EXPECT_TRUE(this->template SendEmptyRequestAndWait<ClientType>());
}

TYPED_TEST(TestAllClientTypesWithServer, test_client_remove_pending_request)
{
  using ClientType = TypeParam;

  auto client = this->template create_client<ClientType>(this->node);

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  auto future_and_req_id = this->template async_send_request<
    ClientType, test_msgs::srv::Empty::Request>(client, request);

  EXPECT_TRUE(client->remove_pending_request(future_and_req_id.request_id));
}

TYPED_TEST(TestAllClientTypesWithServer, prune_requests_older_than_no_pruned)
{
  using ClientType = TypeParam;

  auto client = this->template create_client<ClientType>(this->node);

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  auto future = this->template async_send_request<
    ClientType, test_msgs::srv::Empty::Request>(client, request);
  auto time = std::chrono::system_clock::now() + std::chrono::seconds(1);

  EXPECT_EQ(1u, client->prune_requests_older_than(time));
}

TYPED_TEST(TestAllClientTypesWithServer, prune_requests_older_than_with_pruned)
{
  using ClientType = TypeParam;

  auto client = this->template create_client<ClientType>(this->node);

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  auto future = this->template async_send_request<
    ClientType, test_msgs::srv::Empty::Request>(client, request);
  auto time = std::chrono::system_clock::now() + std::chrono::seconds(1);

  std::vector<int64_t> pruned_requests;
  EXPECT_EQ(1u, client->prune_requests_older_than(time, &pruned_requests));
  ASSERT_EQ(1u, pruned_requests.size());
  EXPECT_EQ(future.request_id, pruned_requests[0]);
}

TYPED_TEST(TestAllClientTypesWithServer, async_send_request_rcl_send_request_error)
{
  using ClientType = TypeParam;

  // Checking rcl_send_request in rclcpp::Client::async_send_request() or
  // rclcpp::GenericClient::async_send_request()
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_request, RCL_RET_ERROR);
  EXPECT_THROW(this->template SendEmptyRequestAndWait<ClientType>(), rclcpp::exceptions::RCLError);
}

TYPED_TEST(TestAllClientTypesWithServer, async_send_request_rcl_service_server_is_available_error)
{
  using ClientType = TypeParam;

  {
    // Checking rcl_service_server_is_available in rclcpp::ClientBase::service_is_ready
    auto client = this->template create_client<ClientType>(this->node);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_NODE_INVALID);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = this->template create_client<ClientType>(this->node);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
  {
    // Checking rcl_service_server_is_available exception in rclcpp::ClientBase::service_is_ready
    auto client = this->template create_client<ClientType>(this->node);
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_service_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(client->service_is_ready(), rclcpp::exceptions::RCLError);
  }
}

TYPED_TEST(TestAllClientTypesWithServer, take_response)
{
  using ClientType = TypeParam;

  auto client = this->template create_client<ClientType>(this->node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto request_header = client->create_request_header();
  test_msgs::srv::Empty::Response response;

  this->template async_send_request<
    ClientType, test_msgs::srv::Empty::Request>(client, request);

  EXPECT_FALSE(this->take_response(client, response, request_header));

  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_OK);
    EXPECT_TRUE(this->take_response(client, response, request_header));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_CLIENT_TAKE_FAILED);
    EXPECT_FALSE(this->take_response(client, response, request_header));
  }
  {
    // Checking rcl_take_response in rclcpp::ClientBase::take_type_erased_response
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_response, RCL_RET_ERROR);
    EXPECT_THROW(
      this->take_response(client, response, request_header),
      rclcpp::exceptions::RCLError);
  }
}

/*
   Testing on_new_response callbacks.
 */
TYPED_TEST(TestAllClientTypesWithServer, on_new_response_callback)
{
  using ClientType = TypeParam;

  auto client_node = std::make_shared<rclcpp::Node>("test_client_node", "ns");
  auto server_node = std::make_shared<rclcpp::Node>("test_server_node", "ns");

  rclcpp::ServicesQoS client_qos;
  client_qos.keep_last(3);

  auto client = this->template create_client<ClientType>(client_node, "test_service", client_qos);

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

  this->template async_send_request<ClientType, test_msgs::srv::Empty::Request>(client, request);
  auto start = std::chrono::steady_clock::now();
  while (server_requests_count == 0 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(10))
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 1u);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (c1 == 0 && std::chrono::steady_clock::now() - start < std::chrono::seconds(10));

  EXPECT_EQ(c1.load(), 1u);

  std::atomic<size_t> c2 {0};
  auto increase_c2_cb = [&c2](size_t count_msgs) {c2 += count_msgs;};
  client->set_on_new_response_callback(increase_c2_cb);

  this->template async_send_request<ClientType, test_msgs::srv::Empty::Request>(client, request);
  start = std::chrono::steady_clock::now();
  while (server_requests_count == 1 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(10))
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 2u);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (c1 == 0 && std::chrono::steady_clock::now() - start < std::chrono::seconds(10));

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);

  client->clear_on_new_response_callback();

  this->template async_send_request<ClientType, test_msgs::srv::Empty::Request>(client, request);
  this->template async_send_request<ClientType, test_msgs::srv::Empty::Request>(client, request);
  this->template async_send_request<ClientType, test_msgs::srv::Empty::Request>(client, request);
  start = std::chrono::steady_clock::now();
  while (server_requests_count < 5 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(10))
  {
    rclcpp::spin_some(server_node);
  }

  ASSERT_EQ(server_requests_count, 5u);

  std::atomic<size_t> c3 {0};
  auto increase_c3_cb = [&c3](size_t count_msgs) {c3 += count_msgs;};
  client->set_on_new_response_callback(increase_c3_cb);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (c3 < 3 && std::chrono::steady_clock::now() - start < std::chrono::seconds(10));

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);
  EXPECT_EQ(c3.load(), 3u);

  std::function<void(size_t)> invalid_cb = nullptr;
  EXPECT_THROW(client->set_on_new_response_callback(invalid_cb), std::invalid_argument);
}

TYPED_TEST(TestAllClientTypesWithServer, client_qos)
{
  using ClientType = TypeParam;

  rclcpp::ServicesQoS qos_profile;
  qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
  rclcpp::Duration duration(std::chrono::milliseconds(1));
  qos_profile.deadline(duration);
  qos_profile.lifespan(duration);
  qos_profile.liveliness_lease_duration(duration);

  auto client = this->template create_client<ClientType>(
    this->node, this->service_name, qos_profile);

  auto rp_qos = client->get_request_publisher_actual_qos();
  auto rs_qos = client->get_response_subscription_actual_qos();

  EXPECT_EQ(qos_profile, rp_qos);
  // Lifespan has no meaning for subscription/readers
  rs_qos.lifespan(qos_profile.lifespan());
  EXPECT_EQ(qos_profile, rs_qos);
}

TYPED_TEST(TestAllClientTypesWithServer, rcl_client_request_publisher_get_actual_qos_error)
{
  using ClientType = TypeParam;

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_client_request_publisher_get_actual_qos, nullptr);
  auto client = this->template create_client<ClientType>(this->node, "service");
  RCLCPP_EXPECT_THROW_EQ(
    client->get_request_publisher_actual_qos(),
    std::runtime_error("failed to get client's request publisher qos settings: error not set"));
}

TYPED_TEST(TestAllClientTypesWithServer, rcl_client_response_subscription_get_actual_qos_error)
{
  using ClientType = TypeParam;

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_client_response_subscription_get_actual_qos, nullptr);
  auto client = this->template create_client<ClientType>(this->node, "service");
  RCLCPP_EXPECT_THROW_EQ(
    client->get_response_subscription_actual_qos(),
    std::runtime_error("failed to get client's response subscription qos settings: error not set"));
}

// The following tests are only for rclcpp::Client
void client_async_send_request_callback_with_request(
  rclcpp::Node::SharedPtr node, const std::string service_name)
{
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
TYPED_TEST(TestAllClientTypesWithServer, async_send_request_callback_with_request)
{
  using ClientType = TypeParam;

  if (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
    client_async_send_request_callback_with_request(this->node, this->service_name);
  } else if (std::is_same_v<ClientType, rclcpp::GenericClient>) {
    GTEST_SKIP() << "Skipping test for GenericClient";
  } else {
    GTEST_SKIP() << "Skipping test";
  }
}

void client_qos_depth(rclcpp::Node::SharedPtr node)
{
  using namespace std::literals::chrono_literals;

  rclcpp::ServicesQoS client_qos_profile;
  client_qos_profile.keep_last(2);

  auto client = node->create_client<test_msgs::srv::Empty>("test_qos_depth", client_qos_profile);

  uint64_t server_cb_count_ = 0;
  auto server_callback = [&](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {server_cb_count_++;};

  auto server_node = std::make_shared<rclcpp::Node>("server_node", "/ns");

  rclcpp::QoS server_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

  auto server = server_node->create_service<test_msgs::srv::Empty>(
    "test_qos_depth", std::move(server_callback), server_qos);

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  ::testing::AssertionResult request_result = ::testing::AssertionSuccess();

  using SharedFuture = rclcpp::Client<test_msgs::srv::Empty>::SharedFuture;
  uint64_t client_cb_count_ = 0;
  auto client_callback = [&client_cb_count_, &request_result](SharedFuture future_response) {
      if (nullptr == future_response.get()) {
        request_result = ::testing::AssertionFailure() << "Future response was null";
      }
      client_cb_count_++;
    };

  uint64_t client_requests = 5;
  for (uint64_t i = 0; i < client_requests; i++) {
    client->async_send_request(request, client_callback);
    std::this_thread::sleep_for(10ms);
  }

  auto start = std::chrono::steady_clock::now();
  while ((server_cb_count_ < client_requests) &&
    (std::chrono::steady_clock::now() - start) < 2s)
  {
    rclcpp::spin_some(server_node);
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_GT(server_cb_count_, client_qos_profile.depth());

  start = std::chrono::steady_clock::now();
  while ((client_cb_count_ < client_qos_profile.depth()) &&
    (std::chrono::steady_clock::now() - start) < 1s)
  {
    rclcpp::spin_some(node);
  }

  // Spin an extra time to check if client QoS depth has been ignored,
  // so more client callbacks might be called than expected.
  rclcpp::spin_some(node);

  EXPECT_EQ(client_cb_count_, client_qos_profile.depth());
}

TYPED_TEST(TestAllClientTypesWithServer, qos_depth)
{
  using ClientType = TypeParam;

  if (std::is_same_v<ClientType, rclcpp::Client<test_msgs::srv::Empty>>) {
    client_qos_depth(this->node);
  } else if (std::is_same_v<ClientType, rclcpp::GenericClient>) {
    GTEST_SKIP() << "Skipping test for GenericClient";
  } else {
    GTEST_SKIP() << "Skipping test";
  }
}
