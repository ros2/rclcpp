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

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "test_msgs/srv/empty.hpp"
#include "test_msgs/srv/basic_types.hpp"

using namespace std::chrono_literals;

class TestGenericService : public ::testing::Test
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

class TestGenericServiceSub : public ::testing::Test
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
   Testing service construction and destruction.
 */
TEST_F(TestGenericService, construction_and_destruction) {
  auto callback = [](
    rclcpp::GenericService::SharedRequest,
    rclcpp::GenericService::SharedResponse) {};
  {
    auto generic_service = node->create_generic_service(
      "test_generic_service", "rcl_interfaces/srv/ListParameters", callback);
    EXPECT_NE(nullptr, generic_service->get_service_handle());
    const rclcpp::ServiceBase * const_service_base = generic_service.get();
    EXPECT_NE(nullptr, const_service_base->get_service_handle());
  }

  {
    ASSERT_THROW(
    {
      auto generic_service = node->create_generic_service(
        "invalid_service?", "test_msgs/srv/Empty", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW(
    {
      auto generic_service = node->create_generic_service(
        "test_generic_service", "test_msgs/srv/NotExist", callback);
    }, rclcpp::exceptions::InvalidServiceTypeError);
  }
}

/*
   Testing service construction and destruction for subnodes.
 */
TEST_F(TestGenericServiceSub, construction_and_destruction) {
  auto callback = [](
    rclcpp::GenericService::SharedRequest,
    rclcpp::GenericService::SharedResponse) {};
  {
    auto generic_service = subnode->create_generic_service(
      "test_generic_service", "rcl_interfaces/srv/ListParameters", callback);
    EXPECT_STREQ(generic_service->get_service_name(), "/ns/sub_ns/test_generic_service");
  }

  {
    ASSERT_THROW(
    {
      auto generic_service = subnode->create_generic_service(
        "invalid_service?", "test_msgs/srv/Empty", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW(
    {
      auto generic_service = subnode->create_generic_service(
        "test_generic_service", "test_msgs/srv/NotExist", callback);
    }, rclcpp::exceptions::InvalidServiceTypeError);
  }
}

TEST_F(TestGenericService, construction_and_destruction_rcl_errors) {
  auto callback = [](
    rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_service_init, RCL_RET_ERROR);
    // reset() isn't necessary for this exception, it just avoids unused return value warning
    EXPECT_THROW(
      node->create_generic_service("service", "test_msgs/srv/Empty", callback).reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_service_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(
      node->create_generic_service("service", "test_msgs/srv/Empty", callback).reset());
  }
}

TEST_F(TestGenericService, generic_service_take_request) {
  auto callback = [](
    rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};
  auto generic_service =
    node->create_generic_service("test_service", "test_msgs/srv/Empty", callback);
  {
    auto request_id = generic_service->create_request_header();
    auto request = generic_service->create_request();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_OK);
    EXPECT_TRUE(generic_service->take_request(request, *request_id.get()));
  }
  {
    auto request_id = generic_service->create_request_header();
    auto request = generic_service->create_request();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_SERVICE_TAKE_FAILED);
    EXPECT_FALSE(generic_service->take_request(request, *request_id.get()));
  }
  {
    auto request_id = generic_service->create_request_header();
    auto request = generic_service->create_request();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_ERROR);
    EXPECT_THROW(
      generic_service->take_request(request, *request_id.get()), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestGenericService, generic_service_send_response) {
  auto callback = [](
    const rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};
  auto generic_service =
    node->create_generic_service("test_service", "test_msgs/srv/Empty", callback);

  {
    auto request_id = generic_service->create_request_header();
    auto response = generic_service->create_response();
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_OK);
    EXPECT_NO_THROW(generic_service->send_response(*request_id.get(), response));
  }

  {
    auto request_id = generic_service->create_request_header();
    auto response = generic_service->create_response();
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_ERROR);
    EXPECT_THROW(
      generic_service->send_response(*request_id.get(), response),
      rclcpp::exceptions::RCLError);
  }
}

/*
   Testing on_new_request callbacks.
 */
TEST_F(TestGenericService, generic_service_on_new_request_callback) {
  auto server_callback = [](
    const rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {FAIL();};
  rclcpp::ServicesQoS service_qos;
  service_qos.keep_last(3);
  auto generic_service = node->create_generic_service(
    "~/test_service", "test_msgs/srv/Empty", server_callback, service_qos);

  std::atomic<size_t> c1 {0};
  auto increase_c1_cb = [&c1](size_t count_msgs) {c1 += count_msgs;};
  generic_service->set_on_new_request_callback(increase_c1_cb);

  auto client = node->create_client<test_msgs::srv::Empty>(
    "~/test_service", service_qos);
  {
    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    client->async_send_request(request);
  }

  auto start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c1 == 0 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);

  std::atomic<size_t> c2 {0};
  auto increase_c2_cb = [&c2](size_t count_msgs) {c2 += count_msgs;};
  generic_service->set_on_new_request_callback(increase_c2_cb);

  {
    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    client->async_send_request(request);
  }

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c2 == 0 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);

  generic_service->clear_on_new_request_callback();

  {
    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    client->async_send_request(request);
    client->async_send_request(request);
    client->async_send_request(request);
  }

  std::atomic<size_t> c3 {0};
  auto increase_c3_cb = [&c3](size_t count_msgs) {c3 += count_msgs;};
  generic_service->set_on_new_request_callback(increase_c3_cb);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c3 < 3 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);
  EXPECT_EQ(c3.load(), 3u);

  std::function<void(size_t)> invalid_cb = nullptr;
  EXPECT_THROW(generic_service->set_on_new_request_callback(invalid_cb), std::invalid_argument);
}

TEST_F(TestGenericService, rcl_service_response_publisher_get_actual_qos_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_service_response_publisher_get_actual_qos, nullptr);
  auto callback = [](
    const rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};
  auto generic_service =
    node->create_generic_service("test_service", "test_msgs/srv/Empty", callback);
  RCLCPP_EXPECT_THROW_EQ(
    generic_service->get_response_publisher_actual_qos(),
    std::runtime_error("failed to get service's response publisher qos settings: error not set"));
}

TEST_F(TestGenericService, rcl_service_request_subscription_get_actual_qos_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_service_request_subscription_get_actual_qos, nullptr);
  auto callback = [](
    const rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};
  auto generic_service =
    node->create_generic_service("test_service", "test_msgs/srv/Empty", callback);
  RCLCPP_EXPECT_THROW_EQ(
    generic_service->get_request_subscription_actual_qos(),
    std::runtime_error("failed to get service's request subscription qos settings: error not set"));
}

TEST_F(TestGenericService, generic_service_qos) {
  rclcpp::ServicesQoS qos_profile;
  qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
  rclcpp::Duration duration(std::chrono::milliseconds(1));
  qos_profile.deadline(duration);
  qos_profile.lifespan(duration);
  qos_profile.liveliness_lease_duration(duration);

  auto callback = [](
    const rclcpp::GenericService::SharedRequest, rclcpp::GenericService::SharedResponse) {};
  auto generic_service =
    node->create_generic_service("test_service", "test_msgs/srv/Empty", callback, qos_profile);

  auto rs_qos = generic_service->get_request_subscription_actual_qos();
  auto rp_qos = generic_service->get_response_publisher_actual_qos();

  EXPECT_EQ(qos_profile, rp_qos);
  // Lifespan has no meaning for subscription/readers
  rs_qos.lifespan(qos_profile.lifespan());
  EXPECT_EQ(qos_profile, rs_qos);
}

TEST_F(TestGenericService, generic_service_qos_depth) {
  uint64_t server_cb_count_ = 0;
  auto server_callback = [&](
    const rclcpp::GenericService::SharedRequest,
    rclcpp::GenericService::SharedResponse) {server_cb_count_++;};

  auto server_node = std::make_shared<rclcpp::Node>("server_node", "/ns");

  rclcpp::QoS server_qos_profile(2);

  auto generic_service = server_node->create_generic_service(
    "test_qos_depth", "test_msgs/srv/Empty", std::move(server_callback), server_qos_profile);

  rclcpp::QoS client_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  auto client = node->create_client<test_msgs::srv::Empty>("test_qos_depth", client_qos_profile);

  ::testing::AssertionResult request_result = ::testing::AssertionSuccess();
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  auto client_callback = [&request_result](
    rclcpp::Client<test_msgs::srv::Empty>::SharedFuture future_response) {
      if (nullptr == future_response.get()) {
        request_result = ::testing::AssertionFailure() << "Future response was null";
      }
    };

  uint64_t client_requests = 5;
  for (uint64_t i = 0; i < client_requests; i++) {
    client->async_send_request(request, client_callback);
    std::this_thread::sleep_for(10ms);
  }

  auto start = std::chrono::steady_clock::now();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  while ((server_cb_count_ < server_qos_profile.depth()) &&
    (std::chrono::steady_clock::now() - start) < 1s)
  {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  // Spin an extra time to check if server QoS depth has been ignored,
  // so more server responses might be processed than expected.
  executor.spin_some();

  EXPECT_EQ(server_cb_count_, server_qos_profile.depth());
}

TEST_F(TestGenericService, generic_service_and_client) {
  const std::string service_name = "test_service";
  const std::string service_type = "test_msgs/srv/BasicTypes";
  int64_t expected_change = 87654321;

  auto callback = [&expected_change](
    const rclcpp::GenericService::SharedRequest request,
    rclcpp::GenericService::SharedResponse response) {
      auto typed_request = static_cast<test_msgs::srv::BasicTypes_Request *>(request.get());
      auto typed_response = static_cast<test_msgs::srv::BasicTypes_Response *>(response.get());

      typed_response->int64_value = typed_request->int64_value + expected_change;
    };
  auto generic_service = node->create_generic_service(service_name, service_type, callback);

  auto client = node->create_client<test_msgs::srv::BasicTypes>(service_name);

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(client->service_is_ready());

  auto request = std::make_shared<test_msgs::srv::BasicTypes::Request>();
  request->int64_value = 12345678;

  auto generic_client_callback = [&request, &expected_change](
    std::shared_future<test_msgs::srv::BasicTypes_Response::SharedPtr> future) {
      auto response = future.get();
      EXPECT_EQ(response->int64_value, (request->int64_value + expected_change));
    };

  auto future =
    client->async_send_request(request, generic_client_callback);
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(), future, std::chrono::seconds(5));
}
