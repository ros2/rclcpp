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

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "test_msgs/srv/empty.hpp"
#include "test_msgs/srv/empty.h"

using namespace std::chrono_literals;

class TestService : public ::testing::Test
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

class TestServiceSub : public ::testing::Test
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
   Testing service construction and destruction.
 */
TEST_F(TestService, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  {
    auto service = node->create_service<ListParameters>("service", callback);
    EXPECT_NE(nullptr, service->get_service_handle());
    const rclcpp::ServiceBase * const_service_base = service.get();
    EXPECT_NE(nullptr, const_service_base->get_service_handle());
  }

  {
    ASSERT_THROW(
    {
      auto service = node->create_service<ListParameters>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Testing service construction and destruction for subnodes.
 */
TEST_F(TestServiceSub, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  {
    auto service = subnode->create_service<ListParameters>("service", callback);
    EXPECT_STREQ(service->get_service_name(), "/ns/sub_ns/service");
  }

  {
    ASSERT_THROW(
    {
      auto service = node->create_service<ListParameters>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestService, construction_and_destruction_rcl_errors) {
  auto callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_service_init, RCL_RET_ERROR);
    // reset() isn't necessary for this exception, it just avoids unused return value warning
    EXPECT_THROW(
      node->create_service<test_msgs::srv::Empty>("service", callback).reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_service_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(node->create_service<test_msgs::srv::Empty>("service", callback).reset());
  }
}

/* Testing basic getters */
TEST_F(TestService, basic_public_getters) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  auto service = node->create_service<ListParameters>("service", callback);
  EXPECT_STREQ(service->get_service_name(), "/ns/service");
  std::shared_ptr<rcl_service_t> service_handle = service->get_service_handle();
  EXPECT_NE(nullptr, service_handle);

  {
    // Create a extern defined const service
    auto node_handle_int = rclcpp::Node::make_shared("base_node");
    rcl_service_t service_handle = rcl_get_zero_initialized_service();
    rcl_service_options_t service_options = rcl_service_get_default_options();
    const rosidl_service_type_support_t * ts =
      rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::Empty>();
    rcl_ret_t ret = rcl_service_init(
      &service_handle,
      node_handle_int->get_node_base_interface()->get_rcl_node_handle(),
      ts, "base_node_service", &service_options);
    if (ret != RCL_RET_OK) {
      FAIL();
      return;
    }
    rclcpp::AnyServiceCallback<test_msgs::srv::Empty> cb;
    const rclcpp::Service<test_msgs::srv::Empty> base(
      node_handle_int->get_node_base_interface()->get_shared_rcl_node_handle(),
      &service_handle, cb);
    // Use get_service_handle specific to const service
    std::shared_ptr<const rcl_service_t> const_service_handle = base.get_service_handle();
    EXPECT_NE(nullptr, const_service_handle);

    EXPECT_EQ(
      RCL_RET_OK, rcl_service_fini(
        &service_handle,
        node_handle_int->get_node_base_interface()->get_rcl_node_handle()));
  }
}

TEST_F(TestService, take_request) {
  auto callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
  auto server = node->create_service<test_msgs::srv::Empty>("service", callback);
  {
    auto request_id = server->create_request_header();
    test_msgs::srv::Empty::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_OK);
    EXPECT_TRUE(server->take_request(request, *request_id.get()));
  }
  {
    auto request_id = server->create_request_header();
    test_msgs::srv::Empty::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_SERVICE_TAKE_FAILED);
    EXPECT_FALSE(server->take_request(request, *request_id.get()));
  }
  {
    auto request_id = server->create_request_header();
    test_msgs::srv::Empty::Request request;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_take_request, RCL_RET_ERROR);
    EXPECT_THROW(server->take_request(request, *request_id.get()), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestService, send_response) {
  auto callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
  auto server = node->create_service<test_msgs::srv::Empty>("service", callback);

  {
    auto request_id = server->create_request_header();
    test_msgs::srv::Empty::Response response;
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_OK);
    EXPECT_NO_THROW(server->send_response(*request_id.get(), response));
  }

  {
    auto request_id = server->create_request_header();
    test_msgs::srv::Empty::Response response;
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_send_response, RCL_RET_ERROR);
    EXPECT_THROW(
      server->send_response(*request_id.get(), response),
      rclcpp::exceptions::RCLError);
  }
}

/*
   Testing on_new_request callbacks.
 */
TEST_F(TestService, on_new_request_callback) {
  auto server_callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {FAIL();};
  rmw_qos_profile_t service_qos = rmw_qos_profile_services_default;
  service_qos.depth = 3;
  auto server = node->create_service<test_msgs::srv::Empty>(
    "~/test_service", server_callback, service_qos);

  std::atomic<size_t> c1 {0};
  auto increase_c1_cb = [&c1](size_t count_msgs) {c1 += count_msgs;};
  server->set_on_new_request_callback(increase_c1_cb);

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
  server->set_on_new_request_callback(increase_c2_cb);

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

  server->clear_on_new_request_callback();

  {
    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    client->async_send_request(request);
    client->async_send_request(request);
    client->async_send_request(request);
  }

  std::atomic<size_t> c3 {0};
  auto increase_c3_cb = [&c3](size_t count_msgs) {c3 += count_msgs;};
  server->set_on_new_request_callback(increase_c3_cb);

  start = std::chrono::steady_clock::now();
  do {
    std::this_thread::sleep_for(100ms);
  } while (c3 < 3 && std::chrono::steady_clock::now() - start < 10s);

  EXPECT_EQ(c1.load(), 1u);
  EXPECT_EQ(c2.load(), 1u);
  EXPECT_EQ(c3.load(), 3u);

  std::function<void(size_t)> invalid_cb = nullptr;
  EXPECT_THROW(server->set_on_new_request_callback(invalid_cb), std::invalid_argument);
}

TEST_F(TestService, rcl_service_response_publisher_get_actual_qos_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_service_response_publisher_get_actual_qos, nullptr);
  auto callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
  auto server = node->create_service<test_msgs::srv::Empty>("service", callback);
  RCLCPP_EXPECT_THROW_EQ(
    server->get_response_publisher_actual_qos(),
    std::runtime_error("failed to get service's response publisher qos settings: error not set"));
}

TEST_F(TestService, rcl_service_request_subscription_get_actual_qos_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_service_request_subscription_get_actual_qos, nullptr);
  auto callback =
    [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
  auto server = node->create_service<test_msgs::srv::Empty>("service", callback);
  RCLCPP_EXPECT_THROW_EQ(
    server->get_request_subscription_actual_qos(),
    std::runtime_error("failed to get service's request subscription qos settings: error not set"));
}


TEST_F(TestService, server_qos) {
  rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
  qos_profile.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  uint64_t duration = 1;
  qos_profile.deadline = {duration, duration};
  qos_profile.lifespan = {duration, duration};
  qos_profile.liveliness_lease_duration = {duration, duration};

  auto callback = [](const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};

  auto server = node->create_service<test_msgs::srv::Empty>(
    "service", callback,
    qos_profile);
  auto init_qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
  auto rs_qos = server->get_request_subscription_actual_qos();
  auto rp_qos = server->get_response_publisher_actual_qos();

  EXPECT_EQ(init_qos, rp_qos);
  // Lifespan has no meaning for subscription/readers
  rs_qos.lifespan(qos_profile.lifespan);
  EXPECT_EQ(init_qos, rs_qos);
}

TEST_F(TestService, server_qos_depth) {
  using namespace std::literals::chrono_literals;

  uint64_t server_cb_count_ = 0;
  auto server_callback = [&](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {server_cb_count_++;};

  auto server_node = std::make_shared<rclcpp::Node>("server_node", "/ns");

  rmw_qos_profile_t server_qos_profile = rmw_qos_profile_default;
  server_qos_profile.depth = 2;

  auto server = server_node->create_service<test_msgs::srv::Empty>(
    "test_qos_depth", std::move(server_callback), server_qos_profile);

  rmw_qos_profile_t client_qos_profile = rmw_qos_profile_default;
  auto client = node->create_client<test_msgs::srv::Empty>("test_qos_depth", client_qos_profile);

  ::testing::AssertionResult request_result = ::testing::AssertionSuccess();
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();

  using SharedFuture = rclcpp::Client<test_msgs::srv::Empty>::SharedFuture;
  auto client_callback = [&request_result](SharedFuture future_response) {
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
  while ((server_cb_count_ < server_qos_profile.depth) &&
    (std::chrono::steady_clock::now() - start) < 1s)
  {
    rclcpp::spin_some(server_node);
    std::this_thread::sleep_for(1ms);
  }

  // Spin an extra time to check if server QoS depth has been ignored,
  // so more server responses might be processed than expected.
  rclcpp::spin_some(server_node);

  EXPECT_EQ(server_cb_count_, server_qos_profile.depth);
}
