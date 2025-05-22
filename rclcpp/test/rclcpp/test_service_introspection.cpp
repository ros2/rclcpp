// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <rcl/service_introspection.h>
#include <rmw/rmw.h>

#include <chrono>
#include <map>
#include <string>

#include "gmock/gmock.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/srv/basic_types.hpp"
#include "service_msgs/msg/service_event_info.hpp"

using namespace std::chrono_literals;
using test_msgs::srv::BasicTypes;
using service_msgs::msg::ServiceEventInfo;


class TestServiceIntrospection : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>(
      "my_node", "/ns");

    auto srv_callback =
      [](const BasicTypes::Request::SharedPtr & req, const BasicTypes::Response::SharedPtr & resp) {
        resp->set__bool_value(!req->bool_value);
        resp->set__int64_value(req->int64_value);
        return resp;
      };

    auto callback = [this](const std::shared_ptr<const BasicTypes::Event> & msg) {
        events.push_back(msg);
        (void)msg;
      };

    client = node->create_client<BasicTypes>("service");
    service = node->create_service<BasicTypes>("service", srv_callback);
    sub = node->create_subscription<BasicTypes::Event>("service/_service_event", 10, callback);
    events.clear();
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Client<BasicTypes>::SharedPtr client;
  rclcpp::Service<BasicTypes>::SharedPtr service;
  rclcpp::Subscription<BasicTypes::Event>::SharedPtr sub;
  std::vector<std::shared_ptr<const BasicTypes::Event>> events;
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
};

TEST_F(TestServiceIntrospection, service_introspection_nominal)
{
  auto request = std::make_shared<BasicTypes::Request>();
  request->set__bool_value(true);
  request->set__int64_value(42);

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Wait for the introspection to attach to our subscription
  size_t tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));

  BasicTypes::Response::SharedPtr response = future.get();
  ASSERT_EQ(response->bool_value, false);
  ASSERT_EQ(response->int64_value, 42);

  // wrap up work to get all the service_event messages
  auto start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }

  std::map<uint8_t, std::shared_ptr<const BasicTypes::Event>> event_map;
  for (auto & event : events) {
    event_map[event->info.event_type] = event;
  }
  ASSERT_EQ(event_map.size(), 4U);

  rmw_gid_t client_gid;
  rmw_get_gid_for_client(rcl_client_get_rmw_handle(client->get_client_handle().get()), &client_gid);
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> client_gid_arr;
  std::move(std::begin(client_gid.data), std::end(client_gid.data), client_gid_arr.begin());
  ASSERT_THAT(
    client_gid_arr,
    testing::Eq(event_map[ServiceEventInfo::REQUEST_SENT]->info.client_gid));

  ASSERT_EQ(
    event_map[ServiceEventInfo::REQUEST_SENT]->info.sequence_number,
    event_map[ServiceEventInfo::REQUEST_RECEIVED]->info.sequence_number);
  ASSERT_EQ(
    event_map[ServiceEventInfo::RESPONSE_SENT]->info.sequence_number,
    event_map[ServiceEventInfo::RESPONSE_RECEIVED]->info.sequence_number);
  ASSERT_EQ(
    event_map[ServiceEventInfo::REQUEST_SENT]->info.sequence_number,
    event_map[ServiceEventInfo::RESPONSE_SENT]->info.sequence_number);
  ASSERT_EQ(
    event_map[ServiceEventInfo::REQUEST_RECEIVED]->info.sequence_number,
    event_map[ServiceEventInfo::RESPONSE_RECEIVED]->info.sequence_number);

  ASSERT_EQ(event_map[ServiceEventInfo::REQUEST_SENT]->request[0].int64_value, 42);
  ASSERT_EQ(event_map[ServiceEventInfo::REQUEST_SENT]->request[0].bool_value, true);
  ASSERT_EQ(event_map[ServiceEventInfo::RESPONSE_SENT]->response[0].int64_value, 42);
  ASSERT_EQ(event_map[ServiceEventInfo::RESPONSE_SENT]->response[0].bool_value, false);
  ASSERT_EQ(event_map[ServiceEventInfo::RESPONSE_SENT]->request.size(), 0U);
  ASSERT_EQ(event_map[ServiceEventInfo::REQUEST_RECEIVED]->response.size(), 0U);
}

TEST_F(TestServiceIntrospection, service_introspection_enable_disable_events)
{
  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF);

  ASSERT_EQ(sub->get_publisher_count(), 0);

  auto request = std::make_shared<BasicTypes::Request>();
  request->set__bool_value(true);
  request->set__int64_value(42);
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 0U);

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF);

  // Wait for the introspection to attach to our subscription
  size_t tries = 1000;
  while (this->sub->get_publisher_count() < 1 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 1u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 2 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 2U);

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Wait for the introspection to attach to our subscription
  tries = 1000;
  while (this->sub->get_publisher_count() < 1 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 1u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 2 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 2U);

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Wait for the introspection to attach to our subscription
  tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 4U);
}

TEST_F(TestServiceIntrospection, service_introspection_enable_disable_event_content)
{
  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Wait for the introspection to attach to our subscription
  size_t tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  auto request = std::make_shared<BasicTypes::Request>();
  request->set__bool_value(true);
  request->set__int64_value(42);
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  auto start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 4U);
  for (const auto & event : events) {
    EXPECT_EQ(event->request.size(), 0U);
    EXPECT_EQ(event->response.size(), 0U);
  }

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);

  // Wait for the introspection to attach to our subscription
  tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 4U);
  for (const auto & event : events) {
    switch (event->info.event_type) {
      case ServiceEventInfo::REQUEST_SENT:
        EXPECT_EQ(event->request.size(), 1U);
        break;
      case ServiceEventInfo::REQUEST_RECEIVED:
        EXPECT_EQ(event->request.size(), 0U);
        break;
      case ServiceEventInfo::RESPONSE_SENT:
        EXPECT_EQ(event->response.size(), 0U);
        break;
      case ServiceEventInfo::RESPONSE_RECEIVED:
        EXPECT_EQ(event->response.size(), 1U);
        break;
    }
  }

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Wait for the introspection to attach to our subscription
  tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 4U);
  for (const auto & event : events) {
    switch (event->info.event_type) {
      case ServiceEventInfo::REQUEST_SENT:
        EXPECT_EQ(event->request.size(), 0U);
        break;
      case ServiceEventInfo::REQUEST_RECEIVED:
        EXPECT_EQ(event->request.size(), 1U);
        break;
      case ServiceEventInfo::RESPONSE_SENT:
        EXPECT_EQ(event->response.size(), 1U);
        break;
      case ServiceEventInfo::RESPONSE_RECEIVED:
        EXPECT_EQ(event->response.size(), 0U);
        break;
    }
  }

  events.clear();

  client->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
  service->configure_introspection(
    node->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

  // Wait for the introspection to attach to our subscription
  tries = 1000;
  while (this->sub->get_publisher_count() < 2 && tries-- > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(sub->get_publisher_count(), 2u);

  future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future, timeout));
  start = std::chrono::steady_clock::now();
  while (events.size() < 4 && (std::chrono::steady_clock::now() - start) < timeout) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(events.size(), 4U);
  for (const auto & event : events) {
    switch (event->info.event_type) {
      case ServiceEventInfo::REQUEST_SENT:
      case ServiceEventInfo::REQUEST_RECEIVED:
        EXPECT_EQ(event->request.size(), 1U);
        break;
      case ServiceEventInfo::RESPONSE_SENT:
      case ServiceEventInfo::RESPONSE_RECEIVED:
        EXPECT_EQ(event->response.size(), 1U);
        break;
    }
  }
}
