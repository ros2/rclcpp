// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rmw/rmw.h"
#include "test_msgs/msg/empty.hpp"

#include "../mocking_utils/patch.hpp"

class TestQosEvent : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    is_fastrtps =
      std::string(rmw_get_implementation_identifier()).find("rmw_fastrtps") != std::string::npos;

    node = std::make_shared<rclcpp::Node>("test_qos_event", "/ns");

    message_callback = [node = node.get()](const test_msgs::msg::Empty::SharedPtr /*msg*/) {
        RCLCPP_INFO(node->get_logger(), "Message received");
      };
  }

  void TearDown()
  {
    node.reset();
  }

  static constexpr char topic_name[] = "test_topic";
  rclcpp::Node::SharedPtr node;
  bool is_fastrtps;
  std::function<void(const test_msgs::msg::Empty::SharedPtr)> message_callback;
};

constexpr char TestQosEvent::topic_name[];

/*
   Testing construction of a publishers with QoS event callback functions.
 */
TEST_F(TestQosEvent, test_publisher_constructor)
{
  rclcpp::PublisherOptions options;

  // options arg with no callbacks
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with one of the callbacks
  options.event_callbacks.deadline_callback =
    [node = node.get()](rclcpp::QOSDeadlineOfferedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Offered deadline missed - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with two of the callbacks
  options.event_callbacks.liveliness_callback =
    [node = node.get()](rclcpp::QOSLivelinessLostInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Liveliness lost - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with three of the callbacks
  options.event_callbacks.incompatible_qos_callback =
    [node = node.get()](rclcpp::QOSOfferedIncompatibleQoSInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Offered incompatible qos - total %d (delta %d), last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };
  try {
    publisher = node->create_publisher<test_msgs::msg::Empty>(
      topic_name, 10, options);
  } catch (const rclcpp::UnsupportedEventTypeException & /*exc*/) {
    EXPECT_TRUE(is_fastrtps);
  }
}

/*
   Testing construction of a subscriptions with QoS event callback functions.
 */
TEST_F(TestQosEvent, test_subscription_constructor)
{
  rclcpp::SubscriptionOptions options;

  // options arg with no callbacks
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with one of the callbacks
  options.event_callbacks.deadline_callback =
    [node = node.get()](rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Requested deadline missed - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with two of the callbacks
  options.event_callbacks.liveliness_callback =
    [node = node.get()](rclcpp::QOSLivelinessChangedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Liveliness changed - alive %d (delta %d), not alive %d (delta %d)",
        event.alive_count, event.alive_count_change,
        event.not_alive_count, event.not_alive_count_change);
    };
  subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with three of the callbacks
  options.event_callbacks.incompatible_qos_callback =
    [node = node.get()](rclcpp::QOSRequestedIncompatibleQoSInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Requested incompatible qos - total %d (delta %d), last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };
  try {
    subscription = node->create_subscription<test_msgs::msg::Empty>(
      topic_name, 10, message_callback, options);
  } catch (const rclcpp::UnsupportedEventTypeException & /*exc*/) {
    EXPECT_TRUE(is_fastrtps);
  }
}

/*
   Testing construction of a subscriptions with QoS event callback functions.
 */
std::string * g_pub_log_msg;
std::string * g_sub_log_msg;
std::promise<void> * g_log_msgs_promise;
TEST_F(TestQosEvent, test_default_incompatible_qos_callbacks)
{
  rcutils_logging_output_handler_t original_output_handler = rcutils_logging_get_output_handler();

  std::string pub_log_msg;
  std::string sub_log_msg;
  std::promise<void> log_msgs_promise;
  g_pub_log_msg = &pub_log_msg;
  g_sub_log_msg = &sub_log_msg;
  g_log_msgs_promise = &log_msgs_promise;
  auto logger_callback = [](
    const rcutils_log_location_t * /*location*/,
    int /*level*/, const char * /*name*/, rcutils_time_point_value_t /*timestamp*/,
    const char * format, va_list * args) -> void {
      char buffer[1024];
      vsnprintf(buffer, sizeof(buffer), format, *args);
      const std::string msg = buffer;
      if (msg.rfind("New subscription discovered", 0) == 0) {
        *g_pub_log_msg = buffer;
      } else if (msg.rfind("New publisher discovered", 0) == 0) {
        *g_sub_log_msg = buffer;
      }

      if (!g_pub_log_msg->empty() && !g_sub_log_msg->empty()) {
        g_log_msgs_promise->set_value();
      }
    };
  rcutils_logging_set_output_handler(logger_callback);

  std::shared_future<void> log_msgs_future = log_msgs_promise.get_future();

  rclcpp::QoS qos_profile_publisher(10);
  qos_profile_publisher.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, qos_profile_publisher);

  rclcpp::QoS qos_profile_subscription(10);
  qos_profile_subscription.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, qos_profile_subscription, message_callback);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node->get_node_base_interface());

  // This future won't complete on fastrtps, so just timeout immediately
  const auto timeout = (is_fastrtps) ? std::chrono::milliseconds(5) : std::chrono::seconds(10);
  ex.spin_until_future_complete(log_msgs_future, timeout);

  if (is_fastrtps) {
    EXPECT_EQ("", pub_log_msg);
    EXPECT_EQ("", sub_log_msg);
  } else {
    EXPECT_EQ(
      "New subscription discovered on this topic, requesting incompatible QoS. "
      "No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY",
      pub_log_msg);
    EXPECT_EQ(
      "New publisher discovered on this topic, offering incompatible QoS. "
      "No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY",
      sub_log_msg);
  }

  rcutils_logging_set_output_handler(original_output_handler);
}

TEST_F(TestQosEvent, construct_destruct_rcl_error) {
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(topic_name, 10);
  auto rcl_handle = publisher->get_publisher_handle();
  ASSERT_NE(nullptr, rcl_handle);

  // This callback requires some type of parameter, but it could be anything
  auto callback = [](int) {};
  const rcl_publisher_event_type_t event_type = RCL_PUBLISHER_OFFERED_DEADLINE_MISSED;

  {
    // Logs error and returns
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_publisher_event_init, RCL_RET_ERROR);

    auto throwing_statement = [callback, rcl_handle, event_type]() {
        // reset() is not needed for the exception, but it handles unused return value warning
        std::make_shared<
          rclcpp::QOSEventHandler<decltype(callback), std::shared_ptr<rcl_publisher_t>>>(
          callback, rcl_publisher_event_init, rcl_handle, event_type).reset();
      };
    // This is done through a lambda because the compiler is having trouble parsing the templated
    // function inside a macro.
    EXPECT_THROW(throwing_statement(), rclcpp::exceptions::RCLError);
  }

  {
    // Logs error and returns
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_event_fini, RCL_RET_ERROR);

    auto throwing_statement = [callback, rcl_handle, event_type]() {
        // reset() is needed for this exception
        std::make_shared<
          rclcpp::QOSEventHandler<decltype(callback), std::shared_ptr<rcl_publisher_t>>>(
          callback, rcl_publisher_event_init, rcl_handle, event_type).reset();
      };

    // This is done through a lambda because the compiler is having trouble parsing the templated
    // function inside a macro.
    EXPECT_NO_THROW(throwing_statement());
  }
}

TEST_F(TestQosEvent, execute) {
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(topic_name, 10);
  auto rcl_handle = publisher->get_publisher_handle();

  bool handler_callback_executed = false;
  // This callback requires some type of parameter, but it could be anything
  auto callback = [&handler_callback_executed](int) {handler_callback_executed = true;};
  rcl_publisher_event_type_t event_type = RCL_PUBLISHER_OFFERED_DEADLINE_MISSED;

  rclcpp::QOSEventHandler<decltype(callback), decltype(rcl_handle)> handler(
    callback, rcl_publisher_event_init, rcl_handle, event_type);

  EXPECT_NO_THROW(handler.execute());
  EXPECT_TRUE(handler_callback_executed);

  {
    handler_callback_executed = false;
    // Logs error and returns early
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_take_event, RCL_RET_ERROR);
    EXPECT_NO_THROW(handler.execute());
    EXPECT_FALSE(handler_callback_executed);
  }
}

TEST_F(TestQosEvent, add_to_wait_set) {
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(topic_name, 10);
  auto rcl_handle = publisher->get_publisher_handle();

  // This callback requires some type of parameter, but it could be anything
  auto callback = [](int) {};

  rcl_publisher_event_type_t event_type = RCL_PUBLISHER_OFFERED_DEADLINE_MISSED;
  rclcpp::QOSEventHandler<decltype(callback), decltype(rcl_handle)> handler(
    callback, rcl_publisher_event_init, rcl_handle, event_type);

  EXPECT_EQ(1u, handler.get_number_of_ready_events());

  {
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_add_event, RCL_RET_OK);
    EXPECT_TRUE(handler.add_to_wait_set(&wait_set));
  }

  {
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_add_event, RCL_RET_ERROR);
    EXPECT_THROW(handler.add_to_wait_set(&wait_set), rclcpp::exceptions::RCLError);
  }
}
