// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/msg/large_message.hpp"
#include "rclcpp/msg/string.hpp"


using namespace std::chrono_literals;

static const int g_max_loops = 200;
static const std::chrono::milliseconds g_sleep_per_loop(10);


class TestPublisher : public ::testing::Test
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

template<>
struct TypeAdapter<std::string, rclcpp::msg::LargeMessage>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = rclcpp::msg::LargeMessage;

  static void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.size = source.size();
    std::memcpy(destination.data.data(), source.data(), source.size());
  }

  static void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.resize(source.size);
    std::memcpy(destination.data(), source.data.data(), source.size);
  }
};

}  // namespace rclcpp

/*
 * Testing publisher creation signatures with a type adapter.
 */
TEST_F(TestPublisher, various_creation_signatures) {
  for (auto is_intra_process : {true, false}) {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(is_intra_process);
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", options);
    {
      using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
      auto publisher = node->create_publisher<StringTypeAdapter>("topic", 42);
      (void)publisher;
    }
    {
      using StringTypeAdapter = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;
      auto publisher = node->create_publisher<StringTypeAdapter>("topic", 42);
      (void)publisher;
    }
  }
}

/*
 * Testing that conversion errors are passed up.
 */
TEST_F(TestPublisher, conversion_exception_is_passed_up) {
  using BadStringTypeAdapter = rclcpp::TypeAdapter<int, rclcpp::msg::String>;
  for (auto is_intra_process : {true, false}) {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(is_intra_process);

    auto callback =
      [](const rclcpp::msg::String::ConstSharedPtr msg) -> void
      {
        (void)msg;
      };

    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", options);
    auto pub = node->create_publisher<BadStringTypeAdapter>("topic_name", 1);
    // A subscription is created to ensure the existence of a buffer in the intra proccess
    // manager which will trigger the faulty conversion.
    auto sub = node->create_subscription<rclcpp::msg::String>("topic_name", 1, callback);
    EXPECT_THROW(pub->publish(1), std::runtime_error);
  }
}

using UseTakeSharedMethod = bool;
class TestPublisherFixture
  : public TestPublisher,
  public ::testing::WithParamInterface<UseTakeSharedMethod>
{
};

/*
 * Testing that publisher sends type adapted types and ROS message types with intra proccess communications.
 */
TEST_P(
  TestPublisherFixture,
  check_type_adapted_message_is_sent_and_received_intra_process) {
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";
  bool is_received;

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 10);
  rclcpp::Subscription<rclcpp::msg::String>::SharedPtr sub;
  if (GetParam()) {
    auto callback =
      [message_data, &is_received](
      const rclcpp::msg::String::ConstSharedPtr msg,
      const rclcpp::MessageInfo & message_info
      ) -> void
      {
        is_received = true;
        ASSERT_STREQ(message_data.c_str(), msg->data.c_str());
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    sub = node->create_subscription<rclcpp::msg::String>(topic_name, 1, callback);
  } else {
    auto callback_unique =
      [message_data, &is_received](
      rclcpp::msg::String::UniquePtr msg,
      const rclcpp::MessageInfo & message_info
      ) -> void
      {
        is_received = true;
        ASSERT_STREQ(message_data.c_str(), msg->data.c_str());
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    sub = node->create_subscription<rclcpp::msg::String>(topic_name, 1, callback_unique);
  }

  auto wait_for_message_to_be_received = [&is_received, &node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      int i = 0;
      executor.add_node(node);
      executor.spin_once(std::chrono::milliseconds(0));
      while (!is_received && i < g_max_loops) {
        printf("spin_node_once() - callback (1) expected - try %d/%d\n", ++i, g_max_loops);
        executor.spin_once(g_sleep_per_loop);
      }
    };
  {
    { // std::string passed by reference
      is_received = false;
      pub->publish(message_data);
      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received();
      ASSERT_TRUE(is_received);
    }
    { // unique pointer to std::string
      is_received = false;
      auto pu_message = std::make_unique<std::string>(message_data);
      pub->publish(std::move(pu_message));
      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received();
      ASSERT_TRUE(is_received);
    }
    { // ROS message passed by reference
      is_received = false;
      rclcpp::msg::String msg;
      msg.data = message_data;
      pub->publish(msg);
      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received();
      ASSERT_TRUE(is_received);
    }
    { // unique ptr to ROS message
      is_received = false;
      auto pu_msg = std::make_unique<rclcpp::msg::String>();
      pu_msg->data = message_data;
      pub->publish(std::move(pu_msg));
      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received();
      ASSERT_TRUE(is_received);
    }
    /* TODO(audrow) Enable once loaned messages are supported for intra process communication
    { // loaned ROS message
      // is_received = false;
      // std::allocator<void> allocator;
      // rclcpp::LoanedMessage<rclcpp::msg::String> loaned_msg(*pub, allocator);
      // loaned_msg.get().data = message_data;
      // pub->publish(std::move(loaned_msg));
      // ASSERT_FALSE(is_received);
      // wait_for_message_to_be_received();
      // ASSERT_TRUE(is_received);
    }
    */
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestPublisherFixtureWithParam,
  TestPublisherFixture,
  ::testing::Values(
    true,   // use take shared method
    false   // not use take shared method
));

/*
 * Testing that publisher sends type adapted types and ROS message types with inter proccess communications.
 */
TEST_F(TestPublisher, check_type_adapted_message_is_sent_and_received) {
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", rclcpp::NodeOptions());

  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  auto do_nothing = [](std::shared_ptr<const rclcpp::msg::String>) {FAIL();};
  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);
  auto sub = node->create_subscription<rclcpp::msg::String>(topic_name, 1, do_nothing);

  auto assert_no_message_was_received_yet = [sub]() {
      rclcpp::msg::String msg;
      rclcpp::MessageInfo msg_info;
      EXPECT_FALSE(sub->take(msg, msg_info));
    };
  auto assert_message_was_received = [sub, message_data]() {
      rclcpp::msg::String msg;
      rclcpp::MessageInfo msg_info;
      bool message_received = false;
      auto start = std::chrono::steady_clock::now();
      do {
        message_received = sub->take(msg, msg_info);
        std::this_thread::sleep_for(100ms);
      } while (!message_received && std::chrono::steady_clock::now() - start < 10s);
      EXPECT_TRUE(message_received);
      ASSERT_STREQ(message_data.c_str(), msg.data.c_str());
    };

  { // std::string passed by reference
    assert_no_message_was_received_yet();
    pub->publish(message_data);
    assert_message_was_received();
  }
  { // unique pointer to std::string
    assert_no_message_was_received_yet();
    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));
    assert_message_was_received();
  }
  { // ROS message passed by reference
    assert_no_message_was_received_yet();
    rclcpp::msg::String msg;
    msg.data = message_data;
    pub->publish(msg);
    assert_message_was_received();
  }
  { // unique ptr to ROS message
    assert_no_message_was_received_yet();
    auto pu_msg = std::make_unique<rclcpp::msg::String>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));
    assert_message_was_received();
  }
  { // loaned ROS message
    assert_no_message_was_received_yet();
    std::allocator<void> allocator;
    rclcpp::LoanedMessage<rclcpp::msg::String> loaned_msg(*pub, allocator);
    loaned_msg.get().data = message_data;
    pub->publish(std::move(loaned_msg));
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    assert_message_was_received();
  }
}

TEST_F(TestPublisher, test_large_message_unique)
{
  // There have been some bugs in the past when trying to type-adapt large messages
  // (larger than the stack size).  Here we just make sure that a 10MB message works,
  // which is larger than the default stack size on Linux.

  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::LargeMessage>;

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", rclcpp::NodeOptions());

  const std::string topic_name = "topic_name";

  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);

  static constexpr size_t length = 10 * 1024 * 1024;
  auto message_data = std::make_unique<std::string>(length, '#');
  pub->publish(std::move(message_data));
}

TEST_F(TestPublisher, test_large_message_constref)
{
  // There have been some bugs in the past when trying to type-adapt large messages
  // (larger than the stack size).  Here we just make sure that a 10MB message works,
  // which is larger than the default stack size on Linux.

  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::LargeMessage>;

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", rclcpp::NodeOptions());

  const std::string topic_name = "topic_name";

  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);

  static constexpr size_t length = 10 * 1024 * 1024;
  std::string message_data(length, '#');
  pub->publish(message_data);
}
