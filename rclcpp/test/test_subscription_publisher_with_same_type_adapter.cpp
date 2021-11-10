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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "rclcpp/msg/string.hpp"


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif


using namespace std::chrono_literals;

static const int g_max_loops = 200;
static const std::chrono::milliseconds g_sleep_per_loop(10);


class TestSubscriptionPublisher : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class CLASSNAME (test_intra_process_within_one_node, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
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
    throw std::runtime_error("This should not happen");
  }
};

}  // namespace rclcpp

void wait_for_message_to_be_received(
  bool & is_received,
  const std::shared_ptr<rclcpp::Node> & node)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_once(std::chrono::milliseconds(0));
  int i = 0;
  while (!is_received && i < g_max_loops) {
    printf("spin_node_once() - callback (1) expected - try %d/%d\n", ++i, g_max_loops);
    executor.spin_once(g_sleep_per_loop);
  }
}

/*
 * Testing that subscriber receives type adapted types and ROS message types with intra proccess communications.
 */
TEST_F(
  CLASSNAME(test_intra_process_within_one_node, RMW_IMPLEMENTATION),
  check_type_adapted_messages_are_received_by_intra_process_subscription) {
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  RCLCPP_WARN(rclcpp::get_node_logger(nullptr), "BLASSS");

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);

  {
    { // std::unique_ptr<std::string>
      bool is_received = false;
      auto callback =
        [message_data, &is_received](
        std::unique_ptr<std::string> msg) -> void {
          is_received = true;
          ASSERT_STREQ(message_data.c_str(), (*msg).c_str());
        };
      auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);


      RCLCPP_WARN(rclcpp::get_node_logger(nullptr), "BLASSS");

      auto pu_message = std::make_unique<std::string>(message_data);
      pub->publish(std::move(pu_message));

      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received(is_received, node);
      ASSERT_TRUE(is_received);
    }
    { // std::unique_ptr<std::string> with message info
      bool is_received = false;
      auto callback =
        [message_data, &is_received](
        std::unique_ptr<std::string> msg,
        const rclcpp::MessageInfo & message_info) -> void {
          is_received = true;
          ASSERT_STREQ(message_data.c_str(), (*msg).c_str());
          ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
        };
      auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

      auto pu_message = std::make_unique<std::string>(message_data);
      pub->publish(std::move(pu_message));

      ASSERT_FALSE(is_received);
      wait_for_message_to_be_received(is_received, node);
      ASSERT_TRUE(is_received);
    }
  }
}
