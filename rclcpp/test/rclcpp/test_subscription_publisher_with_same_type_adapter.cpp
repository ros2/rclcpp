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
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/msg/string.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"

static const int g_max_loops = 200;
static const std::chrono::milliseconds g_sleep_per_loop(10);

class test_intra_process_within_one_node : public ::testing::Test
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

template<>
struct TypeAdapter<double, statistics_msgs::msg::StatisticDataPoint>
{
  using is_specialized = std::true_type;
  using custom_type = double;
  using ros_message_type = statistics_msgs::msg::StatisticDataPoint;

  static void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data_type = 0;
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
  test_intra_process_within_one_node,
  type_adapted_messages_unique_pub_received_by_intra_process_subscription)
{
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string &, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::string & msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), msg.c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string & with message info, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::string & msg, const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), msg.c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string>, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string> with message info, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string>, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string> with message info, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string>, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string> with message info, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> &, publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const std::string> & msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> & with message info,
    // publish with unique_ptr
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const std::string> & msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
}

TEST_F(
  test_intra_process_within_one_node,
  type_adapted_messages_const_ref_pub_received_by_intra_process_subscription)
{
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string &, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::string & msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), msg.c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string & with message info, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::string & msg, const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), msg.c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string>, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string> with message info, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string>, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string> with message info, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string>, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const std::string> msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string> with message info,
    // publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const std::string> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> &, publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const std::string> & msg) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> & with message info,
    // publish with const std::string &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const std::string> & msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        EXPECT_STREQ(message_data.c_str(), (*msg).c_str());
        EXPECT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<StringTypeAdapter>(topic_name, 1, callback);

    std::string pu_message(message_data);
    pub->publish(pu_message);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
}

TEST_F(
  test_intra_process_within_one_node,
  type_adapted_messages_ros_message_ref_pub_received_by_intra_process_subscription)
{
  using DoubleTypeAdapter = rclcpp::TypeAdapter<double, statistics_msgs::msg::StatisticDataPoint>;
  const double message_data = 0.894;
  const std::string topic_name = "topic_name";

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  auto pub = node->create_publisher<DoubleTypeAdapter>(topic_name, 1);

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string &, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const double & msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string & with message info, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const double & msg, const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string>, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string> with message info, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string>, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string> with message info, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string>, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string> with message info,
    // publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> &, publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const double> & msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> & with message info,
    // publish with rclcpp::msg::String &
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const double> & msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    statistics_msgs::msg::StatisticDataPoint msg;
    msg.data = message_data;
    pub->publish(msg);

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
}

TEST_F(
  test_intra_process_within_one_node,
  type_adapted_messages_unique_ptr_ros_message_pub_received_by_intra_process_subscription)
{
  using DoubleTypeAdapter = rclcpp::TypeAdapter<double, statistics_msgs::msg::StatisticDataPoint>;
  const double message_data = 0.7508;
  const std::string topic_name = "topic_name";

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  auto pub = node->create_publisher<DoubleTypeAdapter>(topic_name, 1);

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string &, publish with unique statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const double & msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::string & with message info, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const double & msg, const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string>, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::unique_ptr<std::string> with message info, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::unique_ptr<double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string>, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<std::string> with message info, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string>, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const double> msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback std::shared_ptr<const std::string> with message info, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      std::shared_ptr<const double> msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }

  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> &, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const double> & msg) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
  { // create_publisher with TypeAdapt struct, create_subscription with TypeAdapt struct,
    // callback const std::shared_ptr<const std::string> & with message info, publish with unique
    // statistics_msgs::msg::StatisticDataPoint
    bool is_received = false;
    auto callback =
      [message_data, &is_received](
      const std::shared_ptr<const double> & msg,
      const rclcpp::MessageInfo & message_info) -> void {
        is_received = true;
        ASSERT_EQ(message_data, *msg);
        ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
      };
    auto sub = node->create_subscription<DoubleTypeAdapter>(topic_name, 1, callback);

    auto pu_msg = std::make_unique<statistics_msgs::msg::StatisticDataPoint>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));

    ASSERT_FALSE(is_received);
    wait_for_message_to_be_received(is_received, node);
    ASSERT_TRUE(is_received);
  }
}
