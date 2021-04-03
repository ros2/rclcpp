// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#include <gmock/gmock.h>

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"

#include "rcl/graph.h"

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace ::testing;  // NOLINT
using namespace rclcpp;  // NOLINT

class RclcppGenericNodeFixture : public Test
{
public:
  RclcppGenericNodeFixture()
  {
    node_ = std::make_shared<rclcpp::Node>("pubsub");
    publisher_node_ = std::make_shared<rclcpp::Node>(
      "publisher_node",
      rclcpp::NodeOptions().start_parameter_event_publisher(false));
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void create_publisher(const std::string & topic)
  {
    auto publisher = publisher_node_->create_publisher<test_msgs::msg::Strings>(topic, 10);
    publishers_.push_back(publisher);
  }

  std::vector<std::string> subscribe_raw_messages(
    size_t expected_messages_number, const std::string & topic_name, const std::string & type)
  {
    std::vector<std::string> messages;
    size_t counter = 0;
    auto subscription = node_->create_generic_subscription(
      topic_name, type, rclcpp::QoS(1),
      [&counter, &messages](std::shared_ptr<rclcpp::SerializedMessage> message) {
        test_msgs::msg::Strings string_message;
        rclcpp::Serialization<test_msgs::msg::Strings> serializer;
        serializer.deserialize_message(message.get(), &string_message);
        messages.push_back(string_message.string_value);
        counter++;
      });

    while (counter < expected_messages_number) {
      rclcpp::spin_some(node_);
    }
    return messages;
  }

  rclcpp::SerializedMessage serialize_string_message(const std::string & message)
  {
    test_msgs::msg::Strings string_message;
    string_message.string_value = message;
    rclcpp::Serialization<test_msgs::msg::Strings> ser;
    SerializedMessage result;
    ser.serialize_message(&string_message, &result);
    return result;
  }

  void sleep_to_allow_topics_discovery()
  {
    // This is a short sleep to allow the node some time to discover the topic
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  template<typename Condition, typename Duration>
  bool wait_for(const Condition & condition, const Duration & timeout)
  {
    using clock = std::chrono::system_clock;
    auto start = clock::now();
    while (!condition()) {
      if ((clock::now() - start) > timeout) {
        return false;
      }
      rclcpp::spin_some(node_);
    }
    return true;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Node::SharedPtr publisher_node_;
  std::vector<std::shared_ptr<rclcpp::PublisherBase>> publishers_;
};


TEST_F(RclcppGenericNodeFixture, publisher_and_subscriber_work)
{
  // We currently publish more messages because they can get lost
  std::vector<std::string> test_messages = {"Hello World", "Hello World"};
  std::string topic_name = "/string_topic";
  std::string type = "test_msgs/msg/Strings";

  auto publisher = node_->create_generic_publisher(
    topic_name, type, rclcpp::QoS(1));

  auto subscriber_future_ = std::async(
    std::launch::async, [this, topic_name, type] {
      return subscribe_raw_messages(1, topic_name, type);
    });

  // TODO(karsten1987): Port 'wait_for_sub' to rclcpp
  auto allocator = node_->get_node_options().allocator();
  auto success = false;
  auto ret = rcl_wait_for_subscribers(
    node_->get_node_base_interface()->get_rcl_node_handle(),
    &allocator,
    topic_name.c_str(),
    1u,
    static_cast<rcutils_duration_value_t>(1e9),
    &success);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ASSERT_TRUE(success);

  for (const auto & message : test_messages) {
    publisher->publish(serialize_string_message(message));
  }

  auto subscribed_messages = subscriber_future_.get();
  EXPECT_THAT(subscribed_messages, SizeIs(Not(0)));
  EXPECT_THAT(subscribed_messages[0], StrEq("Hello World"));
}

TEST_F(RclcppGenericNodeFixture, generic_subscription_uses_qos)
{
  // If the GenericSubscription does not use the provided QoS profile,
  // its request will be incompatible with the Publisher's offer and no messages will be passed.
  using namespace std::chrono_literals;
  std::string topic_name = "string_topic";
  std::string topic_type = "test_msgs/msg/Strings";
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  auto publisher = node_->create_publisher<test_msgs::msg::Strings>(topic_name, qos);
  auto subscription = node_->create_generic_subscription(
    topic_name, topic_type, qos,
    [](std::shared_ptr<rclcpp::SerializedMessage>/* message */) {});
  auto connected = [publisher, subscription]() -> bool {
      return publisher->get_subscription_count() && subscription->get_publisher_count();
    };
  // It normally takes < 20ms, 5s chosen as "a very long time"
  ASSERT_TRUE(wait_for(connected, 5s));
}

TEST_F(RclcppGenericNodeFixture, generic_publisher_uses_qos)
{
  // If the GenericPublisher does not use the provided QoS profile,
  // its offer will be incompatible with the Subscription's request and no messages will be passed.
  using namespace std::chrono_literals;
  std::string topic_name = "string_topic";
  std::string topic_type = "test_msgs/msg/Strings";
  rclcpp::QoS qos = rclcpp::QoS(1).transient_local();

  auto publisher = node_->create_generic_publisher(topic_name, topic_type, qos);
  auto subscription = node_->create_subscription<test_msgs::msg::Strings>(
    topic_name, qos,
    [](std::shared_ptr<test_msgs::msg::Strings>/* message */) {});
  auto connected = [publisher, subscription]() -> bool {
      return publisher->get_subscription_count() && subscription->get_publisher_count();
    };
  // It normally takes < 20ms, 5s chosen as "a very long time"
  ASSERT_TRUE(wait_for(connected, 5s));
}
