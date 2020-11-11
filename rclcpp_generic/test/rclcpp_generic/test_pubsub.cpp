// Copyright 2018, Bosch Software Innovations GmbH.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "memory_management.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"

#include "rclcpp_generic/generic_publisher.hpp"
#include "rclcpp_generic/generic_subscription.hpp"

using namespace ::testing;  // NOLINT
using namespace rclcpp_generic;  // NOLINT

class RosBag2NodeFixture : public Test
{
public:
  RosBag2NodeFixture()
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
    auto subscription = GenericSubscription::create(
      node_->get_node_topics_interface(),
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

  std::shared_ptr<rmw_serialized_message_t> serialize_string_message(const std::string & message)
  {
    auto string_message = std::make_shared<test_msgs::msg::Strings>();
    string_message->string_value = message;
    return memory_management_.serialize_message(string_message);
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

  MemoryManagement memory_management_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Node::SharedPtr publisher_node_;
  std::vector<std::shared_ptr<rclcpp::PublisherBase>> publishers_;
};


TEST_F(RosBag2NodeFixture, publisher_and_subscriber_work)
{
  // We currently publish more messages because they can get lost
  std::vector<std::string> test_messages = {"Hello World", "Hello World"};
  std::string topic_name = "string_topic";
  std::string type = "test_msgs/Strings";

  auto publisher = GenericPublisher::create(
    node_->get_node_topics_interface(),
    topic_name, type, rclcpp::QoS(1));

  auto subscriber_future_ = std::async(
    std::launch::async, [this, topic_name, type] {
      return subscribe_raw_messages(1, topic_name, type);
    });
  // Give time to the subscriber to start.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (const auto & message : test_messages) {
    publisher->publish(serialize_string_message(message));
    // This is necessary because sometimes, the subscriber is initialized very late
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto subscribed_messages = subscriber_future_.get();
  EXPECT_THAT(subscribed_messages, SizeIs(Not(0)));
  EXPECT_THAT(subscribed_messages[0], StrEq("Hello World"));
}

TEST_F(RosBag2NodeFixture, generic_subscription_uses_qos)
{
  // If the GenericSubscription does not use the provided QoS profile,
  // its request will be incompatible with the Publisher's offer and no messages will be passed.
  using namespace std::chrono_literals;
  std::string topic_name = "string_topic";
  std::string topic_type = "test_msgs/Strings";
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  auto publisher = node_->create_publisher<test_msgs::msg::Strings>(topic_name, qos);
  auto subscription = GenericSubscription::create(
    node_->get_node_topics_interface(),
    topic_name, topic_type, qos,
    [](std::shared_ptr<rclcpp::SerializedMessage>/* message */) {});
  auto connected = [publisher, subscription]() -> bool {
      return publisher->get_subscription_count() && subscription->get_publisher_count();
    };
  // It normally takes < 20ms, 5s chosen as "a very long time"
  ASSERT_TRUE(wait_for(connected, 5s));
}

TEST_F(RosBag2NodeFixture, generic_publisher_uses_qos)
{
  // If the GenericPublisher does not use the provided QoS profile,
  // its offer will be incompatible with the Subscription's request and no messages will be passed.
  using namespace std::chrono_literals;
  std::string topic_name = "string_topic";
  std::string topic_type = "test_msgs/Strings";
  rclcpp::QoS qos = rclcpp::QoS(1).transient_local();

  auto publisher = GenericPublisher::create(
    node_->get_node_topics_interface(), topic_name, topic_type, qos);
  auto subscription = node_->create_subscription<test_msgs::msg::Strings>(
    topic_name, qos,
    [](std::shared_ptr<test_msgs::msg::Strings>/* message */) {});
  auto connected = [publisher, subscription]() -> bool {
      return publisher->get_subscription_count() && subscription->get_publisher_count();
    };
  // It normally takes < 20ms, 5s chosen as "a very long time"
  ASSERT_TRUE(wait_for(connected, 5s));
}
