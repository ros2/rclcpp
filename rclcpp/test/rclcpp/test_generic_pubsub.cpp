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

  template<typename T1, typename T2>
  std::vector<T1> subscribe_raw_messages(
    size_t expected_recv_msg_count, const std::string & topic_name, const std::string & type)
  {
    std::vector<T1> messages;
    size_t counter = 0;
    auto subscription = node_->create_generic_subscription(
      topic_name, type, rclcpp::QoS(1),
      [&counter, &messages, this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        T2 deserialized_message;
        rclcpp::Serialization<T2> serializer;
        serializer.deserialize_message(message.get(), &deserialized_message);
        messages.push_back(this->get_data_from_msg(deserialized_message));
        counter++;
      });

    while (counter < expected_recv_msg_count) {
      rclcpp::spin_some(node_);
    }
    return messages;
  }

  template<typename T1, typename T2>
  rclcpp::SerializedMessage serialize_message(const T1 & data)
  {
    T2 message;
    write_message(data, message);

    rclcpp::Serialization<T2> ser;
    SerializedMessage result;
    ser.serialize_message(&message, &result);
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

private:
  void write_message(const std::string & data, test_msgs::msg::Strings & message)
  {
    message.string_value = data;
  }

  void write_message(const int64_t & data, test_msgs::msg::BasicTypes & message)
  {
    message.int64_value = data;
  }

  std::string get_data_from_msg(const test_msgs::msg::Strings & message)
  {
    return message.string_value;
  }

  int64_t get_data_from_msg(const test_msgs::msg::BasicTypes & message)
  {
    return message.int64_value;
  }
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
      return subscribe_raw_messages<std::string, test_msgs::msg::Strings>(1, topic_name, type);
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
    publisher->publish(serialize_message<std::string, test_msgs::msg::Strings>(message));
  }

  auto subscribed_messages = subscriber_future_.get();
  EXPECT_THAT(subscribed_messages, SizeIs(Not(0)));
  EXPECT_THAT(subscribed_messages[0], StrEq("Hello World"));
}

TEST_F(RclcppGenericNodeFixture, publish_loaned_msg_work)
{
  // We currently publish more messages because they can get lost
  std::vector<int64_t> test_messages = {100, 100};
  std::string topic_name = "/int64_topic";
  std::string type = "test_msgs/msg/BasicTypes";

  auto publisher = node_->create_generic_publisher(topic_name, type, rclcpp::QoS(1));

  if (publisher->can_loan_messages()) {
    auto subscriber_future_ = std::async(
      std::launch::deferred, [this, topic_name, type] {
        return subscribe_raw_messages<int64_t, test_msgs::msg::BasicTypes>(
          1, topic_name, type);
      });

    auto allocator = node_->get_node_options().allocator();
    auto success = false;
    auto ret = rcl_wait_for_subscribers(
      node_->get_node_base_interface()->get_rcl_node_handle(),
      &allocator,
      topic_name.c_str(),
      1u,
      static_cast<rcutils_duration_value_t>(2e9),
      &success);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ASSERT_TRUE(success);

    for (const auto & message : test_messages) {
      publisher->publish_as_loaned_msg(
        serialize_message<int64_t, test_msgs::msg::BasicTypes>(message));
    }

    auto subscribed_messages = subscriber_future_.get();
    EXPECT_THAT(subscribed_messages, SizeIs(Not(0)));
    EXPECT_EQ(subscribed_messages[0], test_messages[0]);
  } else {
    ASSERT_THROW(
    {
      publisher->publish_as_loaned_msg(
        serialize_message<int64_t, test_msgs::msg::BasicTypes>(test_messages[0]));
    }, rclcpp::exceptions::RCLError);
  }
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
    [](std::shared_ptr<const test_msgs::msg::Strings>/* message */) {});
  auto connected = [publisher, subscription]() -> bool {
      return publisher->get_subscription_count() && subscription->get_publisher_count();
    };
  // It normally takes < 20ms, 5s chosen as "a very long time"
  ASSERT_TRUE(wait_for(connected, 5s));
}
