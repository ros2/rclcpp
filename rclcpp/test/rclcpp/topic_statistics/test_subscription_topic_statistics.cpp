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

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/msg/message_with_header.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "test_msgs/msg/empty.hpp"

#include "test_topic_stats_utils.hpp"

namespace
{
constexpr const char kTestPubNodeName[]{"test_pub_stats_node"};
constexpr const char kTestSubNodeName[]{"test_sub_stats_node"};
constexpr const char kTestSubStatsTopic[]{"/test_sub_stats_topic"};
constexpr const char kTestSubStatsEmptyTopic[]{"/test_sub_stats_empty_topic"};
constexpr const char kTestTopicStatisticsTopic[]{"/test_topic_statistics_topic"};
constexpr const char kMessageAgeSourceLabel[]{"message_age"};
constexpr const char kMessagePeriodSourceLabel[]{"message_period"};
constexpr const uint64_t kNoSamples{0};
constexpr const std::chrono::seconds kTestDuration{10};
constexpr const uint64_t kNumExpectedMessages{8};
constexpr const uint64_t kNumExpectedMessageAgeMessages{kNumExpectedMessages / 2};
constexpr const uint64_t kNumExpectedMessagePeriodMessages{kNumExpectedMessages / 2};
}  // namespace

using rclcpp::msg::MessageWithHeader;
using test_msgs::msg::Empty;
using rclcpp::topic_statistics::SubscriptionTopicStatistics;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using libstatistics_collector::moving_average_statistics::StatisticData;

/**
 * Wrapper class to test and expose parts of the SubscriptionTopicStatistics<T> class.
 * \tparam CallbackMessageT
 */
template<typename CallbackMessageT>
class TestSubscriptionTopicStatistics : public SubscriptionTopicStatistics<CallbackMessageT>
{
public:
  TestSubscriptionTopicStatistics(
    const std::string & node_name,
    rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher)
  : SubscriptionTopicStatistics<CallbackMessageT>(node_name, publisher)
  {
  }

  virtual ~TestSubscriptionTopicStatistics() = default;

  /// Exposed for testing
  std::vector<StatisticData> get_current_collector_data() const
  {
    return SubscriptionTopicStatistics<CallbackMessageT>::get_current_collector_data();
  }
};

/**
 * Empty publisher node: used to publish empty messages
 */
class EmptyPublisher : public rclcpp::Node
{
public:
  EmptyPublisher(
    const std::string & name, const std::string & topic,
    const std::chrono::milliseconds & publish_period = std::chrono::milliseconds{100})
  : Node(name)
  {
    publisher_ = create_publisher<Empty>(topic, 10);
    publish_timer_ = this->create_wall_timer(
      publish_period, [this]() {
        this->publish_message();
      });
  }

  virtual ~EmptyPublisher() = default;

private:
  void publish_message()
  {
    auto msg = Empty{};
    publisher_->publish(msg);
  }

  rclcpp::Publisher<Empty>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

/**
 * MessageWithHeader publisher node: used to publish MessageWithHeader with `header` value set
 */
class MessageWithHeaderPublisher : public rclcpp::Node
{
public:
  MessageWithHeaderPublisher(
    const std::string & name, const std::string & topic,
    const std::chrono::milliseconds & publish_period = std::chrono::milliseconds{100})
  : Node(name)
  {
    publisher_ = create_publisher<MessageWithHeader>(topic, 10);
    publish_timer_ = this->create_wall_timer(
      publish_period, [this]() {
        this->publish_message();
      });
    uniform_dist_ = std::uniform_int_distribution<uint32_t>{1000000, 100000000};
  }

  virtual ~MessageWithHeaderPublisher() = default;

private:
  void publish_message()
  {
    std::random_device rd;
    std::mt19937 gen{rd()};
    uint32_t d = uniform_dist_(gen);
    auto msg = MessageWithHeader{};
    // Subtract ~1 second (add some noise for a non-zero standard deviation)
    // so the received message age calculation is always > 0
    msg.header.stamp = this->now() - rclcpp::Duration{1, d};
    publisher_->publish(msg);
  }

  rclcpp::Publisher<MessageWithHeader>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::uniform_int_distribution<uint32_t> uniform_dist_;
};

/**
 * Empty subscriber node: used to create subscriber topic statistics requirements
 */
class EmptySubscriber : public rclcpp::Node
{
public:
  EmptySubscriber(const std::string & name, const std::string & topic)
  : Node(name)
  {
    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    auto callback = [](Empty::UniquePtr msg) {
        (void) msg;
      };
    subscription_ = create_subscription<Empty,
        std::function<void(Empty::UniquePtr)>>(
      topic,
      rclcpp::QoS(rclcpp::KeepAll()),
      callback,
      options);
  }
  virtual ~EmptySubscriber() = default;

private:
  rclcpp::Subscription<Empty>::SharedPtr subscription_;
};

/**
 * MessageWithHeader subscriber node: used to create subscriber topic statistics requirements
 */
class MessageWithHeaderSubscriber : public rclcpp::Node
{
public:
  MessageWithHeaderSubscriber(const std::string & name, const std::string & topic)
  : Node(name)
  {
    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    auto callback = [](MessageWithHeader::UniquePtr msg) {
        (void) msg;
      };
    subscription_ = create_subscription<MessageWithHeader,
        std::function<void(MessageWithHeader::UniquePtr)>>(
      topic,
      rclcpp::QoS(rclcpp::KeepAll()),
      callback,
      options);
  }
  virtual ~MessageWithHeaderSubscriber() = default;

private:
  rclcpp::Subscription<MessageWithHeader>::SharedPtr subscription_;
};

/**
 * Test fixture to bring up and teardown rclcpp
 */
class TestSubscriptionTopicStatisticsFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0 /* argc */, nullptr /* argv */);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

/**
 * Check if a received statistics message is empty (no data was observed)
 * \param message_to_check
 */
void check_if_statistics_message_is_empty(const MetricsMessage & message_to_check)
{
  for (const auto & stats_point : message_to_check.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
        EXPECT_EQ(0, stats_point.data) << "unexpected sample count" << stats_point.data;
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
      case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
        EXPECT_TRUE(std::isnan(stats_point.data)) << "unexpected value" << stats_point.data <<
          " for type:" << type;
        break;
      default:
        FAIL() << "received unknown statistics type: " << std::dec <<
          static_cast<unsigned int>(type);
    }
  }
}

/**
 * Check if a received statistics message observed data and contains some calculation
 * \param message_to_check
 */
void check_if_statistic_message_is_populated(const MetricsMessage & message_to_check)
{
  for (const auto & stats_point : message_to_check.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
        EXPECT_LT(0, stats_point.data) << "unexpected sample count " << stats_point.data;
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
        EXPECT_LT(0, stats_point.data) << "unexpected avg " << stats_point.data;
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
        EXPECT_LT(0, stats_point.data) << "unexpected mi n" << stats_point.data;
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
        EXPECT_LT(0, stats_point.data) << "unexpected max " << stats_point.data;
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
        EXPECT_LT(0, stats_point.data) << "unexpected stddev " << stats_point.data;
        break;
      default:
        FAIL() << "received unknown statistics type: " << std::dec <<
          static_cast<unsigned int>(type);
    }
  }
}

/**
 * Test an invalid argument is thrown for a bad input publish period.
 */
TEST(TestSubscriptionTopicStatistics, test_invalid_publish_period)
{
  rclcpp::init(0 /* argc */, nullptr /* argv */);

  auto node = std::make_shared<rclcpp::Node>("test_period_node");

  auto options = rclcpp::SubscriptionOptions();
  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  options.topic_stats_options.publish_period = std::chrono::milliseconds(0);

  auto callback = [](Empty::UniquePtr msg) {
      (void) msg;
    };

  ASSERT_THROW(
    (node->create_subscription<Empty, std::function<void(Empty::UniquePtr)>>(
      "should_throw_invalid_arg",
      rclcpp::QoS(rclcpp::KeepAll()),
      callback,
      options)), std::invalid_argument);

  rclcpp::shutdown();
}

/**
 * Test that we can manually construct the subscription topic statistics utility class
 * without any errors and defaults to empty measurements.
 */
TEST_F(TestSubscriptionTopicStatisticsFixture, test_manual_construction)
{
  auto empty_subscriber = std::make_shared<EmptySubscriber>(
    kTestSubNodeName,
    kTestSubStatsEmptyTopic);

  // Manually create publisher tied to the node
  auto topic_stats_publisher =
    empty_subscriber->create_publisher<MetricsMessage>(
    kTestTopicStatisticsTopic,
    10);

  // Construct a separate instance
  auto sub_topic_stats = std::make_unique<TestSubscriptionTopicStatistics<Empty>>(
    empty_subscriber->get_name(),
    topic_stats_publisher);

  // Expect no data has been collected / no samples received
  for (const auto & data : sub_topic_stats->get_current_collector_data()) {
    EXPECT_TRUE(std::isnan(data.average));
    EXPECT_TRUE(std::isnan(data.min));
    EXPECT_TRUE(std::isnan(data.max));
    EXPECT_TRUE(std::isnan(data.standard_deviation));
    EXPECT_EQ(kNoSamples, data.sample_count);
  }
}

/**
 * Publish messages that do not have a header timestamp, test that all statistics messages
 * were received, and verify the statistics message contents.
 */
TEST_F(TestSubscriptionTopicStatisticsFixture, test_receive_stats_for_message_no_header)
{
  // Create an empty publisher
  auto empty_publisher = std::make_shared<EmptyPublisher>(
    kTestPubNodeName,
    kTestSubStatsEmptyTopic);
  // empty_subscriber has a topic statistics instance as part of its subscription
  // this will listen to and generate statistics for the empty message

  // Create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_single_empty_stats_message_listener",
    "/statistics",
    kNumExpectedMessages);

  auto empty_subscriber = std::make_shared<EmptySubscriber>(
    kTestSubNodeName,
    kTestSubStatsEmptyTopic);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(empty_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(empty_subscriber);

  // Spin and get future
  ex.spin_until_future_complete(
    statistics_listener->GetFuture(),
    kTestDuration);

  // Compare message counts, sample count should be the same as published and received count
  EXPECT_EQ(kNumExpectedMessages, statistics_listener->GetNumberOfMessagesReceived());

  // Check the received message total count
  const auto received_messages = statistics_listener->GetReceivedMessages();
  EXPECT_EQ(kNumExpectedMessages, received_messages.size());

  // check the type of statistics that were received and their counts
  uint64_t message_age_count{0};
  uint64_t message_period_count{0};

  std::set<std::string> received_metrics;
  for (const auto & msg : received_messages) {
    if (msg.metrics_source == "message_age") {
      message_age_count++;
    }
    if (msg.metrics_source == "message_period") {
      message_period_count++;
    }
  }
  EXPECT_EQ(kNumExpectedMessageAgeMessages, message_age_count);
  EXPECT_EQ(kNumExpectedMessagePeriodMessages, message_period_count);

  // Check the collected statistics for message period.
  // Message age statistics will not be calculated because Empty messages
  // don't have a `header` with timestamp. This means that we expect to receive a `message_age`
  // and `message_period` message for each empty message published.
  for (const auto & msg : received_messages) {
    if (msg.metrics_source == kMessageAgeSourceLabel) {
      check_if_statistics_message_is_empty(msg);
    } else if (msg.metrics_source == kMessagePeriodSourceLabel) {
      check_if_statistic_message_is_populated(msg);
    }
  }
}

TEST_F(TestSubscriptionTopicStatisticsFixture, test_receive_stats_for_message_with_header)
{
  // Create a MessageWithHeader publisher
  auto msg_with_header_publisher = std::make_shared<MessageWithHeaderPublisher>(
    kTestPubNodeName,
    kTestSubStatsTopic);
  // empty_subscriber has a topic statistics instance as part of its subscription
  // this will listen to and generate statistics for the empty message

  // Create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_stats_for_message_with_header",
    "/statistics",
    kNumExpectedMessages);

  auto msg_with_header_subscriber = std::make_shared<MessageWithHeaderSubscriber>(
    kTestSubNodeName,
    kTestSubStatsTopic);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(msg_with_header_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(msg_with_header_subscriber);

  // Spin and get future
  ex.spin_until_future_complete(
    statistics_listener->GetFuture(),
    kTestDuration);

  // Compare message counts, sample count should be the same as published and received count
  EXPECT_EQ(kNumExpectedMessages, statistics_listener->GetNumberOfMessagesReceived());

  // Check the received message total count
  const auto received_messages = statistics_listener->GetReceivedMessages();
  EXPECT_EQ(kNumExpectedMessages, received_messages.size());

  // check the type of statistics that were received and their counts
  uint64_t message_age_count{0};
  uint64_t message_period_count{0};

  std::set<std::string> received_metrics;
  for (const auto & msg : received_messages) {
    if (msg.metrics_source == kMessageAgeSourceLabel) {
      message_age_count++;
    }
    if (msg.metrics_source == kMessagePeriodSourceLabel) {
      message_period_count++;
    }
  }
  EXPECT_EQ(kNumExpectedMessageAgeMessages, message_age_count);
  EXPECT_EQ(kNumExpectedMessagePeriodMessages, message_period_count);

  for (const auto & msg : received_messages) {
    check_if_statistic_message_is_populated(msg);
  }
}
