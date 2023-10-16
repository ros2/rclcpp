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
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/strings.hpp"

#include "test_topic_stats_utils.hpp"

namespace
{
constexpr const std::chrono::seconds defaultStatisticsPublishPeriod{1};
constexpr const char kTestPubNodeName[]{"test_pub_stats_node"};
constexpr const char kTestSubNodeName[]{"test_sub_stats_node"};
constexpr const char kTestSubStatsTopic[]{"/test_sub_stats_topic"};
constexpr const char kTestSubStatsEmptyTopic[]{"/test_sub_stats_empty_topic"};
constexpr const char kTestTopicStatisticsTopic[]{"/test_topic_statistics_topic"};
constexpr const char kMessageAgeSourceLabel[]{"message_age"};
constexpr const char kMessagePeriodSourceLabel[]{"message_period"};
constexpr const uint64_t kNoSamples{0};
constexpr const std::chrono::seconds kTestTimeout{10};
constexpr const uint64_t kNumExpectedWindows{4};
constexpr const uint64_t kNumExpectedMessages{kNumExpectedWindows * 2};
constexpr const uint64_t kNumExpectedMessageAgeMessages{kNumExpectedWindows};
constexpr const uint64_t kNumExpectedMessagePeriodMessages{kNumExpectedWindows};
constexpr const std::chrono::seconds kUnstableMessageAgeWindowDuration{
  defaultStatisticsPublishPeriod * (kNumExpectedWindows / 2)};
// kUnstableMessageAgeWindowDuration can take following value.
// Min: defaultStatisticsPublishPeriod * 2
// Max: defaultStatisticsPublishPeriod * (kNumExpectedWindows - 2)
constexpr const std::chrono::seconds kUnstableMessageAgeOffset{std::chrono::seconds{1}};
}  // namespace

using test_msgs::msg::Empty;
using rclcpp::topic_statistics::SubscriptionTopicStatistics;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using libstatistics_collector::moving_average_statistics::StatisticData;

/**
 * Wrapper class to test and expose parts of the SubscriptionTopicStatistics class.
 */
class TestSubscriptionTopicStatistics : public SubscriptionTopicStatistics
{
public:
  TestSubscriptionTopicStatistics(
    const std::string & node_name,
    rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher)
  : SubscriptionTopicStatistics(node_name, std::move(publisher))
  {
  }

  ~TestSubscriptionTopicStatistics() override = default;

  /// Exposed for testing
  using SubscriptionTopicStatistics::get_current_collector_data;
};

/**
 * PublisherNode wrapper: used to create publisher node
 */
template<typename MessageT>
class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(
    const std::string & name, const std::string & topic,
    const std::chrono::milliseconds & publish_period = std::chrono::milliseconds{100})
  : Node(name)
  {
    publisher_ = create_publisher<MessageT>(topic, 10);
    publish_timer_ = this->create_wall_timer(
      publish_period, [this]() {
        this->publish_message();
      });
  }

  ~PublisherNode() override = default;

private:
  void publish_message()
  {
    auto msg = MessageT{};
    publisher_->publish(msg);
  }

  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

/**
 * TransitionMessageStamp publisher emulator node : used to emulate publishing messages by
 * directly calling rclcpp::Subscription::handle_message(msg_shared_ptr, message_info).
 * The message age results change during the test.
 */
template<typename MessageT>
class TransitionMessageStampPublisherEmulator : public rclcpp::Node
{
public:
  TransitionMessageStampPublisherEmulator(
    const std::string & name,
    const std::chrono::seconds transition_duration, const std::chrono::seconds message_age_offset,
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription,
    const std::chrono::milliseconds & publish_period = std::chrono::milliseconds{100})
  : Node(name), transition_duration_(transition_duration), message_age_offset_(message_age_offset),
    subscription_(std::move(subscription))
  {
    publish_timer_ = this->create_wall_timer(publish_period, [this]() {this->publish_message();});
    start_time_ = this->now();
  }

private:
  void publish_message()
  {
    std::shared_ptr<void> msg_shared_ptr = std::make_shared<MessageT>();
    rmw_message_info_t rmw_message_info = rmw_get_zero_initialized_message_info();

    auto now = this->now();
    auto elapsed_time = now - start_time_;
    if (elapsed_time < transition_duration_) {
      // Apply only to the topic statistics in the first half
      // Subtract offset so message_age is always >= offset.
      rmw_message_info.source_timestamp = (now - message_age_offset_).nanoseconds();
    } else {
      rmw_message_info.source_timestamp = now.nanoseconds();
    }
    rclcpp::MessageInfo message_info{rmw_message_info};
    subscription_->handle_message(msg_shared_ptr, message_info);
  }

  std::chrono::seconds transition_duration_;
  std::chrono::seconds message_age_offset_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

/**
 * Message subscriber node: used to create subscriber with enabled topic statistics collectors
 *
 */
template<typename MessageT>
class SubscriberWithTopicStatistics : public rclcpp::Node
{
public:
  SubscriberWithTopicStatistics(
    const std::string & name, const std::string & topic,
    std::chrono::milliseconds publish_period = defaultStatisticsPublishPeriod)
  : Node(name)
  {
    // Manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = publish_period;

    auto callback = [](typename MessageT::UniquePtr msg) {
        (void) msg;
      };
    subscription_ = create_subscription<MessageT,
        std::function<void(typename MessageT::UniquePtr)>>(
      topic,
      rclcpp::QoS(rclcpp::KeepAll()),
      callback,
      options);
  }
  ~SubscriberWithTopicStatistics() override = default;

  typename rclcpp::Subscription<MessageT>::SharedPtr get_subscription()
  {
    return subscription_;
  }

private:
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
};

/**
 * Test fixture to bring up and teardown rclcpp
 */
class TestSubscriptionTopicStatisticsFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0 /* argc */, nullptr /* argv */);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

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
        EXPECT_LT(0, stats_point.data) << "unexpected min " << stats_point.data;
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
TEST_F(TestSubscriptionTopicStatisticsFixture, test_invalid_publish_period)
{
  ASSERT_THROW(
    SubscriberWithTopicStatistics<Empty>(
      "test_period_node", "should_throw_invalid_arg", std::chrono::milliseconds(0)
    ),
    std::invalid_argument);
}

/**
 * Test that we can manually construct the subscription topic statistics utility class
 * without any errors and defaults to empty measurements.
 */
TEST_F(TestSubscriptionTopicStatisticsFixture, test_manual_construction)
{
  auto empty_subscriber = std::make_shared<SubscriberWithTopicStatistics<Empty>>(
    kTestSubNodeName,
    kTestSubStatsEmptyTopic);

  // Manually create publisher tied to the node
  auto topic_stats_publisher =
    empty_subscriber->create_publisher<MetricsMessage>(
    kTestTopicStatisticsTopic,
    10);

  // Construct a separate instance
  auto sub_topic_stats = std::make_unique<TestSubscriptionTopicStatistics>(
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
  auto empty_publisher = std::make_shared<PublisherNode<Empty>>(
    kTestPubNodeName,
    kTestSubStatsEmptyTopic);
  // empty_subscriber has a topic statistics instance as part of its subscription
  // this will listen to and generate statistics for the empty message

  // Create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_single_empty_stats_message_listener",
    "/statistics",
    kNumExpectedMessages);

  auto empty_subscriber = std::make_shared<SubscriberWithTopicStatistics<Empty>>(
    kTestSubNodeName,
    kTestSubStatsEmptyTopic);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(empty_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(empty_subscriber);

  // Spin and get future
  ex.spin_until_future_complete(statistics_listener->GetFuture(), kTestTimeout);

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

  // Check the collected statistics for message period.
  for (const auto & msg : received_messages) {
    check_if_statistic_message_is_populated(msg);
  }
}

TEST_F(TestSubscriptionTopicStatisticsFixture, test_receive_stats_include_window_reset)
{
  // msg_subscriber_with_topic_statistics has a topic statistics instance as part of its
  // subscription this will listen to and generate statistics
  auto msg_subscriber_with_topic_statistics =
    std::make_shared<SubscriberWithTopicStatistics<test_msgs::msg::Strings>>(
    kTestSubNodeName,
    kTestSubStatsTopic);

  // Create a message publisher
  auto msg_publisher =
    std::make_shared<TransitionMessageStampPublisherEmulator<test_msgs::msg::Strings>>(
    kTestPubNodeName, kUnstableMessageAgeWindowDuration,
    kUnstableMessageAgeOffset, msg_subscriber_with_topic_statistics->get_subscription());

  // Create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_stats_include_window_reset", "/statistics", kNumExpectedMessages);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(msg_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(msg_subscriber_with_topic_statistics);

  // Spin and get future
  ex.spin_until_future_complete(statistics_listener->GetFuture(), kTestTimeout);

  const auto received_messages = statistics_listener->GetReceivedMessages();
  EXPECT_EQ(kNumExpectedMessages, received_messages.size());

  auto message_age_offset =
    std::chrono::duration<double, std::milli>(kUnstableMessageAgeOffset).count();

  // Check that the first statistic contains the offset inside of its window
  auto head_message = received_messages[0];
  for (const auto & stats_point : head_message.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
        EXPECT_GE(stats_point.data, message_age_offset);
        break;
      default:
        break;
    }
  }

  // Check that the last statistic does not contain the offset outside of its window
  auto tail_message = received_messages[received_messages.size() - 1];
  for (const auto & stats_point : tail_message.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
        EXPECT_LT(stats_point.data, message_age_offset);
        break;
      default:
        break;
    }
  }
}
