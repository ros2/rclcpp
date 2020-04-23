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
#include <set>
#include <string>
#include <vector>

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "test_msgs/msg/empty.hpp"

#include "test_topic_stats_utils.hpp"

namespace
{
constexpr const char kTestPubNodeName[]{"test_pub_stats_node"};
constexpr const char kTestSubNodeName[]{"test_sub_stats_node"};
constexpr const char kTestSubStatsTopic[]{"/test_sub_stats_topic"};
constexpr const char kTestTopicStatisticsTopic[]{"/test_topic_statistics_topic"};
constexpr const uint64_t kNoSamples{0};
constexpr const std::chrono::seconds kTestDuration{10};
}  // namespace

using sensor_msgs::msg::Imu;
using test_msgs::msg::Empty;
using rclcpp::topic_statistics::SubscriptionTopicStatistics;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using libstatistics_collector::moving_average_statistics::StatisticData;

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
    ++number_published_;
    auto msg = Empty{};
    publisher_->publish(msg);
  }

  rclcpp::Publisher<Empty>::SharedPtr publisher_;
  std::atomic<size_t> number_published_{0};
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

/**
 * IMU message publisher node: used to publish IMU messages with `header` value set
 */
class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher(
    const std::string & name, const std::string & topic,
    const std::chrono::milliseconds & publish_period = std::chrono::milliseconds{100})
  : Node(name)
  {
    publisher_ = create_publisher<Imu>(topic, 10);
    publish_timer_ = this->create_wall_timer(
      publish_period, [this]() {
        this->publish_message();
      });
  }

  virtual ~ImuPublisher() = default;

private:
  void publish_message()
  {
    ++number_published_;
    auto msg = Imu{};
    auto now = this->now();
    auto nanos = now.nanoseconds() - 1000*1000;
    msg.header.stamp.sec = static_cast<std::int32_t>(RCL_NS_TO_S(nanos));
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(nanos % (1000 * 1000 * 1000));
    publisher_->publish(msg);
  }

  rclcpp::Publisher<Imu>::SharedPtr publisher_;
  std::atomic<size_t> number_published_{0};
  rclcpp::TimerBase::SharedPtr publish_timer_;
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

    auto callback = [this](Empty::UniquePtr msg) {
        this->receive_message(*msg);
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
  void receive_message(const Empty &) const
  {
  }
  rclcpp::Subscription<Empty>::SharedPtr subscription_;
};

/**
 * IMU subscriber node: used to create subscriber topic statistics requirements
 */
class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber(const std::string & name, const std::string & topic)
  : Node(name)
  {
    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    auto callback = [this](Imu::UniquePtr msg) {
        this->receive_message(*msg);
      };
    subscription_ = create_subscription<Imu,
        std::function<void(Imu::UniquePtr)>>(
      topic,
      rclcpp::QoS(rclcpp::KeepAll()),
      callback,
      options);
  }
  virtual ~ImuSubscriber() = default;

private:
  void receive_message(const Imu &) const
  {
  }
  rclcpp::Subscription<Imu>::SharedPtr subscription_;
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
    empty_subscriber = std::make_shared<EmptySubscriber>(
      kTestSubNodeName,
      kTestSubStatsTopic);
    imu_subscriber = std::make_shared<ImuSubscriber>(
      kTestSubNodeName,
      kTestSubStatsTopic);
  }

  void TearDown()
  {
    rclcpp::shutdown();
    empty_subscriber.reset();
  }
  std::shared_ptr<EmptySubscriber> empty_subscriber;
  std::shared_ptr<ImuSubscriber> imu_subscriber;
};

TEST_F(TestSubscriptionTopicStatisticsFixture, test_manual_construction)
{
  // manually create publisher tied to the node
  auto topic_stats_publisher =
    empty_subscriber->create_publisher<MetricsMessage>(
    kTestTopicStatisticsTopic,
    10);

  // construct a separate instance
  auto sub_topic_stats = std::make_unique<TestSubscriptionTopicStatistics<Empty>>(
    empty_subscriber->get_name(),
    topic_stats_publisher);

  // expect no data has been collected / no samples received
  for (const auto & data : sub_topic_stats->get_current_collector_data()) {
    EXPECT_TRUE(std::isnan(data.average));
    EXPECT_TRUE(std::isnan(data.min));
    EXPECT_TRUE(std::isnan(data.max));
    EXPECT_TRUE(std::isnan(data.standard_deviation));
    EXPECT_EQ(kNoSamples, data.sample_count);
  }
}

TEST_F(TestSubscriptionTopicStatisticsFixture, test_receive_single_empty_stats_message)
{
  // create an empty publisher
  auto empty_publisher = std::make_shared<EmptyPublisher>(
    kTestPubNodeName,
    kTestSubStatsTopic);
  // empty_subscriber has a topic statistics instance as part of its subscription
  // this will listen to and generate statistics for the empty message

  // create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_single_empty_stats_message_listener",
    "/statistics",
    2);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(empty_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(empty_subscriber);

  // spin and get future
  ex.spin_until_future_complete(
    statistics_listener->GetFuture(),
    kTestDuration);

  // compare message counts, sample count should be the same as published and received count
  EXPECT_EQ(2, statistics_listener->GetNumberOfMessagesReceived());

  // check the received message and the data types
  const auto received_messages = statistics_listener->GetReceivedMessages();

  EXPECT_EQ(2u, received_messages.size());

  std::set<std::string> received_metrics;
  for (const auto & msg : received_messages) {
    received_metrics.insert(msg.metrics_source);
  }
  EXPECT_TRUE(received_metrics.find("message_age") != received_metrics.end());
  EXPECT_TRUE(received_metrics.find("message_period") != received_metrics.end());

  // Check the collected statistics for message period.
  // Message age statistics will not be calculated because Empty messages
  // don't have a `header` with timestamp.

  // TODO(prajakta-gokhale): Change Empty message type to something with a `header`,
  // and have below assertions work for all collectors.
  for (const auto & msg : received_messages) {
    if (msg.metrics_source == "message_period") {
      for (const auto & stats_point : msg.statistics) {
        const auto type = stats_point.data_type;
        switch (type) {
          case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
            EXPECT_LT(0, stats_point.data) << "unexpected sample count";
            break;
          case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
            EXPECT_LT(0, stats_point.data) << "unexpected avg";
            break;
          case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
            EXPECT_LT(0, stats_point.data) << "unexpected min";
            break;
          case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
            EXPECT_LT(0, stats_point.data) << "unexpected max";
            break;
          case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
            EXPECT_LT(0, stats_point.data) << "unexpected stddev";
            break;
          default:
            FAIL() << "received unknown statistics type: " << std::dec <<
              static_cast<unsigned int>(type);
        }
      }
    }
  }
}

TEST_F(TestSubscriptionTopicStatisticsFixture, test_receive_stats_for_message_with_header)
{
  // create an empty publisher
  auto imu_publisher = std::make_shared<ImuPublisher>(
    kTestPubNodeName,
    kTestSubStatsTopic);
  // empty_subscriber has a topic statistics instance as part of its subscription
  // this will listen to and generate statistics for the empty message

  // create a listener for topic statistics messages
  auto statistics_listener = std::make_shared<rclcpp::topic_statistics::MetricsMessageSubscriber>(
    "test_receive_stats_for_message_with_header",
    "/statistics",
    2);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(imu_publisher);
  ex.add_node(statistics_listener);
  ex.add_node(imu_subscriber);

  // spin and get future
  ex.spin_until_future_complete(
    statistics_listener->GetFuture(),
    kTestDuration);

  // compare message counts, sample count should be the same as published and received count
  EXPECT_EQ(2, statistics_listener->GetNumberOfMessagesReceived());

  // check the received message and the data types
  const auto received_messages = statistics_listener->GetReceivedMessages();

  EXPECT_EQ(2u, received_messages.size());

  std::set<std::string> received_metrics;
  for (const auto & msg : received_messages) {
    received_metrics.insert(msg.metrics_source);
  }
  EXPECT_TRUE(received_metrics.find("message_age") != received_metrics.end());
  EXPECT_TRUE(received_metrics.find("message_period") != received_metrics.end());

  // Check the collected statistics for message period.

  for (const auto & msg : received_messages) {
    for (const auto & stats_point : msg.statistics) {
      const auto type = stats_point.data_type;
      switch (type) {
        case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
          EXPECT_LT(0, stats_point.data) << "unexpected sample count";
          break;
        case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
          EXPECT_LT(0, stats_point.data) << "unexpected avg";
          break;
        case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
          EXPECT_LT(0, stats_point.data) << "unexpected min";
          break;
        case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
          EXPECT_LT(0, stats_point.data) << "unexpected max";
          break;
        case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
          EXPECT_LE(0, stats_point.data) << "unexpected stddev";
          break;
        default:
          FAIL() << "received unknown statistics type: " << std::dec <<
            static_cast<unsigned int>(type);
      }
    }
  }
}
