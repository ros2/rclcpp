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

#include <memory>
#include <string>
#include <vector>

#include "libstatistics_collector/moving_average_statistics/types.hpp"

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "test_msgs/msg/empty.hpp"

namespace
{
constexpr const char kTestNodeName[]{"test_sub_stats_node"};
constexpr const char kTestSubStatsTopic[]{"/test_sub_stats_topic"};
constexpr const char kTestTopicStatisticsTopic[]{"/test_topic_statistics_topic"};
constexpr const uint64_t kNoSamples{0};
}  // namespace

using test_msgs::msg::Empty;
using statistics_msgs::msg::MetricsMessage;
using rclcpp::topic_statistics::SubscriptionTopicStatistics;
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
 * Empty subscriber node: used to create subscriber topic statistics requirements
 */
class EmptySubscriber : public rclcpp::Node
{
public:
  EmptySubscriber(const std::string & name, const std::string & topic)
  : Node(name)
  {
    auto callback = [this](Empty::UniquePtr msg) {
        this->receive_message(*msg);
      };
    subscription_ = create_subscription<Empty,
        std::function<void(Empty::UniquePtr)>>(
      topic,
      rclcpp::QoS(rclcpp::KeepAll()),
      callback);
  }

  virtual ~EmptySubscriber() = default;

private:
  void receive_message(const Empty &) const
  {
  }

  rclcpp::Subscription<Empty>::SharedPtr subscription_;
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
      kTestNodeName,
      kTestSubStatsTopic);
  }

  void TearDown()
  {
    rclcpp::shutdown();
    empty_subscriber.reset();
  }
  std::shared_ptr<EmptySubscriber> empty_subscriber;
};

TEST_F(TestSubscriptionTopicStatisticsFixture, test_manual_construction)
{
  // manually create publisher tied to the node
  auto topic_stats_publisher =
    empty_subscriber->create_publisher<MetricsMessage>(
    kTestTopicStatisticsTopic,
    10);

  // construct the instance
  auto sub_topic_stats = std::make_unique<TestSubscriptionTopicStatistics<Empty>>(
    empty_subscriber->get_name(),
    topic_stats_publisher);

  using libstatistics_collector::moving_average_statistics::StatisticData;

  // expect no data has been collected / no samples received
  for (const auto & data : sub_topic_stats->get_current_collector_data()) {
    EXPECT_TRUE(std::isnan(data.average));
    EXPECT_TRUE(std::isnan(data.min));
    EXPECT_TRUE(std::isnan(data.max));
    EXPECT_TRUE(std::isnan(data.standard_deviation));
    EXPECT_EQ(kNoSamples, data.sample_count);
  }
}
