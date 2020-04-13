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

#ifndef RCLCPP__TOPIC_STATISTICS__SUBSCRIBER_TOPIC_STATISTICS_HPP_
#define RCLCPP__TOPIC_STATISTICS__SUBSCRIBER_TOPIC_STATISTICS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>


#include "rclcpp/create_publisher.hpp"
#include "rcl/time.h"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"

#include "metrics_statistics_msgs/msg/metrics_message.hpp"

#include "libstatistics_collector/collector/generate_statistics_message.hpp"
#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"
#include "libstatistics_collector/moving_average_statistics/types.hpp"

namespace
{
/// Return the current nanoseconds (count) since epoch.
/**
 * For now, use hard coded time instead of a node's clock (to support sim time and playback)
 * due to node clock lifecycle issues.
 * \return the current nanoseconds (count) since epoch
 */
int64_t GetCurrentNanosecondsSinceEpoch()
{
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}
}  // namespace

namespace rclcpp
{
namespace topic_statistics
{

constexpr const char kDefaultPublishTopicName[]{"topic_statistics"};
constexpr const std::chrono::milliseconds kDefaultPublishingPeriod{std::chrono::minutes(1)};

using libstatistics_collector::collector::GenerateStatisticMessage;
using metrics_statistics_msgs::msg::MetricsMessage;
using libstatistics_collector::moving_average_statistics::StatisticData;

/**
 * Class used to collect, measure, and publish topic statistics data. Current statistics
 * supported for subscribers are received message age and received message period.
 *
 * \tparam CallbackMessageT the subscribed message type
 */
template<typename CallbackMessageT>
class SubscriberTopicStatistics
{
  using TopicStatsCollector =
    libstatistics_collector::topic_statistics_collector::TopicStatisticsCollector<
    CallbackMessageT>;
  using ReceivedMessagePeriod =
    libstatistics_collector::topic_statistics_collector::ReceivedMessagePeriodCollector<
    CallbackMessageT>;

public:
  /// Construct a SubcriberTopicStatistics object.
  /**
  * This object wraps utilities, defined in libstatistics_collector, to collect,
  * measure, and publish topic statistics data.
   *
  * \param node the node creating the subscription, used to create the publisher and timer to
  * publish topic statistics.
  * \param publishing_topic the topic to publish statistics to
  * \param publishing_period the period at which topic statistics messages are published
  */
  SubscriberTopicStatistics(
    rclcpp::Node & node,
    const std::string & publishing_topic = kDefaultPublishTopicName,
    const std::chrono::milliseconds & publishing_period = kDefaultPublishingPeriod)
  {
    publisher_ =
      node.create_publisher<metrics_statistics_msgs::msg::MetricsMessage>(
      publishing_topic,
      10);

    auto callback = [this]()
      {
        this->PublishMessage();
      };

    publisher_timer_ = node.create_wall_timer(publishing_period, callback);

    node_name_ = node.get_name();

    BringUp();
  }

  virtual ~SubscriberTopicStatistics()
  {
    TearDown();
  }

  /// Handle a message received by the subscription to collect statistics.
  /**
   * \param received_message the message received by the subscription
   * \param now_nanoseconds current time in nanoseconds
   */
  virtual void OnMessageReceived(
    const CallbackMessageT & received_message,
    const rcl_time_point_value_t now_nanoseconds) const
  {
    (void) received_message;

    for (const auto & collector : subscriber_statistics_collectors_) {
      collector->OnMessageReceived(received_message, now_nanoseconds);
    }
  }

  /// Return a vector of all the currently collected data
  /**
   * \return a vector of all the collected data
   */
  std::vector<StatisticData> GetCurrentCollectorData() const
  {
    std::vector<StatisticData> data;
    for (const auto & collector : subscriber_statistics_collectors_) {
      data.push_back(collector->GetStatisticsResults());
    }
    return data;
  }

private:
  /// Construct and start all collectors and set window_start_.
  void BringUp()
  {
    auto received_message_period = std::make_unique<ReceivedMessagePeriod>();
    received_message_period->Start();
    subscriber_statistics_collectors_.emplace_back(std::move(received_message_period));

    window_start_ = rclcpp::Time(GetCurrentNanosecondsSinceEpoch());
  }

  /// Stop all collectors, clear measurements, stop publishing timer, and reset publisher.
  void TearDown()
  {
    for (auto & collector : subscriber_statistics_collectors_) {
      collector->Stop();
    }

    subscriber_statistics_collectors_.clear();

    if (publisher_timer_) {
      publisher_timer_->cancel();
      publisher_timer_.reset();
    }

    publisher_.reset();
  }

  /// Publish a populated MetricsStatisticsMessage
  virtual void PublishMessage()
  {
    rclcpp::Time window_end{GetCurrentNanosecondsSinceEpoch()};

    for (auto & collector : subscriber_statistics_collectors_) {
      const auto collected_stats = collector->GetStatisticsResults();

      auto message = libstatistics_collector::collector::GenerateStatisticMessage(
        node_name_,
        collector->GetMetricName(),
        collector->GetMetricUnit(),
        window_start_,
        window_end,
        collected_stats);
      publisher_->publish(message);
    }
    window_start_ = window_end;
  }

  /// Collection of statistics collectors
  std::vector<std::unique_ptr<TopicStatsCollector>> subscriber_statistics_collectors_{};
  /// Node name used to generate topic statistics messages to be published
  std::string node_name_;
  /// Publisher, created by the node, used to publish topic statistics messages
  rclcpp::Publisher<metrics_statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;
  /// Timer which fires the publisher
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  /// The start of the collection window, used in the published topic statistics message
  rclcpp::Time window_start_;
};
}  // namespace topic_statistics
}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS__SUBSCRIBER_TOPIC_STATISTICS_HPP_
