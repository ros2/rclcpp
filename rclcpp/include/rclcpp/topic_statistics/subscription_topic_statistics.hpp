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

#ifndef RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_
#define RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "libstatistics_collector/collector/generate_statistics_message.hpp"
#include "libstatistics_collector/moving_average_statistics/types.hpp"
#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_age.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"

#include "rcl/time.h"
#include "rclcpp/time.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"

#include "rcpputils/mutex.hpp"

namespace rclcpp
{
namespace topic_statistics
{

constexpr const char kDefaultPublishTopicName[]{"/statistics"};
constexpr const std::chrono::milliseconds kDefaultPublishingPeriod{std::chrono::seconds(1)};

using libstatistics_collector::collector::GenerateStatisticMessage;
using statistics_msgs::msg::MetricsMessage;
using libstatistics_collector::moving_average_statistics::StatisticData;

/**
 * Class used to collect, measure, and publish topic statistics data. Current statistics
 * supported for subscribers are received message age and received message period.
 *
 * \tparam CallbackMessageT the subscribed message type
 */
template<typename CallbackMessageT>
class SubscriptionTopicStatistics
{
  using TopicStatsCollector =
    libstatistics_collector::topic_statistics_collector::TopicStatisticsCollector<
    CallbackMessageT>;
  using ReceivedMessageAge =
    libstatistics_collector::topic_statistics_collector::ReceivedMessageAgeCollector<
    CallbackMessageT>;
  using ReceivedMessagePeriod =
    libstatistics_collector::topic_statistics_collector::ReceivedMessagePeriodCollector<
    CallbackMessageT>;

public:
  /// Construct a SubscriptionTopicStatistics object.
  /**
   * This object wraps utilities, defined in libstatistics_collector, to collect,
   * measure, and publish topic statistics data. This throws an invalid_argument
   * if the input publisher is null.
   *
   * \param node_name the name of the node, which created this instance, in order to denote
   * topic source
   * \param publisher instance constructed by the node in order to publish statistics data.
   * This class owns the publisher.
   * \throws std::invalid_argument if publisher pointer is nullptr
   */
  SubscriptionTopicStatistics(
    const std::string & node_name,
    rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher)
  : node_name_(node_name),
    publisher_(std::move(publisher))
  {
    // TODO(dbbonnie): ros-tooling/aws-roadmap/issues/226, received message age

    if (nullptr == publisher_) {
      throw std::invalid_argument("publisher pointer is nullptr");
    }

    bring_up();
  }

  virtual ~SubscriptionTopicStatistics()
  {
    tear_down();
  }

  /// Handle a message received by the subscription to collect statistics.
  /**
   * This method acquires a lock to prevent race conditions to collectors list.
   *
   * \param received_message the message received by the subscription
   * \param now_nanoseconds current time in nanoseconds
   */
  virtual void handle_message(
    const CallbackMessageT & received_message,
    const rclcpp::Time now_nanoseconds) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & collector : subscriber_statistics_collectors_) {
      collector->OnMessageReceived(received_message, now_nanoseconds.nanoseconds());
    }
  }

  /// Set the timer used to publish statistics messages.
  /**
   * \param publisher_timer the timer to fire the publisher, created by the node
   */
  void set_publisher_timer(rclcpp::TimerBase::SharedPtr publisher_timer)
  {
    publisher_timer_ = publisher_timer;
  }

  /// Publish a populated MetricsStatisticsMessage.
  /**
   * This method acquires a lock to prevent race conditions to collectors list.
   */
  virtual void publish_message_and_reset_measurements()
  {
    std::vector<MetricsMessage> msgs;
    rclcpp::Time window_end{get_current_nanoseconds_since_epoch()};

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & collector : subscriber_statistics_collectors_) {
        const auto collected_stats = collector->GetStatisticsResults();
        collector->ClearCurrentMeasurements();

        auto message = libstatistics_collector::collector::GenerateStatisticMessage(
          node_name_,
          collector->GetMetricName(),
          collector->GetMetricUnit(),
          window_start_,
          window_end,
          collected_stats);
        msgs.push_back(message);
      }
    }

    for (auto & msg : msgs) {
      publisher_->publish(msg);
    }
    window_start_ = window_end;
  }

protected:
  /// Return a vector of all the currently collected data.
  /**
   * This method acquires a lock to prevent race conditions to collectors list.
   *
   * \return a vector of all the collected data
   */
  std::vector<StatisticData> get_current_collector_data() const
  {
    std::vector<StatisticData> data;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & collector : subscriber_statistics_collectors_) {
      data.push_back(collector->GetStatisticsResults());
    }
    return data;
  }

private:
  /// Construct and start all collectors and set window_start_.
  /**
   * This method acquires a lock to prevent race conditions to collectors list.
   */
  void bring_up()
  {
    auto received_message_age = std::make_unique<ReceivedMessageAge>();
    received_message_age->Start();
    subscriber_statistics_collectors_.emplace_back(std::move(received_message_age));

    auto received_message_period = std::make_unique<ReceivedMessagePeriod>();
    received_message_period->Start();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      subscriber_statistics_collectors_.emplace_back(std::move(received_message_period));
    }

    window_start_ = rclcpp::Time(get_current_nanoseconds_since_epoch());
  }

  /// Stop all collectors, clear measurements, stop publishing timer, and reset publisher.
  /**
   * This method acquires a lock to prevent race conditions to collectors list.
   */
  void tear_down()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & collector : subscriber_statistics_collectors_) {
        collector->Stop();
      }

      subscriber_statistics_collectors_.clear();
    }

    if (publisher_timer_) {
      publisher_timer_->cancel();
      publisher_timer_.reset();
    }

    publisher_.reset();
  }

  /// Return the current nanoseconds (count) since epoch.
  /**
   * \return the current nanoseconds (count) since epoch
   */
  int64_t get_current_nanoseconds_since_epoch() const
  {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  }

  /// Mutex to protect the subsequence vectors
  mutable rcpputils::PIMutex mutex_;
  /// Collection of statistics collectors
  std::vector<std::unique_ptr<TopicStatsCollector>> subscriber_statistics_collectors_{};
  /// Node name used to generate topic statistics messages to be published
  const std::string node_name_;
  /// Publisher, created by the node, used to publish topic statistics messages
  rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;
  /// Timer which fires the publisher
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  /// The start of the collection window, used in the published topic statistics message
  rclcpp::Time window_start_;
};
}  // namespace topic_statistics
}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_
