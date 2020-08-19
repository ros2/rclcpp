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

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "statistics_msgs/msg/metrics_message.hpp"

#ifndef RCLCPP__TOPIC_STATISTICS__TEST_TOPIC_STATS_UTILS_HPP_
#define RCLCPP__TOPIC_STATISTICS__TEST_TOPIC_STATS_UTILS_HPP_

namespace rclcpp
{
namespace topic_statistics
{

using statistics_msgs::msg::MetricsMessage;

/**
* Provide an interface to wait for a promise to be satisfied via its future.
*/
class PromiseSetter
{
public:
  /**
   * Reassign the promise member and return it's future. Acquires a mutex in order
   * to mutate member variables.
   *
   * \return the promise member's future, called upon PeriodicMeasurement
   */
  std::shared_future<bool> GetFuture()
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    use_future_ = true;
    promise_ = std::promise<bool>();
    return promise_.get_future();
  }

protected:
  /**
   * Set the promise to true, which signals the corresponding future. Acquires a mutex and sets
   * the promise to true iff GetFuture was invoked before this.
   */
  void SetPromise()
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    if (use_future_) {
      // only set if GetFuture was called
      promise_.set_value(true);
      use_future_ = false;          // the promise needs to be reassigned to set again
    }
  }

private:
  mutable std::mutex mutex_;
  std::promise<bool> promise_;
  bool use_future_{false};
};

/**
 * Node which listens for published MetricsMessages. This uses the PromiseSetter API
 * in order to signal, via a future, that rclcpp should stop spinning upon
 * message handling.
 */
class MetricsMessageSubscriber : public rclcpp::Node, public PromiseSetter
{
public:
  /**
   * Constructs a MetricsMessageSubscriber.
   * \param name the node name
   * \param name the topic name
   * \param number of messages to receive to trigger the PromiseSetter future
   */
  MetricsMessageSubscriber(
    const std::string & name,
    const std::string & topic_name,
    const uint64_t number_of_messages_to_receive = 2)
  : rclcpp::Node(name),
    number_of_messages_to_receive_(number_of_messages_to_receive)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {
        this->MetricsMessageCallback(*msg);
      };
    subscription_ = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(
      topic_name,
      10 /*history_depth*/,
      callback);
  }

  /**
   * Acquires a mutex in order to get the last message received member.
   * \return the last message received
   */
  std::vector<MetricsMessage> GetReceivedMessages() const
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    return received_messages_;
  }

  /**
   * Return the number of messages received by this subscriber.
   * \return the number of messages received by the subscriber callback
   */
  uint64_t GetNumberOfMessagesReceived() const
  {
    return num_messages_received_;
  }

private:
  /**
   * Subscriber callback. Acquires a mutex to set the last message received and
   * sets the promise to true.
   * \param msg
   */
  void MetricsMessageCallback(const MetricsMessage & msg)
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    ++num_messages_received_;
    received_messages_.push_back(msg);
    if (num_messages_received_ >= number_of_messages_to_receive_) {
      PromiseSetter::SetPromise();
    }
  }

  std::vector<MetricsMessage> received_messages_;
  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::atomic<uint64_t> num_messages_received_{0};
  const uint64_t number_of_messages_to_receive_;
};

}  // namespace topic_statistics
}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS__TEST_TOPIC_STATS_UTILS_HPP_
