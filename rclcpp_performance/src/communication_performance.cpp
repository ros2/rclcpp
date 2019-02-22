// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "rclcpp_performance/msg/matching_publisher.hpp"

using PeriodT = std::chrono::milliseconds;
using TimestampT = std::chrono::microseconds;

class  PublisherConstantRate
{
public:
  explicit PublisherConstantRate(
    rclcpp::Node* node,
    const std::string & topic_name,
    uint64_t publisher_id,
    std::string base_dir,
    uint64_t number_of_messages_to_send,
    uint64_t message_length,
    PeriodT period,
    bool use_unique_message,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default) :
  number_of_messages_to_send_(number_of_messages_to_send)
  {
    // Note(ivanpauno): msg type could be changed, to try the different publish methods.
    shared_msg_ = std::make_shared<rclcpp_performance::msg::MatchingPublisher>();
    shared_msg_->publisher_id = publisher_id;
    shared_msg_->data.resize(message_length);
    shared_msg_->message_id = 0;

    std::stringstream ss;

    ss << base_dir << "/publisher_" << publisher_id << std::ends;
    logging_file_.open(ss.str(), std::ios_base::trunc);
    logging_file_.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    pub_ = node->create_publisher<rclcpp_performance::msg::MatchingPublisher>(
        topic_name,
        qos_profile);

    // Use a timer to schedule periodic message publishing.
    // The callback is different depending on the message type.
    if (!use_unique_message) {
      auto callback = [this]() -> void
        {
          if (this->number_of_messages_to_send_) {
            this->logging_file_ << this->shared_msg_->message_id << ", "
            << std::chrono::duration_cast<TimestampT>(
                std::chrono::system_clock::now().time_since_epoch()).count()
              << std::endl;
            this->pub_->publish(this->shared_msg_);
            this->shared_msg_->message_id++;
          } else {
            this->timer_->cancel();
            this->st_number_of_publishers_--;
          }
          this->number_of_messages_to_send_--;
        };
      timer_ = node->create_wall_timer(period, callback);
    } else {
      auto callback = [this, message_length]() -> void
        {
          if (this->number_of_messages_to_send_) {
            // First create a shared message and copy the data.
            // Then log and publish.
            rclcpp_performance::msg::MatchingPublisher::UniquePtr unique_msg =
              std::make_unique<rclcpp_performance::msg::MatchingPublisher>();
            unique_msg->publisher_id = this->shared_msg_->publisher_id;
            unique_msg->message_id = this->shared_msg_->message_id;
            unique_msg->data.resize(message_length);
            this->logging_file_ << unique_msg->message_id << ", "
            << std::chrono::duration_cast<TimestampT>(
                std::chrono::system_clock::now().time_since_epoch()).count()
              << std::endl;
            this->pub_->publish(unique_msg);
            this->shared_msg_->message_id++;
          } else {
            this->timer_->cancel();
            this->st_number_of_publishers_--;
          }
          this->number_of_messages_to_send_--;
        };
      timer_ = node->create_wall_timer(period, callback);
    }

    // Increment static counting
    PublisherConstantRate::st_number_of_publishers_++;
  }

  static uint64_t get_number_of_publishers() {
    return st_number_of_publishers_;
  }

private:
  rclcpp_performance::msg::MatchingPublisher::SharedPtr shared_msg_;
  rclcpp::Publisher<rclcpp_performance::msg::MatchingPublisher>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream logging_file_;
  uint64_t number_of_messages_to_send_;

  static uint64_t st_number_of_publishers_;
};

uint64_t PublisherConstantRate::st_number_of_publishers_ = 0;

class SubscriptionLogger
{
public:
  explicit SubscriptionLogger(
    rclcpp::Node* node,
    const std::string & topic_name,
    uint64_t subscription_id,
    std::string base_dir,
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default
    )
  {
    std::stringstream ss;
    ss << base_dir << "/subscription_" << subscription_id << std::ends;

    logging_file_.open(ss.str(), std::ios_base::trunc);
    logging_file_.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    auto callback = [this](const rclcpp_performance::msg::MatchingPublisher::SharedPtr msg) -> void
      { 
        this->logging_file_ << msg->publisher_id << ", "
          << msg->message_id << ", "
          << std::chrono::duration_cast<TimestampT>(
            std::chrono::system_clock::now().time_since_epoch()).count()
          << std::endl;
      };

    sub_ = node->create_subscription<rclcpp_performance::msg::MatchingPublisher>(
      topic_name,
      callback,
      qos_profile);
  }

private:
  rclcpp::Subscription<rclcpp_performance::msg::MatchingPublisher>::SharedPtr sub_;
  std::ofstream logging_file_;
};

class ShutdownHook
{
public:
  ShutdownHook(
    rclcpp::Node* node,
    PeriodT period = PeriodT(10000))
  {
    auto callback = [this]() -> void
      {
        uint64_t number_of_publishers =
          PublisherConstantRate::get_number_of_publishers();
        printf(
          "Checking shutdown, %lu publishers are still alive\n",
          number_of_publishers);
        if (!number_of_publishers) {
          this->timer_->cancel();
          rclcpp::shutdown();
          printf("Done!\n");
        }
      };
    timer_ = node->create_wall_timer(period, callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

class PerformanceTestNode : public rclcpp::Node
{
public:
  explicit PerformanceTestNode(
    std::string node_name,
    rclcpp::NodeOptions node_options,
    std::string base_dir,
    uint64_t number_of_pub_sub,
    uint64_t number_of_messages,
    uint64_t message_length,
    PeriodT publish_period,
    bool use_unique_message) :
  Node(node_name, node_options)
  {
    // Create Publishers and Subscriptions, and match one to the other.
    for (uint64_t i = 0; i < number_of_pub_sub; i++) {
      std::stringstream ss;
      ss << "topic_" << i << std::ends;
      std::shared_ptr<SubscriptionLogger> sub =
        std::make_shared<SubscriptionLogger>(
          this,
          ss.str(),
          i,
          base_dir);
      subscriptions_.push_back(sub);
      std::shared_ptr<PublisherConstantRate> pub =
        std::make_shared<PublisherConstantRate>(
          this,
          ss.str(),
          i,
          base_dir,
          number_of_messages,
          message_length,
          publish_period,
          use_unique_message);
      publishers_.push_back(pub);
    }
    shutdown_hook_ = std::make_shared<ShutdownHook>(this);
  }
private:
  std::vector<std::shared_ptr<PublisherConstantRate>> publishers_;
  std::vector<std::shared_ptr<SubscriptionLogger>> subscriptions_;
  std::shared_ptr<ShutdownHook> shutdown_hook_;
};

int main(int argc, char ** argv)
{
  std::string base_dir = "logs/";
  PeriodT period = PeriodT(100);
  uint64_t number_of_messages = 1000;
  uint64_t number_of_publishers = 100;
  uint64_t message_length = 100;
  rclcpp::NodeOptions node_options;
  bool use_unique_message = false;

  rclcpp::init(argc, argv);

  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-d");
  if (nullptr != cli_option) {
    base_dir = std::string(cli_option);
  }
  printf("Output directory: %s\n", base_dir.c_str());

  cli_option = rcutils_cli_get_option(argv, argv + argc, "-p");
  if (nullptr != cli_option) {
    uint64_t p;
    std::istringstream iss(cli_option);
    iss >> p;
    period = PeriodT(p);
  }

  cli_option = rcutils_cli_get_option(argv, argv + argc, "-m");
  if (nullptr != cli_option) {
    std::istringstream iss(cli_option);
    iss >> number_of_messages;
  }

  cli_option = rcutils_cli_get_option(argv, argv + argc, "-n");
  if (nullptr != cli_option) {
    std::istringstream iss(cli_option);
    iss >> number_of_publishers;
  }

  cli_option = rcutils_cli_get_option(argv, argv + argc, "-l");
  if (nullptr != cli_option) {
    std::istringstream iss(cli_option);
    iss >> message_length;
  }

  bool option = rcutils_cli_option_exist(argv, argv + argc, "-intra");
  if (option) {
    node_options.use_intra_process_comms(true);
  }

  option = rcutils_cli_option_exist(argv, argv + argc, "-unique");
  if (option) {
    use_unique_message = true;
  }

  // TODO(ivanpauno): Add directory error checking

  auto node = std::make_shared<PerformanceTestNode>(
    "communication_performance",
    node_options,
    base_dir,
    number_of_publishers, //number of publishers/subscriptions
    number_of_messages, // number of messages to be send
    message_length, // message length in bytes
    period, // Publish period
    use_unique_message); //use unique message or shared

  rclcpp::spin(node);
  // Shutdown is called internally by the node,
  // when the test ends.
  // Repeated here in case signal was sent.
  rclcpp::shutdown();
  return 0;
}
