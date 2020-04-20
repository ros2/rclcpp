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
#include <string>
#include <vector>

#include "rclcpp/subscription_options.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr const char defaultPublishTopic[] = "/statistics";
}

TEST(TestSubscriptionOptions, topic_statistics_options) {
  auto options = rclcpp::SubscriptionOptions();

  EXPECT_EQ(options.topic_stats_options.state, rclcpp::TopicStatisticsState::DISABLED);
  EXPECT_EQ(options.topic_stats_options.publish_topic, defaultPublishTopic);
  EXPECT_EQ(options.topic_stats_options.publish_period, 1s);

  options.topic_stats_options.state = rclcpp::TopicStatisticsState::ENABLED;
  options.topic_stats_options.publish_topic = "topic_statistics";
  options.topic_stats_options.publish_period = 5min;

  EXPECT_EQ(options.topic_stats_options.state, rclcpp::TopicStatisticsState::ENABLED);
  EXPECT_EQ(options.topic_stats_options.publish_topic, "topic_statistics");
  EXPECT_EQ(options.topic_stats_options.publish_period, 5min);
}
