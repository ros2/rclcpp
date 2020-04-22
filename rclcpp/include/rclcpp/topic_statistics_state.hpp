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

#ifndef RCLCPP__TOPIC_STATISTICS_STATE_HPP_
#define RCLCPP__TOPIC_STATISTICS_STATE_HPP_

namespace rclcpp
{

/// Represent the state of topic statistics collector.
/// Used as argument in create_subscriber.
enum class TopicStatisticsState
{
  /// Explicitly enable topic statistics at subscription level.
  Enable,
  /// Explicitly disable topic statistics at subscription level.
  Disable,
  /// Take topic statistics state from the node.
  NodeDefault
};

}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS_STATE_HPP_
