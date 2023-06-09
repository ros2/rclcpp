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

#ifndef RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_
#define RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_

#include <stdexcept>

#include "rclcpp/topic_statistics_state.hpp"

namespace rclcpp
{
namespace detail
{

/// Return whether or not topic statistics is enabled, resolving "NodeDefault" if needed.
template<typename OptionsT, typename NodeBaseT>
bool
resolve_enable_topic_statistics(const OptionsT & options, const NodeBaseT & node_base)
{
  bool topic_stats_enabled;
  switch (options.topic_stats_options.state) {
    case TopicStatisticsState::Enable:
      topic_stats_enabled = true;
      break;
    case TopicStatisticsState::Disable:
      topic_stats_enabled = false;
      break;
    case TopicStatisticsState::NodeDefault:
      topic_stats_enabled = node_base.get_enable_topic_statistics_default();
      break;
    default:
      throw std::runtime_error("Unrecognized EnableTopicStatistics value");
  }

  return topic_stats_enabled;
}

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_
