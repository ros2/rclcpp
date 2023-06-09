// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUBSCRIPTION_CONTENT_FILTER_OPTIONS_HPP_
#define RCLCPP__SUBSCRIPTION_CONTENT_FILTER_OPTIONS_HPP_

#include <string>
#include <vector>

namespace rclcpp
{

/// Options to configure content filtered topic in the subscription.
struct ContentFilterOptions
{
  /// Filter expression is similar to the WHERE part of an SQL clause.
  std::string filter_expression;
  /**
   * Expression parameters is the tokens placeholder ‘parameters’ (i.e., "%n" tokens begin from 0)
   * in the filter_expression. The maximum expression_parameters size is 100.
   */
  std::vector<std::string> expression_parameters;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_CONTENT_FILTER_OPTIONS_HPP_
