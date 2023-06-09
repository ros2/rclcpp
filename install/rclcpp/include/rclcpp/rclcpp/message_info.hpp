// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__MESSAGE_INFO_HPP_
#define RCLCPP__MESSAGE_INFO_HPP_

#include "rmw/types.h"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Additional meta data about messages taken from subscriptions.
class RCLCPP_PUBLIC MessageInfo
{
public:
  /// Default empty constructor.
  MessageInfo() = default;

  /// Conversion constructor, which is intentionally not marked as explicit.
  /**
   * \param[in] rmw_message_info message info to initialize the class
   */
  // cppcheck-suppress noExplicitConstructor
  MessageInfo(const rmw_message_info_t & rmw_message_info);  // NOLINT(runtime/explicit)

  virtual ~MessageInfo();

  /// Return the message info as the underlying rmw message info type.
  const rmw_message_info_t &
  get_rmw_message_info() const;

  /// Return the message info as the underlying rmw message info type.
  rmw_message_info_t &
  get_rmw_message_info();

private:
  rmw_message_info_t rmw_message_info_;
};

}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_INFO_HPP_
