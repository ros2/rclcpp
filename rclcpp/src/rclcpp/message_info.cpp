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

#include "rclcpp/message_info.hpp"

namespace rclcpp
{

MessageInfo::MessageInfo(const rmw_message_info_t & rmw_message_info)
: rmw_message_info_(rmw_message_info)
{}

MessageInfo::~MessageInfo()
{}

const rmw_message_info_t &
MessageInfo::get_rmw_message_info() const
{
  return rmw_message_info_;
}

rmw_message_info_t &
MessageInfo::get_rmw_message_info()
{
  return rmw_message_info_;
}

}  // namespace rclcpp
