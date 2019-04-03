// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/intra_process_manager_impl.hpp"

#include <memory>

using rclcpp::intra_process_manager::FixedSizeString;
using rclcpp::intra_process_manager::IntraProcessManagerImplBase;

IntraProcessManagerImplBase::SharedPtr
rclcpp::intra_process_manager::create_default_impl()
{
  return std::make_shared<IntraProcessManagerImpl<>>();
}

FixedSizeString::FixedSizeString(const char * str)
{
  std::strncpy(str_, str, RMW_TOPIC_MAX_NAME_LENGTH + 1);
}

const char * FixedSizeString::c_str() const
{
  return str_;
}

bool rclcpp::intra_process_manager::operator<(
  const FixedSizeString & lhs,
  const FixedSizeString & rhs)
{
  return std::strcmp(lhs.c_str(), rhs.c_str());
}
