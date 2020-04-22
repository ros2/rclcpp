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

#include "rclcpp/any_executable.hpp"

using rclcpp::AnyExecutable;

AnyExecutable::AnyExecutable()
: subscription(nullptr),
  timer(nullptr),
  service(nullptr),
  client(nullptr),
  callback_group(nullptr),
  node_base(nullptr)
{}

AnyExecutable::~AnyExecutable()
{
  // Make sure that discarded (taken but not executed) AnyExecutable's have
  // their callback groups reset. This can happen when an executor is canceled
  // between taking an AnyExecutable and executing it.
  if (callback_group) {
    callback_group->can_be_taken_from().store(true);
  }
}
