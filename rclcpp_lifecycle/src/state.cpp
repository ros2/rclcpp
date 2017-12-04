// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/state.hpp"

#include <string>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rcl_lifecycle/data_types.h"

namespace rclcpp_lifecycle
{

void
swap(rcl_lifecycle_state_t * lhs, const rcl_lifecycle_state_t * rhs)
{
  lhs->id = rhs->id;
  lhs->label = rcutils_strndup(rhs->label, strlen(rhs->label), rcutils_get_default_allocator());
}

void
swap(State & lhs, const State & rhs)
{
  if (lhs.owns_rcl_state_handle_) {
    auto state_handle = new rcl_lifecycle_state_t;
    swap(state_handle, rhs.state_handle_);
    lhs.state_handle_ = state_handle;
  } else {
    lhs.state_handle_ = rhs.state_handle_;
  }
}

State::State()
: State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "unknown")
{}

State::State(uint8_t id, const std::string & label)
: owns_rcl_state_handle_(true)
{
  if (label.empty()) {
    throw std::runtime_error("Lifecycle State cannot have an empty label.");
  }

  auto state_handle = new rcl_lifecycle_state_t;
  state_handle->id = id;
  state_handle->label =
    rcutils_strndup(label.c_str(), label.size(), rcutils_get_default_allocator());

  state_handle_ = state_handle;
}

State::State(const rcl_lifecycle_state_t * rcl_lifecycle_state_handle)
: owns_rcl_state_handle_(false)
{
  state_handle_ = rcl_lifecycle_state_handle;
}

State::State(const State & rhs)
: owns_rcl_state_handle_(rhs.owns_rcl_state_handle_)
{
  swap(*this, rhs);
}

State::~State()
{
  if (owns_rcl_state_handle_) {
    delete state_handle_;
  }
}

State &
State::operator=(const State & rhs)
{
  if (this != &rhs) {
    owns_rcl_state_handle_ = rhs.owns_rcl_state_handle_;
    swap(*this, rhs);
  }

  return *this;
}

uint8_t
State::id() const
{
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }
  return state_handle_->id;
}

std::string
State::label() const
{
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }
  return state_handle_->label;
}

}  // namespace rclcpp_lifecycle
