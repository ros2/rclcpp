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

#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rcl_lifecycle/data_types.h"

namespace rclcpp_lifecycle
{

void
copy_from(State & lhs, const State & rhs)
{
  if (lhs.owns_rcl_state_handle_) {
    auto state_handle = reinterpret_cast<rcl_lifecycle_state_t *>(
      rhs.allocator_.allocate(sizeof(rcl_lifecycle_state_t), rhs.allocator_.state));
    state_handle->id = rhs.state_handle_->id;
    state_handle->label = rcutils_strndup(
      rhs.state_handle_->label,
      strlen(rhs.state_handle_->label),
      rhs.allocator_);
    lhs.state_handle_ = state_handle;
  } else {
    lhs.state_handle_ = rhs.state_handle_;
  }
}

State::State(rcutils_allocator_t allocator)
: State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "unknown", allocator)
{}

State::State(
  uint8_t id,
  const std::string & label,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(true)
{
  if (label.empty()) {
    throw std::runtime_error("Lifecycle State cannot have an empty label.");
  }

  auto state_handle = reinterpret_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  state_handle->id = id;
  state_handle->label =
    rcutils_strndup(label.c_str(), label.size(), allocator_);

  state_handle_ = state_handle;
}

State::State(
  const rcl_lifecycle_state_t * rcl_lifecycle_state_handle,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(false)
{
  state_handle_ = rcl_lifecycle_state_handle;
}

State::State(const State & rhs)
: allocator_(rhs.allocator_),
  owns_rcl_state_handle_(rhs.owns_rcl_state_handle_)
{
  copy_from(*this, rhs);
}

State::~State()
{
  if (owns_rcl_state_handle_) {
    allocator_.deallocate(state_handle_->label, allocator_.state);
    allocator_.deallocate(const_cast<rcl_lifecycle_state_t *>(state_handle_), allocator_.state);
  }
}

State &
State::operator=(const State & rhs)
{
  if (this != &rhs) {
    if (owns_rcl_state_handle_) {
      allocator_.deallocate(state_handle_->label, allocator_.state);
      allocator_.deallocate(const_cast<rcl_lifecycle_state_t *>(state_handle_), allocator_.state);
    }
    allocator_ = rhs.allocator_;
    owns_rcl_state_handle_ = rhs.owns_rcl_state_handle_;
    copy_from(*this, rhs);
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
