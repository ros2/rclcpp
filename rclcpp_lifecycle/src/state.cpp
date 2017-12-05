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

#include <lifecycle_msgs/msg/state.hpp>

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rcl_lifecycle/data_types.h>

#include <string>

namespace rclcpp_lifecycle
{

State::State(rcutils_allocator_t allocator)
: State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "unknown", allocator)
{}

State::State(
  uint8_t id,
  const std::string & label,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(true),
  state_handle_(nullptr)
{
  if (label.empty()) {
    throw std::runtime_error("Lifecycle State cannot have an empty label.");
  }

  state_handle_ = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!state_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  state_handle_->id = id;
  state_handle_->label =
    rcutils_strndup(label.c_str(), label.size(), allocator_);
  if (!state_handle_->label) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_state_t");
  }
}

State::State(
  const rcl_lifecycle_state_t * rcl_lifecycle_state_handle,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(false),
  state_handle_(nullptr)
{
  if (!rcl_lifecycle_state_handle) {
    throw std::runtime_error("rcl_lifecycle_state_handle is null");
  }
  state_handle_ = const_cast<rcl_lifecycle_state_t *>(rcl_lifecycle_state_handle);
}

State::~State()
{
  reset();
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

void
State::reset()
{
  if (!owns_rcl_state_handle_) {
    state_handle_ = nullptr;
  }

  if (!state_handle_) {
    return;
  }

  if (state_handle_->label) {
    allocator_.deallocate(const_cast<char *>(state_handle_->label), allocator_.state);
    state_handle_->label = nullptr;
  }
  allocator_.deallocate(state_handle_, allocator_.state);
  state_handle_ = nullptr;
}

}  // namespace rclcpp_lifecycle
