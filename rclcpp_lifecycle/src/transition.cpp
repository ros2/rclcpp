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

#include "rclcpp_lifecycle/transition.hpp"

#include <string>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rcl_lifecycle/data_types.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

namespace rclcpp_lifecycle
{

Transition::Transition(
  uint8_t id,
  const std::string & label,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_transition_handle_(true),
  transition_handle_(nullptr)
{
  transition_handle_ = static_cast<rcl_lifecycle_transition_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));
  if (!transition_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_transition_t");
  }
  transition_handle_->start = nullptr;
  transition_handle_->goal = nullptr;

  transition_handle_->id = id;
  transition_handle_->label = rcutils_strndup(
    label.c_str(), label.size(), allocator_);
  if (!transition_handle_->label) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_transition_t");
  }
}

Transition::Transition(
  uint8_t id, const std::string & label,
  State && start, State && goal,
  rcutils_allocator_t allocator)
: Transition(id, label, allocator)
{
  transition_handle_->start = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!transition_handle_->start) {
    reset();
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  transition_handle_->start->id = start.id();
  transition_handle_->start->label =
    rcutils_strndup(start.label().c_str(), start.label().size(), allocator_);
  if (!transition_handle_->start->label) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_state_t");
  }

  transition_handle_->goal = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!transition_handle_->goal) {
    reset();
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  transition_handle_->goal->id = goal.id();
  transition_handle_->goal->label =
    rcutils_strndup(goal.label().c_str(), goal.label().size(), allocator_);
  if (!transition_handle_->goal->label) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_state_t");
  }
}

Transition::Transition(
  const rcl_lifecycle_transition_t * rcl_lifecycle_transition_handle,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_transition_handle_(false),
  transition_handle_(nullptr)
{
  if (!rcl_lifecycle_transition_handle) {
    throw std::runtime_error("rcl_lifecycle_transition_handle is null");
  }
  transition_handle_ = const_cast<rcl_lifecycle_transition_t *>(rcl_lifecycle_transition_handle);
}

Transition::~Transition()
{
  reset();
}

uint8_t
Transition::id() const
{
  if (!transition_handle_) {
    throw std::runtime_error("Error in state! Internal transition_handle_ is NULL.");
  }
  return transition_handle_->id;
}

std::string
Transition::label() const
{
  if (!transition_handle_) {
    throw std::runtime_error("Error in state! Internal transition_handle_ is NULL.");
  }
  return transition_handle_->label;
}

State
Transition::start_state() const
{
  if (!transition_handle_) {
    throw std::runtime_error("Error in state! Internal transition_handle_ is NULL.");
  }
  // State constructor throws if start pointer is null
  return State(transition_handle_->start, allocator_);
}

State
Transition::goal_state() const
{
  if (!transition_handle_) {
    throw std::runtime_error("Error in state! Internal transition_handle_ is NULL.");
  }
  // State constructor throws if goal pointer is null
  return State(transition_handle_->goal, allocator_);
}

void
Transition::reset()
{
  // can't free anything which is not owned
  if (!owns_rcl_transition_handle_) {
    transition_handle_ = nullptr;
  }

  if (!transition_handle_) {
    return;
  }

  if (transition_handle_->start) {
    if (transition_handle_->start->label) {
      allocator_.deallocate(
        const_cast<char *>(transition_handle_->start->label), allocator_.state);
      transition_handle_->start->label = nullptr;
    }
    allocator_.deallocate(transition_handle_->start, allocator_.state);
    transition_handle_->start = nullptr;
  }
  if (transition_handle_->goal) {
    if (transition_handle_->goal->label) {
      allocator_.deallocate(
        const_cast<char *>(transition_handle_->goal->label), allocator_.state);
      transition_handle_->goal->label = nullptr;
    }
    allocator_.deallocate(transition_handle_->goal, allocator_.state);
    transition_handle_->goal = nullptr;
  }
  if (transition_handle_->label) {
    allocator_.deallocate(
      const_cast<char *>(transition_handle_->label), allocator_.state);
    transition_handle_->label = nullptr;
  }
  allocator_.deallocate(transition_handle_, allocator_.state);
  transition_handle_ = nullptr;
}
}  // namespace rclcpp_lifecycle
