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

void
copy_from(Transition & lhs, const Transition & rhs)
{
  if (lhs.owns_rcl_transition_handle_) {
    auto transition_handle = reinterpret_cast<rcl_lifecycle_transition_t *>(
      rhs.allocator_.allocate(sizeof(rcl_lifecycle_transition_t), rhs.allocator_.state));
    transition_handle->id = rhs.transition_handle_->id;
    transition_handle->label = rcutils_strndup(
      rhs.transition_handle_->label,
      strlen(rhs.transition_handle_->label),
      rhs.allocator_);

    // don't deep copy the start and goal states
    // because a matching is done via pointer address
    transition_handle->start = rhs.transition_handle_->start;
    transition_handle->goal = rhs.transition_handle_->goal;
    lhs.transition_handle_ = transition_handle;
  } else {
    lhs.transition_handle_ = rhs.transition_handle_;
  }
}

Transition::Transition(
  uint8_t id,
  const std::string & label,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_transition_handle_(true)
{
  auto transition_handle = reinterpret_cast<rcl_lifecycle_transition_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));
  transition_handle->id = id;
  transition_handle->label = rcutils_strndup(
    label.c_str(), label.size(), allocator_);

  transition_handle->start = nullptr;
  transition_handle->goal = nullptr;
  transition_handle_ = transition_handle;
}

Transition::Transition(
  uint8_t id, const std::string & label,
  State && start, State && goal,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_transition_handle_(true)
{
  auto transition_handle = reinterpret_cast<rcl_lifecycle_transition_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));
  transition_handle->id = id;
  transition_handle->label = rcutils_strndup(
    label.c_str(), label.size(), allocator_);

  auto start_state = reinterpret_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  start_state->id = start.id();
  start_state->label =
    rcutils_strndup(start.label().c_str(), start.label().size(), allocator_);

  auto goal_state = reinterpret_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  goal_state->id = goal.id();
  goal_state->label =
    rcutils_strndup(goal.label().c_str(), goal.label().size(), allocator_);

  transition_handle->start = start_state;
  transition_handle->goal = goal_state;
  transition_handle_ = transition_handle;
}

Transition::Transition(
  const rcl_lifecycle_transition_t * rcl_lifecycle_transition_handle,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_transition_handle_(false)
{
  transition_handle_ = rcl_lifecycle_transition_handle;
}

Transition::Transition(const Transition & rhs)
: allocator_(rhs.allocator_),
  owns_rcl_transition_handle_(rhs.owns_rcl_transition_handle_)
{
  copy_from(*this, rhs);
}

Transition::~Transition()
{
  if (owns_rcl_transition_handle_) {
    if (transition_handle_->start) {
      allocator_.deallocate(transition_handle_->start->label, allocator_.state);
      allocator_.deallocate(transition_handle_->start, allocator_.state);
    }
    if (transition_handle_->goal) {
      allocator_.deallocate(transition_handle_->goal->label, allocator_.state);
      allocator_.deallocate(transition_handle_->goal, allocator_.state);
    }
    allocator_.deallocate(transition_handle_->label, allocator_.state);
    allocator_.deallocate(
      const_cast<rcl_lifecycle_transition_t *>(transition_handle_), allocator_.state);
  }
}

Transition &
Transition::operator=(const Transition & rhs)
{
  if (this != &rhs) {
    if (owns_rcl_transition_handle_) {
      if (transition_handle_->start) {
        allocator_.deallocate(transition_handle_->start->label, allocator_.state);
        allocator_.deallocate(transition_handle_->start, allocator_.state);
      }
      if (transition_handle_->goal) {
        allocator_.deallocate(transition_handle_->goal->label, allocator_.state);
        allocator_.deallocate(transition_handle_->goal, allocator_.state);
      }
      allocator_.deallocate(transition_handle_->label, allocator_.state);
      allocator_.deallocate(
        const_cast<rcl_lifecycle_transition_t *>(transition_handle_), allocator_.state);
    }
    allocator_ = rhs.allocator_;
    owns_rcl_transition_handle_ = rhs.transition_handle_;
    copy_from(*this, rhs);
  }

  return *this;
}

uint8_t
Transition::id() const
{
  return transition_handle_->id;
}

std::string
Transition::label() const
{
  return transition_handle_->label;
}

State
Transition::start_state() const
{
  return State(transition_handle_->start);
}

State
Transition::goal_state() const
{
  return State(transition_handle_->goal);
}

}  // namespace rclcpp_lifecycle
