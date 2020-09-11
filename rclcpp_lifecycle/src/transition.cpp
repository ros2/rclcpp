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

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

#include "rcutils/allocator.h"

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
  // zero initialize
  transition_handle_->id = 0;
  transition_handle_->label = nullptr;
  transition_handle_->start = nullptr;
  transition_handle_->goal = nullptr;

  auto ret = rcl_lifecycle_transition_init(
    transition_handle_, id, label.c_str(), nullptr, nullptr, &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
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
  // zero initialize
  transition_handle_->start->id = 0;
  transition_handle_->start->label = nullptr;

  auto ret = rcl_lifecycle_state_init(
    transition_handle_->start, start.id(), start.label().c_str(), &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  transition_handle_->goal = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!transition_handle_->goal) {
    reset();
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // zero initialize
  transition_handle_->goal->id = 0;
  transition_handle_->goal->label = nullptr;

  ret = rcl_lifecycle_state_init(
    transition_handle_->goal, goal.id(), goal.label().c_str(), &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
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

Transition::Transition(const Transition & rhs)
: allocator_(rhs.allocator_),
  owns_rcl_transition_handle_(false),
  transition_handle_(nullptr)
{
  *this = rhs;
}

Transition::~Transition()
{
  reset();
}

Transition &
Transition::operator=(const Transition & rhs)
{
  if (this == &rhs) {
    return *this;
  }

  // reset all currently used resources
  reset();

  allocator_ = rhs.allocator_;
  owns_rcl_transition_handle_ = rhs.owns_rcl_transition_handle_;

  // we don't own the handle, so we can return straight ahead
  if (!owns_rcl_transition_handle_) {
    transition_handle_ = rhs.transition_handle_;
    return *this;
  }

  // we own the handle, so we have to deep-copy the rhs object
  transition_handle_ = static_cast<rcl_lifecycle_transition_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));
  if (!transition_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_transition_t");
  }
  // zero initialize
  transition_handle_->id = 0;
  transition_handle_->label = nullptr;
  transition_handle_->start = nullptr;
  transition_handle_->goal = nullptr;

  auto ret = rcl_lifecycle_transition_init(
    transition_handle_, rhs.id(), rhs.label().c_str(), nullptr, nullptr, &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // only copy start state when available
  if (rhs.transition_handle_->start) {
    transition_handle_->start = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
    if (!transition_handle_->start) {
      reset();
      throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
    }
    // zero initialize
    transition_handle_->start->id = 0;
    transition_handle_->start->label = nullptr;

    ret = rcl_lifecycle_state_init(
      transition_handle_->start,
      rhs.start_state().id(),
      rhs.start_state().label().c_str(),
      &allocator_);
    if (ret != RCL_RET_OK) {
      reset();
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  // only copy goal state when available
  if (rhs.transition_handle_->goal) {
    transition_handle_->goal = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
    if (!transition_handle_->goal) {
      reset();
      throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
    }
    // zero initialize
    transition_handle_->goal->id = 0;
    transition_handle_->goal->label = nullptr;

    ret = rcl_lifecycle_state_init(
      transition_handle_->goal,
      rhs.goal_state().id(),
      rhs.goal_state().label().c_str(),
      &allocator_);
    if (ret != RCL_RET_OK) {
      reset();
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }
  return *this;
}

uint8_t
Transition::id() const
{
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }
  return static_cast<uint8_t>(transition_handle_->id);
}

std::string
Transition::label() const
{
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }
  return transition_handle_->label;
}

State
Transition::start_state() const
{
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }
  // State constructor throws if start pointer is null
  return State(transition_handle_->start, allocator_);
}

State
Transition::goal_state() const
{
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }
  // State constructor throws if goal pointer is null
  return State(transition_handle_->goal, allocator_);
}

void
Transition::reset() noexcept
{
  // can't free anything which is not owned
  if (!owns_rcl_transition_handle_) {
    transition_handle_ = nullptr;
  }

  if (!transition_handle_) {
    return;
  }

  auto ret = rcl_lifecycle_transition_fini(transition_handle_, &allocator_);
  allocator_.deallocate(transition_handle_, allocator_.state);
  transition_handle_ = nullptr;
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp_lifecycle"),
      "rcl_lifecycle_transition_fini did not complete successfully, leaking memory");
  }
}
}  // namespace rclcpp_lifecycle
