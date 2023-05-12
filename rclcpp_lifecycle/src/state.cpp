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

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

#include "rcutils/allocator.h"

#include "mutex_map.hpp"

namespace rclcpp_lifecycle
{
MutexMap State::state_handle_mutex_map_;

State::State(rcutils_allocator_t allocator)
: State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "unknown", allocator)
{
  state_handle_mutex_map_.add(this);
}

State::State(
  uint8_t id,
  const std::string & label,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(true),
  state_handle_(nullptr)
{
  state_handle_mutex_map_.add(this);

  if (label.empty()) {
    throw std::runtime_error("Lifecycle State cannot have an empty label.");
  }

  state_handle_ = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!state_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // zero initialize
  state_handle_->id = 0;
  state_handle_->label = nullptr;

  auto ret = rcl_lifecycle_state_init(state_handle_, id, label.c_str(), &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

State::State(
  const rcl_lifecycle_state_t * rcl_lifecycle_state_handle,
  rcutils_allocator_t allocator)
: allocator_(allocator),
  owns_rcl_state_handle_(false),
  state_handle_(nullptr)
{
  state_handle_mutex_map_.add(this);

  if (!rcl_lifecycle_state_handle) {
    throw std::runtime_error("rcl_lifecycle_state_handle is null");
  }
  state_handle_ = const_cast<rcl_lifecycle_state_t *>(rcl_lifecycle_state_handle);
}

State::State(const State & rhs)
: allocator_(rhs.allocator_),
  owns_rcl_state_handle_(false),
  state_handle_(nullptr)
{
  state_handle_mutex_map_.add(this);

  *this = rhs;
}

State::~State()
{
  reset();
  state_handle_mutex_map_.remove(this);
}

State &
State::operator=(const State & rhs)
{
  if (this == &rhs) {
    return *this;
  }

  const auto lock = std::lock_guard<std::recursive_mutex>(state_handle_mutex_map_.getMutex(this));

  // reset all currently used resources
  reset();

  allocator_ = rhs.allocator_;
  owns_rcl_state_handle_ = rhs.owns_rcl_state_handle_;

  // we don't own the handle, so we can return straight ahead
  if (!owns_rcl_state_handle_) {
    state_handle_ = rhs.state_handle_;
    return *this;
  }

  // we own the handle, so we have to deep-copy the rhs object
  state_handle_ = static_cast<rcl_lifecycle_state_t *>(
    allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!state_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // zero initialize
  state_handle_->id = 0;
  state_handle_->label = nullptr;

  auto ret = rcl_lifecycle_state_init(
    state_handle_, rhs.id(), rhs.label().c_str(), &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_state_t");
  }

  return *this;
}

uint8_t
State::id() const
{
  const auto lock = std::lock_guard<std::recursive_mutex>(state_handle_mutex_map_.getMutex(this));
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }
  return static_cast<uint8_t>(state_handle_->id);
}

std::string
State::label() const
{
  const auto lock = std::lock_guard<std::recursive_mutex>(state_handle_mutex_map_.getMutex(this));
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }
  return state_handle_->label;
}

void
State::reset() noexcept
{
  const auto lock = std::lock_guard<std::recursive_mutex>(state_handle_mutex_map_.getMutex(this));

  if (!owns_rcl_state_handle_) {
    state_handle_ = nullptr;
  }

  if (!state_handle_) {
    return;
  }

  auto ret = rcl_lifecycle_state_fini(state_handle_, &allocator_);
  allocator_.deallocate(state_handle_, allocator_.state);
  state_handle_ = nullptr;
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp_lifecycle"),
      "rcl_lifecycle_transition_fini did not complete successfully, leaking memory");
  }
}

}  // namespace rclcpp_lifecycle
