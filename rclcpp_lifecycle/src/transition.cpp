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

#include <lifecycle_msgs/msg/transition.hpp>
#include <rcl_lifecycle/data_types.h>

#include <string>

namespace rclcpp_lifecycle
{

Transition::Transition(uint8_t id, const std::string & label)
: owns_rcl_transition_handle_(true)
{
  auto transition_handle = new rcl_lifecycle_transition_t;
  transition_handle->id = id;
  transition_handle->label = label.c_str();

  transition_handle->start = nullptr;
  transition_handle->goal = nullptr;
  transition_handle_ = transition_handle;
}

Transition::Transition(
  uint8_t id, const std::string & label,
  State && start, State && goal)
: owns_rcl_transition_handle_(true)
{
  auto transition_handle = new rcl_lifecycle_transition_t;
  transition_handle->id = id;
  transition_handle->label = label.c_str();

  auto start_state = new rcl_lifecycle_state_t;
  start_state->id = start.id();
  start_state->label = start.label().c_str();

  auto goal_state = new rcl_lifecycle_state_t;
  goal_state->id = goal.id();
  goal_state->label = start.label().c_str();

  transition_handle->start = start_state;
  transition_handle->goal = goal_state;
  transition_handle_ = transition_handle;
}

Transition::Transition(const rcl_lifecycle_transition_t * rcl_lifecycle_transition_handle)
: owns_rcl_transition_handle_(false)
{
  transition_handle_ = rcl_lifecycle_transition_handle;
}

Transition::~Transition()
{
  if (owns_rcl_transition_handle_) {
    if (transition_handle_->start) {
      delete transition_handle_->start;
    }
    if (transition_handle_->goal) {
      delete transition_handle_->goal;
    }
    delete transition_handle_;
  }
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
  return State(
    transition_handle_->start->id,
    transition_handle_->start->label);
}

State
Transition::goal_state() const
{
  return State(
    transition_handle_->goal->id,
    transition_handle_->goal->label);
}

}  // namespace rclcpp_lifecycle
