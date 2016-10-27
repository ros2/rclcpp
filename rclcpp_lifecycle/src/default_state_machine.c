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

#ifndef RCL__STATE_MACHINE_H_
#define RCL__STATE_MACHINE_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rcl_lifecycle/lifecycle_state.h"
#include "rcl_lifecycle/default_state_machine.h"

#if __cplusplus
extern "C"
{
#endif

rcl_state_t
rcl_create_state(unsigned int state, char* label)
{
  rcl_state_t ret_state = {.state = state, .label = label};
  return ret_state;
}

rcl_state_transition_t
rcl_create_state_transition(rcl_state_t start, rcl_state_t goal)
{
  rcl_state_transition_t ret_transition = {.start = start, .goal = goal};
  return ret_transition;
}

// default implementation as despicted on
// design.ros2.org
rcl_state_machine_t
rcl_get_default_state_machine()
{
  rcl_state_machine_t state_machine;
  state_machine.transition_map.state_index = NULL;
  state_machine.transition_map.transition_arrays = NULL;
  state_machine.transition_map.size = 0;

  // set default state as unconfigured
  state_machine.current_state = rcl_state_unconfigured;

  // from unconfigured to inactive
  rcl_state_transition_t unconfigured_to_inactive = {.start = rcl_state_unconfigured, .goal = rcl_state_inactive};
  // from unconfigured to error state
  rcl_state_transition_t unconfigured_to_error    = {.start = rcl_state_unconfigured, .goal = rcl_state_error};

  // from inactive to active
  rcl_state_transition_t inactive_to_active = {.start = rcl_state_inactive, .goal = rcl_state_active};
  // from inactive to error state
  rcl_state_transition_t inactive_to_error  = {.start = rcl_state_inactive, .goal = rcl_state_error};

  // from active to finalized
  rcl_state_transition_t active_to_finalized  = {.start = rcl_state_active, .goal = rcl_state_finalized};
  // from active to inactive
  rcl_state_transition_t active_to_inactive   = {.start = rcl_state_active, .goal = rcl_state_inactive};
  // from active to error
  rcl_state_transition_t active_to_error      = {.start = rcl_state_active, .goal = rcl_state_error};

  // add transitions to map
  rcl_append_transition(&state_machine.transition_map, unconfigured_to_inactive);
  rcl_append_transition(&state_machine.transition_map, unconfigured_to_error);
  rcl_append_transition(&state_machine.transition_map, inactive_to_active);
  rcl_append_transition(&state_machine.transition_map, inactive_to_error);
  rcl_append_transition(&state_machine.transition_map, active_to_finalized);
  rcl_append_transition(&state_machine.transition_map, active_to_inactive);
  rcl_append_transition(&state_machine.transition_map, active_to_error);

  return state_machine;
}

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__STATE_MACHINE_H_
