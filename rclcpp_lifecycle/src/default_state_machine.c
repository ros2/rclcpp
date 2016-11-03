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
rcl_create_state(unsigned int index, char* label)
{
  rcl_state_t ret_state = {.index = index, .label = label};
  return ret_state;
}

rcl_state_transition_t
rcl_create_state_transition(rcl_state_t index, rcl_state_t goal)
{
  rcl_state_transition_t ret_transition = {.transition_index = index, NULL, .goal = goal};
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
  state_machine.current_state = &rcl_state_unconfigured;

  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_unconfigured_inactive
    = {rcl_state_configuring, NULL, rcl_state_inactive};
  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_unconfigured_finalized
    = {rcl_state_shuttingdown, NULL, rcl_state_finalized};

  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_inactive_unconfigured
    = {rcl_state_cleaningup, NULL, rcl_state_unconfigured};
  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_inactive_finalized
    = {rcl_state_shuttingdown, NULL, rcl_state_finalized};
  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_inactive_active
    = {rcl_state_activating, NULL, rcl_state_active};

  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_active_inactive
    = {rcl_state_deactivating, NULL, rcl_state_inactive};
  rcl_state_transition_t LIFECYCLE_EXPORT rcl_transition_active_finalized
    = {rcl_state_shuttingdown, NULL, rcl_state_finalized};

  // add transitions to map
  rcl_append_transition(&state_machine.transition_map, rcl_state_unconfigured, rcl_transition_unconfigured_inactive);
  rcl_append_transition(&state_machine.transition_map, rcl_state_unconfigured, rcl_transition_unconfigured_finalized);
  rcl_append_transition(&state_machine.transition_map, rcl_state_inactive, rcl_transition_inactive_unconfigured);
  rcl_append_transition(&state_machine.transition_map, rcl_state_inactive, rcl_transition_inactive_finalized);
  rcl_append_transition(&state_machine.transition_map, rcl_state_inactive, rcl_transition_inactive_active);
  rcl_append_transition(&state_machine.transition_map, rcl_state_active, rcl_transition_active_inactive);
  rcl_append_transition(&state_machine.transition_map, rcl_state_active, rcl_transition_active_finalized);

  return state_machine;
}

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__STATE_MACHINE_H_
