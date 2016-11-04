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
rcl_create_state_transition(unsigned int index, const char* label)
{
  rcl_state_transition_t ret_transition = {{.index = index, .label = label}, NULL, NULL, NULL};
  return ret_transition;
}

// default implementation as despicted on
// design.ros2.org
rcl_state_machine_t
rcl_get_default_state_machine()
{
  rcl_state_machine_t state_machine;
  state_machine.transition_map.primary_states = NULL;
  state_machine.transition_map.transition_arrays = NULL;
  state_machine.transition_map.size = 0;

  // set default state as unconfigured
  //state_machine.current_state = &rcl_state_unconfigured;

  rcl_state_transition_t rcl_transition_configuring
    = rcl_create_state_transition(rcl_state_configuring.index, rcl_state_configuring.label);
  rcl_state_transition_t rcl_transition_shuttingdown
    = rcl_create_state_transition(rcl_state_shuttingdown.index, rcl_state_shuttingdown.label);
  rcl_state_transition_t rcl_transition_cleaningup
    = rcl_create_state_transition(rcl_state_cleaningup.index, rcl_state_cleaningup.label);
  rcl_state_transition_t rcl_transition_activating
    = rcl_create_state_transition(rcl_state_activating.index, rcl_state_activating.label);
  rcl_state_transition_t rcl_transition_deactivating
    = rcl_create_state_transition(rcl_state_deactivating.index, rcl_state_deactivating.label);;

  rcl_register_primary_state(&state_machine.transition_map, rcl_state_unconfigured);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_inactive);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_active);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_finalized);

  // add transitions to map
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_unconfigured, rcl_state_inactive, rcl_transition_configuring);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_inactive, rcl_state_unconfigured, rcl_transition_cleaningup);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_unconfigured, rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_inactive, rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_inactive, rcl_state_active, rcl_transition_activating);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_active, rcl_state_inactive, rcl_transition_deactivating);
  rcl_register_transition_by_state(&state_machine.transition_map, rcl_state_active, rcl_state_finalized, rcl_transition_shuttingdown);

  state_machine.current_state = &state_machine.transition_map.primary_states[0]; // set to first entry
  return state_machine;
}

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__STATE_MACHINE_H_
