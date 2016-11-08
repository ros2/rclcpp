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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rosidl_generator_c/message_type_support.h"
#include "rosidl_generator_c/string_functions.h"
#include "std_msgs/msg/string.h"

#include "rcl_lifecycle/states.h"
#include "rcl_lifecycle/transition_map.h"

#include "default_state_machine.hxx"

#if __cplusplus
extern "C"
{
#endif

//  *INDENT-OFF*
const rcl_state_t rcl_state_unknown          = {"unknown", 0};
const rcl_state_t rcl_state_unconfigured     = {"unconfigured", 1};
const rcl_state_t rcl_state_inactive         = {"my_inactive", 2};
const rcl_state_t rcl_state_active           = {"active", 3};
const rcl_state_t rcl_state_finalized        = {"finalized", 4};

const rcl_state_t rcl_state_configuring      = {"configuring", 10};
const rcl_state_t rcl_state_cleaningup       = {"cleaningup", 11};
const rcl_state_t rcl_state_shuttingdown     = {"shuttingdown", 12};
const rcl_state_t rcl_state_activating       = {"activating", 13};
const rcl_state_t rcl_state_deactivating     = {"deactivating", 14};
const rcl_state_t rcl_state_errorprocessing  = {"errorprocessing", 15};
// *INDENT-ON*

rcl_state_t
rcl_create_state(unsigned int index, char * label)
{
  rcl_state_t ret_state = {.index = index, .label = label};
  return ret_state;
}

rcl_state_transition_t
rcl_create_state_transition(unsigned int index, const char * label)
{
  rcl_state_transition_t ret_transition = {{.index = index, .label = label},
                                           NULL, NULL, NULL, &rcl_state_errorprocessing};
  return ret_transition;
}

// default implementation as despicted on
// design.ros2.org
rcl_ret_t
rcl_init_default_state_machine(rcl_state_machine_t * state_machine)
{
  // Maybe we can directly store only pointers to states
  rcl_register_primary_state(&state_machine->transition_map, rcl_state_unconfigured);
  rcl_register_primary_state(&state_machine->transition_map, rcl_state_inactive);
  rcl_register_primary_state(&state_machine->transition_map, rcl_state_active);
  rcl_register_primary_state(&state_machine->transition_map, rcl_state_finalized);

  rcl_state_transition_t rcl_transition_configuring = rcl_create_state_transition(
    rcl_state_configuring.index, rcl_state_configuring.label);
  rcl_state_transition_t rcl_transition_shuttingdown = rcl_create_state_transition(
    rcl_state_shuttingdown.index, rcl_state_shuttingdown.label);
  rcl_state_transition_t rcl_transition_cleaningup = rcl_create_state_transition(
    rcl_state_cleaningup.index, rcl_state_cleaningup.label);
  rcl_state_transition_t rcl_transition_activating = rcl_create_state_transition(
    rcl_state_activating.index, rcl_state_activating.label);
  rcl_state_transition_t rcl_transition_deactivating = rcl_create_state_transition(
    rcl_state_deactivating.index, rcl_state_deactivating.label);

  // add transitions to map
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_unconfigured, &rcl_state_inactive, rcl_transition_configuring);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_inactive, &rcl_state_unconfigured, rcl_transition_cleaningup);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_unconfigured, &rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_inactive, &rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_inactive, &rcl_state_active, rcl_transition_activating);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_active, &rcl_state_inactive, rcl_transition_deactivating);
  rcl_register_transition_by_state(&state_machine->transition_map,
    &rcl_state_active, &rcl_state_finalized, rcl_transition_shuttingdown);

  // set to first entry
  // state_machine->current_state = &state_machine->transition_map.primary_states[0];
  state_machine->current_state = &rcl_state_unconfigured;
  return RCL_RET_OK;
}

#if __cplusplus
}
#endif  // extern "C"
