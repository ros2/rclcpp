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

#ifndef RCL_LIFECYCLE__RCL_LIFECYCLE_H_
#define RCL_LIFECYCLE__RCL_LIFECYCLE_H_

#if __cplusplus
extern "C"
{
#endif

#ifndef __cplusplus
  #ifndef bool
    #define bool int
    #define true 1
    #define false 0
  #endif
#endif

#include <rcl_lifecycle/visibility_control.h>
#include <rcl_lifecycle/data_types.h>
#include <rcl_lifecycle/states.h>

LIFECYCLE_EXPORT
rcl_state_machine_t
rcl_get_zero_initialized_state_machine(rcl_node_t * node_handle);

LIFECYCLE_EXPORT
rcl_ret_t
rcl_state_machine_init(rcl_state_machine_t * state_machine,
  const char* node_name, bool default_states);

LIFECYCLE_EXPORT
rcl_ret_t
rcl_state_machine_fini(rcl_state_machine_t * state_machine);

// function definitions
/*
 * @brief traverses the transition map of the given
 * state machine to find if there is a transition from the
 * current state to the specified goal state
 * @return the transition state which is valid
 * NULL if not available
 */
LIFECYCLE_EXPORT
const rcl_state_transition_t *
rcl_is_valid_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index);
LIFECYCLE_EXPORT
const rcl_state_transition_t *
rcl_is_valid_transition_by_label(rcl_state_machine_t * state_machine,
  const char * transition_label);

LIFECYCLE_EXPORT
const rcl_state_transition_t *
rcl_get_registered_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_state_index);
LIFECYCLE_EXPORT
const rcl_state_transition_t *
rcl_get_registered_transition_by_label(rcl_state_machine_t * state_machine,
  const char * transition_state_label);

LIFECYCLE_EXPORT
rcl_state_t
rcl_create_state(unsigned int state, char * label);

LIFECYCLE_EXPORT
rcl_state_transition_t
rcl_create_transition(rcl_state_t start, rcl_state_t goal);

LIFECYCLE_EXPORT
void
rcl_register_callback(rcl_state_machine_t * state_machine,
  unsigned int state_index, unsigned int transition_index, bool (* fcn)(void));

LIFECYCLE_EXPORT
bool
rcl_start_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index);

LIFECYCLE_EXPORT
bool
rcl_finish_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index, bool success);

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL_LIFECYCLE__RCL_LIFECYCLE_H_
