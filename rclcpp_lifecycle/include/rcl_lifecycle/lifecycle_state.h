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

#ifndef RCL__LIFECYCLE_STATE_H_
#define RCL__LIFECYCLE_STATE_H_

#if __cplusplus
extern "C"
{
#endif

//#include <stdbool.h>
//#define bool int;
// #ifdef __cplusplus
// #error WRONG COMPILER
// #endif
#ifndef __cplusplus
typedef int bool;
#define true 1
#define false 0
#endif

#include <rcl_lifecycle/visibility_control.h>
#include <rcl_lifecycle/transition_map.h>

/**
 * @brief simple definition of a state
 * @param state: integer giving the state
 * @param label: label for easy indexing
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_t
{
  const char* label;
  unsigned int index;
} rcl_state_t;

/**
 * @brief transition definition
 * @param start: rcl_state_t as a start state
 * @param goal: rcl_state_t as a goal state
 * TODO: Maybe specify callback pointer here
 * and call on_* functions directly
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_transition_t
{
  rcl_state_t transition_state;
  void* callback;
  rcl_state_t* start;
  rcl_state_t* goal;
} rcl_state_transition_t;

/**
 * @brief: statemachine object holding
 * a variable state object as current state
 * of the complete machine.
 * @param transition_map: a map object of all
 * possible transitions registered with this
 * state machine.
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_machine_t
{
  const rcl_state_t* current_state;
  rcl_transition_map_t transition_map;
} rcl_state_machine_t;


// function definitions
/*
 * @brief traverses the transition map of the given
 * state machine to find if there is a transition from the
 * current state to the specified goal state
 * @return the transition state which is valid
 * NULL if not available
 */
const rcl_state_transition_t*
LIFECYCLE_EXPORT rcl_is_valid_transition_by_index(rcl_state_machine_t* state_machine, unsigned int transition_index);
const rcl_state_transition_t*
LIFECYCLE_EXPORT rcl_is_valid_transition_by_label(rcl_state_machine_t* state_machine, const char* transition_label);

const rcl_state_transition_t*
LIFECYCLE_EXPORT rcl_get_registered_transition_by_index(rcl_state_machine_t* state_machine, unsigned int transition_state_index);
const rcl_state_transition_t*
LIFECYCLE_EXPORT rcl_get_registered_transition_by_label(rcl_state_machine_t* state_machine, const char* transition_state_label);

rcl_state_t
LIFECYCLE_EXPORT rcl_create_state(unsigned int state, char* label);

rcl_state_transition_t
LIFECYCLE_EXPORT rcl_create_transition(rcl_state_t start, rcl_state_t goal);

rcl_state_machine_t
LIFECYCLE_EXPORT rcl_get_default_state_machine();

void
LIFECYCLE_EXPORT rcl_register_callback(rcl_state_machine_t* state_machine, unsigned int state_index, unsigned int transition_index, bool(*fcn)(void));

bool
LIFECYCLE_EXPORT rcl_invoke_transition(rcl_state_machine_t* state_machine, rcl_state_t transition_index);

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__LIFECYCLE_STATE_H
