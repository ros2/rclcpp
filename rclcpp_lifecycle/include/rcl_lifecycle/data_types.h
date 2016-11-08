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

#ifndef RCL_LIFECYCLE__DATA_TYPES_H_
#define RCL_LIFECYCLE__DATA_TYPES_H_

#include <rcl/rcl.h>
#include "rcl_lifecycle/visibility_control.h"

#if __cplusplus
extern "C"
{
#endif

/**
 * @brief simple definition of a state
 * @param state: integer giving the state
 * @param label: label for easy indexing
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_t
{
  const char * label;
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
  void * callback;
  const rcl_state_t * start;
  const rcl_state_t * goal;
  const rcl_state_t * error;
} rcl_state_transition_t;

/**
 * @brief All transitions which are
 * valid associations for a primary state.
 * One array belongs to one primary state
 * within the map.
 */
typedef struct LIFECYCLE_EXPORT _rcl_transition_array_t
{
  rcl_state_transition_t * transitions;
  unsigned int size;
} rcl_transition_array_t;

/**
 * @brief stores an array of transitions
 * index by a start state
 */
typedef struct LIFECYCLE_EXPORT _rcl_transition_map_t
{
  // associative array between primary state
  // and their respective transitions
  // 1 ps -> 1 transition_array
  rcl_state_t * primary_states;
  rcl_transition_array_t * transition_arrays;
  unsigned int size;
} rcl_transition_map_t;

/**
 * @brief: object holding all necessary
 * ROS communication interfaces for the statemachine.
 * node_handle pointer for instantiating
 * state_publisher for publishing state changes
 * srv_get_state for getting current state
 * srv_change_state for requesting a state change
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_comm_interface_t
{
  rcl_node_t * node_handle;
  rcl_publisher_t state_publisher;
  rcl_service_t srv_get_state;
  rcl_service_t srv_change_state;
} rcl_state_comm_interface_t;

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
  const rcl_state_t * current_state;
  // Map/Associated array of registered states and transitions
  rcl_transition_map_t transition_map;
  // Communication interface into a ROS world
  rcl_state_comm_interface_t comm_interface;
} rcl_state_machine_t;

#if __cplusplus
}
#endif

#endif  // RCL_LIFECYCLE__DATA_TYPES_H_
