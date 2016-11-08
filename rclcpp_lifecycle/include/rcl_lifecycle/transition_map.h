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


#ifndef RCL__TRANSITION_MAP_H_
#define RCL__TRANSITION_MAP_H_

#if __cplusplus
extern "C"
{
#endif

typedef struct _rcl_state_t rcl_state_t;
typedef struct _rcl_state_transition_t rcl_state_transition_t;

/**
 * @brief All transitions which are
 * valid associations for a primary state.
 * One array belongs to one primary state
 * within the map.
 */
typedef struct LIFECYCLE_EXPORT _transition_array
{
  rcl_state_transition_t* transitions;
  unsigned int size;
} rcl_transition_array_t;

/**
 * @brief stores an array of transitions
 * index by a start state
 */
typedef struct LIFECYCLE_EXPORT _map
{
  //rcl_state_t* state_index;
  // associative array between primary state and their respective transitions
  // 1 ps -> 1 transition_array
  rcl_state_t* primary_states;
  rcl_transition_array_t* transition_arrays;
  unsigned int size;
} rcl_transition_map_t;

LIFECYCLE_EXPORT
void
rcl_register_primary_state(rcl_transition_map_t* m, rcl_state_t primary_state);

/**
 * @brief appends a transition in a map
 * the classification is based on the start state
 * within the given transition
 */
LIFECYCLE_EXPORT
void
rcl_register_transition_by_state(rcl_transition_map_t* m, rcl_state_t start, rcl_state_t goal, rcl_state_transition_t transition);

/**
 * @brief appends a transition in a map
 * the classification is based on the start state
 * within the given transition
 */
LIFECYCLE_EXPORT
void
rcl_register_transition_by_label(rcl_transition_map_t* m, const char* start_label, const char* goal_label, rcl_state_transition_t transition);

/**
 * @brief appends a transition in a map
 * the classification is based on the start state
 * within the given transition
 */
LIFECYCLE_EXPORT
void
rcl_register_transition_by_index(rcl_transition_map_t* m, unsigned int start_index, unsigned int goal_index, rcl_state_transition_t transition);

/**
 * @brief gets the registered primary state based on a label
 * @return primary state pointer, NULL if not found
 */
LIFECYCLE_EXPORT
rcl_state_t*
rcl_get_primary_state_by_label(rcl_transition_map_t* m, const char* label);
/**
 * @brief gets the registered primary state based on a index
 * @return primary state pointer, NULL if not found
 */
LIFECYCLE_EXPORT
rcl_state_t*
rcl_get_primary_state_by_index(rcl_transition_map_t* m, unsigned int index);

/**
 * @brief gets all transitions based on a label
 * label is supposed to come from a rcl_state_t object
 */
LIFECYCLE_EXPORT
rcl_transition_array_t*
rcl_get_transitions_by_label(rcl_transition_map_t* m, const char* label);
/**
 * @brief gets all transitions based on a state
 * state is supposed to come from a rcl_state_t object
 */
LIFECYCLE_EXPORT
rcl_transition_array_t*
rcl_get_transitions_by_index(rcl_transition_map_t* m, unsigned int index);

/**
 * @brief helper functions to print
 */
LIFECYCLE_EXPORT
void
rcl_print_transition_array(const rcl_transition_array_t* transition_array);
LIFECYCLE_EXPORT
void
rcl_print_transition_map(const rcl_transition_map_t* m);

#if __cplusplus
}
#endif

#endif  // RCL__TRANSITION_MAP_H_
