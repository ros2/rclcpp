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

#include <rcl_lifecycle/lifecycle_state.h>

#if __cplusplus
extern "C"
{
#endif

typedef struct _rcl_state_t rcl_state_t;

typedef struct _rcl_state_transition_t rcl_state_transition_t;

/**
 * @brief The actual data array
 * within the map. One array belongs
 * to one label within the map.
 */
typedef struct _data_array
{
  rcl_state_transition_t* transitions;
  unsigned int size;
} rcl_transition_array_t;

/**
 * @brief helper index for keeping
 * track of the map content
 */
typedef struct _index
{
  unsigned int index;
  const char* label;
} rcl_transition_map_index_t;

/**
 * @brief stores an array of transitions
 * index by a start state
 */
typedef struct _map
{
  //rcl_state_t* state_index;
  rcl_transition_map_index_t* state_index;
  rcl_transition_array_t* transition_arrays;
  unsigned int size;
} rcl_transition_map_t;

/**
 * @brief appends a transition in a map
 * the classification is based on the start state
 * within the given transition
 */
void rcl_append_transition(rcl_transition_map_t* m, rcl_state_transition_t transition);

/**
 * @brief gets all transitions based on a label
 * label is supposed to come from a rcl_state_t object
 */
rcl_transition_array_t* rcl_get_map_by_label(rcl_transition_map_t* m, const char* label);
/**
 * @brief gets all transitions based on a state
 * state is supposed to come from a rcl_state_t object
 */
rcl_transition_array_t* rcl_get_map_by_state(rcl_transition_map_t* m, const unsigned int state);

/**
 * @brief helper functions to print
 */
void rcl_print_transition_array(const rcl_transition_array_t* da);
void rcl_print_transition_map(const rcl_transition_map_t* m);

#if __cplusplus
}
#endif

#endif  // RCL__TRANSITION_MAP_H_
