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

#if __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rcl_lifecycle/transition_map.h"

void
rcl_register_primary_state(rcl_transition_map_t * m,
  rcl_state_t primary_state)
{
  if (rcl_get_primary_state_by_index(m, primary_state.index) != NULL) {
    // primary state is already registered
    fprintf(stderr, "%s:%u, Primary state %u is already registered\n",
      __FILE__, __LINE__, primary_state.index);
    return;
  }

  // add new primary state memory
  ++m->size;
  if (m->size == 1) {
    m->primary_states = malloc(m->size * sizeof(rcl_state_t));
    m->transition_arrays = malloc(m->size * sizeof(rcl_transition_array_t));
  } else {
    m->primary_states = realloc(
      m->primary_states, m->size * sizeof(rcl_state_t));
    m->transition_arrays = realloc(
      m->transition_arrays, m->size * sizeof(rcl_transition_array_t));
  }
  m->primary_states[m->size - 1] = primary_state;
  m->transition_arrays[m->size - 1].transitions = NULL;
  m->transition_arrays[m->size - 1].size = 0;  // initialize array to size 0
}

void
rcl_register_transition_by_state(rcl_transition_map_t * m,
  const rcl_state_t * start, const rcl_state_t * goal, rcl_state_transition_t transition)
{
  transition.start = start;
  transition.goal = goal;

  // TODO(karsten1987): check whether we can improve that
  rcl_transition_array_t * transition_array = rcl_get_transitions_by_index(
    m, transition.start->index);
  if (!transition_array) {
    fprintf(stderr, "%s:%u, Unable to find transition array registered for start index %u",
      __FILE__, __LINE__, transition.start->index);
    return;
  }

  // we add a new transition, so increase the size
  ++transition_array->size;
  if (transition_array->size == 1) {
    transition_array->transitions = malloc(
      transition_array->size * sizeof(rcl_state_transition_t));
  } else {
    transition_array->transitions = realloc(
      transition_array->transitions, transition_array->size * sizeof(rcl_state_transition_t));
  }
  // finally set the new transition to the end of the array
  transition_array->transitions[transition_array->size - 1] = transition;
}

void
rcl_register_transition_by_label(rcl_transition_map_t * m,
  const char * start_label, const char * goal_label, rcl_state_transition_t transition)
{
  // the idea here is to add this transition based on the
  // start label and classify them.
  const rcl_state_t * start_state = rcl_get_primary_state_by_label(m, start_label);
  if (start_state == NULL) {
    // return false here?
    return;
  }
  const rcl_state_t * goal_state = rcl_get_primary_state_by_label(m, goal_label);
  if (goal_state == NULL) {
    // return false here?
    return;
  }
  rcl_register_transition_by_state(m, start_state, goal_state, transition);
}

void
rcl_register_transition_by_index(rcl_transition_map_t * m,
  unsigned int start_index, unsigned int goal_index, rcl_state_transition_t transition)
{
  // the idea here is to add this transition based on the
  // start label and classify them.
  const rcl_state_t * start_state = rcl_get_primary_state_by_index(m, start_index);
  if (start_state == NULL) {
    // return false here?
    return;
  }
  const rcl_state_t * goal_state = rcl_get_primary_state_by_index(m, goal_index);
  if (goal_state == NULL) {
    // return false here?
    return;
  }
  rcl_register_transition_by_state(m, start_state, goal_state, transition);
}

rcl_state_t *
rcl_get_primary_state_by_label(rcl_transition_map_t * m,
  const char * label)
{
  for (unsigned int i = 0; i < m->size; ++i) {
    if (m->primary_states[i].label == label) {
      return &m->primary_states[i];
    }
  }
  return NULL;
}

rcl_state_t *
rcl_get_primary_state_by_index(rcl_transition_map_t * m,
  unsigned int index)
{
  for (unsigned int i = 0; i < m->size; ++i) {
    if (m->primary_states[i].index == index) {
      return &m->primary_states[i];
    }
  }
  return NULL;
}

rcl_transition_array_t *
rcl_get_transitions_by_label(rcl_transition_map_t * m,
  const char * label)
{
  for (unsigned int i = 0; i < m->size; ++i) {
    if (strcmp(m->primary_states[i].label, label) == 0) {
      return &m->transition_arrays[i];
    }
  }
  return NULL;
}

rcl_transition_array_t *
rcl_get_transitions_by_index(rcl_transition_map_t * m,
  unsigned int index)
{
  for (unsigned int i = 0; i < m->size; ++i) {
    if (m->primary_states[i].index == index) {
      return &m->transition_arrays[i];
    }
  }
  return NULL;
}

void
rcl_print_transition_array(const rcl_transition_array_t * ta)
{
  for (unsigned int i = 0; i < ta->size; ++i) {
    printf("%s(%u)(%s)\t", ta->transitions[i].transition_state.label,
      ta->transitions[i].transition_state.index,
      (ta->transitions[i].callback == NULL) ? " " : "x");
  }
  printf("\n");
}

void
rcl_print_transition_map(const rcl_transition_map_t * m)
{
  printf("rcl_transition_map_t size %u\n", m->size);
  for (unsigned int i = 0; i < m->size; ++i) {
    printf("Start state %s(%u) ::: ", m->primary_states[i].label,
      m->primary_states[i].index);
    rcl_print_transition_array(&(m->transition_arrays[i]));
  }
}

#if __cplusplus
}
#endif
