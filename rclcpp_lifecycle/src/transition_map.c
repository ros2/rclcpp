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

#include <rcl_lifecycle/lifecycle_state.h>
#include <rcl_lifecycle/transition_map.h>

void append_to_transition_array(rcl_transition_array_t* transition_array, rcl_state_transition_t data)
{
  // There is no transition array !?
  if (!transition_array)
  {
    return;
  }

  // we add a new transition, so increase the size
  ++transition_array->size;

  // TODO: Not sure if this is really necessary to differentiate
  // between malloc and realloc
  if (transition_array->size == 1)
  {
    transition_array->transitions = malloc(sizeof(rcl_state_transition_t));
  }
  else
  {
  transition_array->transitions
    = realloc(transition_array->transitions, transition_array->size*sizeof(rcl_state_transition_t));
  }

  // finally set the new transition to the end of the array
  transition_array->transitions[transition_array->size-1] = data;
}

void rcl_append_transition(rcl_transition_map_t* m, rcl_state_transition_t data)
{
  // the idea here is to add this transition based on the
  // start label and classify them.

  // check wether the label exits already
  for (unsigned int i=0; i<m->size; ++i)
  {
    if (strcmp(m->state_index[i].label, data.start.label) == 0)
    {
      // label exists already
      //printf("Found existing label %s. Will append new data\n", m->state_index[i].label);
      // we will append data into rcl_transition_array_t with given data.start.label
      append_to_transition_array(&(m->transition_arrays[i]), data);
      return;
    }
  }

  //printf("Label %s does not exist yet. Will add a new one\n", data.start.label);
  //printf("There are %u existing labels\n", m->size);
  // when we reach this point, the label does not exist yet
  // reallocate the index array and extend it for the new label
  rcl_transition_map_index_t idx = {.index = m->size, .label = data.start.label};
  ++m->size;
  m->state_index = realloc(m->state_index, m->size*sizeof(rcl_transition_map_index_t));
  m->state_index[idx.index] = idx;

  // reallocate the actual data array and extend it for the data
  m->transition_arrays = realloc(m->transition_arrays, m->size*sizeof(rcl_transition_array_t));
  m->transition_arrays[idx.index].size = 0;
  append_to_transition_array(&m->transition_arrays[idx.index], data);
}

rcl_transition_array_t* rcl_get_map_by_label(rcl_transition_map_t* m, const char* label)
{
  for (unsigned int i=0; i<m->size; ++i)
  {
    if (strcmp(m->state_index[i].label, label) == 0)
    {
      return &m->transition_arrays[i];
    }
  }
  return NULL;
}

rcl_transition_array_t* rcl_get_map_by_state(rcl_transition_map_t* m, const unsigned int state)
{
  for (unsigned int i=0; i<m->size; ++i)
  {
    if (m->state_index[i].index == state )
    {
      return &m->transition_arrays[i];
    }
  }
  return NULL;
}

void rcl_print_transition_array(const rcl_transition_array_t* da)
{
    for (unsigned int i=0; i<da->size; ++i)
    {
      printf("%s\t", da->transitions[i].goal.label);
    }
    printf("\n");
}

void rcl_print_transition_map(const rcl_transition_map_t* m)
{
  printf("rcl_transition_map_t size %u\n", m->size);
  for (unsigned int i=0; i<m->size; ++i)
  {
    printf("Start state %s ::: ", m->state_index[i].label);
    rcl_print_transition_array(&(m->transition_arrays[i]));
  }
}
#if __cplusplus
}
#endif
