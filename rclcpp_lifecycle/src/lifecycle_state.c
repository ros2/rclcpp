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

#if __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <string.h>

#include "rcl_lifecycle/lifecycle_state.h"

const rcl_state_transition_t*
rcl_is_valid_transition(rcl_state_machine_t* state_machine, const rcl_state_t* goal_state)
{
  unsigned int current_index = state_machine->current_state->index;
  const rcl_transition_array_t* valid_transitions = rcl_get_transitions_by_index(&state_machine->transition_map, current_index);
  for (unsigned int i=0; i<valid_transitions->size; ++i)
  {
    if (valid_transitions->transitions[i].goal->index == goal_state->index)
    {
      return &valid_transitions->transitions[i];
    }
  }
  return NULL;
}

const rcl_state_transition_t*
rcl_get_registered_transition_by_index(rcl_state_machine_t* state_machine, unsigned int transition_state_index)
{
  // extensive search approach
  // TODO(karsten1987) can be improved by having a separate array for "registered transition"
  const rcl_transition_map_t* map = &state_machine->transition_map;
  for (unsigned int i=0; i<map->size; ++i)
  {
    for (unsigned int j=0; j<map->transition_arrays[i].size; ++j)
    {
      if (map->transition_arrays[i].transitions[j].transition_state.index == transition_state_index)
      {
        return &map->transition_arrays[i].transitions[j];
      }
    }
  }
  return NULL;
}

const rcl_state_transition_t*
rcl_get_registered_transition_by_label(rcl_state_machine_t* state_machine, const char* transition_state_label)
{
  // extensive search approach
  // TODO(karsten1987) can be improved by having a separate array for "registered transition"
  const rcl_transition_map_t* map = &state_machine->transition_map;
  for (unsigned int i=0; i<map->size; ++i)
  {
    for (unsigned int j=0; j<map->transition_arrays[i].size; ++j)
    {
      if (strcmp(map->transition_arrays[i].transitions[j].transition_state.label, transition_state_label) == 0)
      {
        return &map->transition_arrays[i].transitions[j];
      }
    }
  }
  return NULL;
}

void
rcl_register_callback(rcl_state_machine_t* state_machine, unsigned int state_index, unsigned int transition_index, bool(*fcn)(void))
{
  rcl_transition_array_t* transitions = rcl_get_transitions_by_index(&state_machine->transition_map, state_index);
  for (unsigned int i=0; i<transitions->size; ++i)
  {
    if (transitions->transitions[i].transition_state.index == transition_index)
    {
      transitions->transitions[i].callback = fcn;
    }
  }
}

// maybe change directly the current state here,
// no need to that all the time inside high level language
bool
rcl_invoke_transition(rcl_state_machine_t* state_machine, rcl_state_t transition_index)
{
  unsigned int current_index = state_machine->current_state->index;
  rcl_transition_array_t* transitions = rcl_get_transitions_by_index(&state_machine->transition_map, current_index);

  if (transitions == NULL)
  {
    return false;
  }
  bool success = false;
  for (unsigned int i=0; i<transitions->size; ++i)
  {
    if (transitions->transitions[i].transition_state.index == transition_index.index)
    {
      ((bool(*)(void))transitions->transitions[i].callback)();
      success = true;
      // break here ?! would allow only one to one transitions
    }
  }
  return success;
}

#if __cplusplus
}
#endif  // extern "C"
