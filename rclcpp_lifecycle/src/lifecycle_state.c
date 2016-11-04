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

#include "rcl_lifecycle/lifecycle_state.h"

typedef rcl_state_transition_t rcl_transition;
typedef rcl_state_t rcl_state;

bool
rcl_is_valid_transition(rcl_state_machine_t* state_machine, const rcl_state_t* goal_state)
{
  rcl_transition_array_t* all_transitions = rcl_get_transitions_by_label(&state_machine->transition_map, state_machine->current_state->label);

  for (unsigned int i=0; i<all_transitions->size; ++i)
  {
    if (all_transitions->transitions[i].goal.index == goal_state->index)
    {
      return true;
    }
  }
  return false;
}

void
rcl_register_callback(rcl_state_machine_t* state_machine, unsigned int state_index, unsigned int transition_index, bool(*fcn)(void))
{
  rcl_transition_array_t* all_transitions = rcl_get_transitions_by_index(&state_machine->transition_map, state_index);

  for (unsigned int i=0; i<all_transitions->size; ++i)
  {
    if (all_transitions->transitions[i].transition_index.index == transition_index)
    {
      all_transitions->transitions[i].callback = fcn;
    }
  }
}

// maybe change directly the current state here,
// no need to that all the time inside high level language
bool
rcl_invoke_transition(rcl_state_machine_t* state_machine, rcl_state_t transition_index)
{
  rcl_transition_array_t* all_transitions = rcl_get_transitions_by_index(&state_machine->transition_map, state_machine->current_state->index);

  if (!all_transitions)
  {
    return false;
  }
  bool success = false;
  for (unsigned int i=0; i<all_transitions->size; ++i)
  {
    if (all_transitions->transitions[i].transition_index.index == transition_index.index)
    {
      ((bool(*)(void))all_transitions->transitions[i].callback)();
      success = true;
      // break here ?! would allow only one to one transitions
    }
  }
  return success;
}

#if __cplusplus
}
#endif  // extern "C"
