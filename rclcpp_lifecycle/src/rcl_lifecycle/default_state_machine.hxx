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

#ifndef RCL_LIFECYCLE__DEFAULT_STATE_MACHINE_HXX_
#define RCL_LIFECYCLE__DEFAULT_STATE_MACHINE_HXX_

#include <rcl/types.h>

#include "rcl_lifecycle/data_types.h"
#include "rcl_lifecycle/visibility_control.h"

#if __cplusplus
extern "C"
{
#endif

LIFECYCLE_EXPORT
rcl_ret_t
rcl_init_default_state_machine(rcl_state_machine_t * state_machine);

#if __cplusplus
}
#endif

#endif  // RCL_LIFECYCLE__DEFAULT_STATE_MACHINE_HXX_
