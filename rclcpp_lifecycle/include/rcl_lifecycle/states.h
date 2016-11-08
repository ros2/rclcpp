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

#ifndef RCL_LIFECYCLE__STATES_H_
#define RCL_LIFECYCLE__STATES_H_

#include <rcl_lifecycle/visibility_control.h>
#include <rcl_lifecycle/data_types.h>

#if __cplusplus
extern "C"
{
#endif

// primary states based on
// design.ros2.org/articles/node_lifecycle.html
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_unknown;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_unconfigured;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_inactive;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_active;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_finalized;

LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_configuring;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_cleaningup;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_shuttingdown;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_activating;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_deactivating;
LIFECYCLE_EXPORT extern const rcl_state_t rcl_state_errorprocessing;

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL_LIFECYCLE__STATES_H_
