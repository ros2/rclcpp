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

#ifndef RCL__DEFAULT_STATE_MACHINE_H_
#define RCL__DEFAULT_STATE_MACHINE_H_

#include <rcl_lifecycle/visibility_control.h>

#if __cplusplus
extern "C"
{
#endif

// primary states based on
// design.ros2.org/articles/node_lifecycle.html
/*static*/ const rcl_state_t LIFECYCLE_EXPORT rcl_state_unconfigured = {/*.state = */0, /*.label = */"unconfigured"};
/*static*/ const rcl_state_t LIFECYCLE_EXPORT rcl_state_inactive     = {/*.state = */1, /*.label = */"inactive"};
/*static*/ const rcl_state_t LIFECYCLE_EXPORT rcl_state_active       = {/*.state = */2, /*.label = */"active"};
/*static*/ const rcl_state_t LIFECYCLE_EXPORT rcl_state_finalized    = {/*.state = */3, /*.label = */"finalized"};
/*static*/ const rcl_state_t LIFECYCLE_EXPORT rcl_state_error        = {/*.state = */4, /*.label = */"error"};

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__DEFAULT_STATE_MACHINE_H_
