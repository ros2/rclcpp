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
const rcl_state_t LIFECYCLE_EXPORT rcl_state_unconfigured = {"unconfigured", 0};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_inactive = {"inactive", 1};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_active = {"active", 2};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_finalized = {"finalized", 3};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_configuring = {"configuring", 4};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_cleaningup = {"cleaningup", 5};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_shuttingdown = {"shuttingdown", 6};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_activating = {"activating", 7};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_deactivating = {"deactivating", 8};
const rcl_state_t LIFECYCLE_EXPORT rcl_state_errorprocessing = {"errorprocessing", 9};

#if __cplusplus
}
#endif  // extern "C"

#endif  // RCL__DEFAULT_STATE_MACHINE_H_
