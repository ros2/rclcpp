// Copyright 2023 iRobot Corporation.
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

#ifndef RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
#define RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_

#include <memory>

namespace rclcpp
{
namespace experimental
{
namespace executors
{

enum ExecutorEventType
{
  CLIENT_EVENT,
  SUBSCRIPTION_EVENT,
  SERVICE_EVENT,
  TIMER_EVENT,
  WAITABLE_EVENT
};

struct ExecutorEvent
{
  const void * entity_key;
  std::shared_ptr<void> data;
  int waitable_data;
  ExecutorEventType type;
  size_t num_events;
};

}  // namespace executors
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
