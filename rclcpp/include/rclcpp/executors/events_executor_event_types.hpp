// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR_EVENT_TYPES_HPP_

namespace rclcpp
{
namespace executors
{

// forward declaration of EventsExecutor to avoid circular dependency
class EventsExecutor;

enum ExecutorEventType
{
  SUBSCRIPTION_EVENT,
  SERVICE_EVENT,
  CLIENT_EVENT,
  WAITABLE_EVENT
};

struct ExecutorEvent
{
  const void * entity_id;
  ExecutorEventType type;
};

struct EventsExecutorCallbackData
{
  EventsExecutorCallbackData(
    EventsExecutor * exec,
    void * id,
    ExecutorEventType type)
  {
    executor = exec;
    entity_id = id;
    event_type = type;
  }

  // Equal operator
  bool operator==(const EventsExecutorCallbackData & other) const
  {
    return (executor == other.executor) &&
           (entity_id == other.entity_id) &&
           (event_type == other.event_type);
  }

  // Struct members
  EventsExecutor * executor;
  void * entity_id;
  ExecutorEventType event_type;
};

// To be able to use std::unordered_map with an EventsExecutorCallbackData
// as key, we need a hasher:
struct KeyHasher
{
  size_t operator()(const EventsExecutorCallbackData & k) const
  {
    return ((std::hash<EventsExecutor *>()(k.executor) ^
           (std::hash<void *>()(k.entity_id) << 1)) >> 1) ^
           (std::hash<ExecutorEventType>()(k.event_type) << 1);
  }
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
