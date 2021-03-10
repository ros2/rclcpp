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

// The EventsExecutorCallbackData struct is what the listeners
// will use as argument when calling their callbacks from the
// RMW implementation. The listeners get a (void *) of this struct,
// and the executor is in charge to cast it back and use the data.
struct EventsExecutorCallbackData
{
  EventsExecutorCallbackData(
    EventsExecutor * _executor,
    ExecutorEvent _event)
  {
    executor = _executor;
    event = _event;
  }

  // Equal operator
  bool operator==(const EventsExecutorCallbackData & other) const
  {
    return (event.entity_id == other.event.entity_id);
  }

  // Struct members
  EventsExecutor * executor;
  ExecutorEvent event;
};

// To be able to use std::unordered_map with an EventsExecutorCallbackData
// as key, we need a hasher. We use the entity ID as hash, since it is
// unique for each EventsExecutorCallbackData object.
struct KeyHasher
{
  size_t operator()(const EventsExecutorCallbackData & key) const
  {
    return std::hash<const void *>()(key.event.entity_id);
  }
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_EVENT_TYPES_HPP_
