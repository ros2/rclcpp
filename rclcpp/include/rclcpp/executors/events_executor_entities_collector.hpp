// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR_ENTITIES_COLLECTOR_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR_ENTITIES_COLLECTOR_HPP_

#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rclcpp/experimental/executable_list.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{
typedef std::map<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>> WeakCallbackGroupsToNodesMap;

class EventsExecutorEntitiesCollector final
  : public rclcpp::Waitable,
  public std::enable_shared_from_this<EventsExecutorEntitiesCollector>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsExecutorEntitiesCollector)

  // Constructor
  RCLCPP_PUBLIC
  EventsExecutorEntitiesCollector() = default;

  // Destructor
  RCLCPP_PUBLIC
  ~EventsExecutorEntitiesCollector();

  RCLCPP_PUBLIC
  void
  set_callback(void * executor_context, Event_callback executor_callback);

  RCLCPP_PUBLIC
  void
  execute() override;

  RCLCPP_PUBLIC
  void
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    void * executor_context,
    Event_callback executor_callback);

  RCLCPP_PUBLIC
  bool
  remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  RCLCPP_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return false;
  }

  RCLCPP_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return true;
  }

private:
  void
  set_entities_callbacks();

  /// List of weak nodes registered in the static executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;

  /// Context (associated executor)
  void * executor_context_ = nullptr;

  /// Event callback: push new events to queue
  Event_callback executor_callback_ = nullptr;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_ENTITIES_COLLECTOR_HPP_
