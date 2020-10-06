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

#include <list>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{

class EventsExecutorEntitiesCollector final : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsExecutorEntitiesCollector)

  using TimerFn = std::function<void (const rclcpp::TimerBase::SharedPtr & timer)>;
  using ClearTimersFn = std::function<void (void)>;

  // Constructor
  RCLCPP_PUBLIC
  EventsExecutorEntitiesCollector() = default;

  // Destructor
  RCLCPP_PUBLIC
  ~EventsExecutorEntitiesCollector();

  RCLCPP_PUBLIC
  void
  set_callbacks(
    void * executor_context,
    Event_callback executor_callback,
    TimerFn push_timer,
    TimerFn clear_timer,
    ClearTimersFn clear_all_timers);

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
  void
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

  /// List of weak nodes registered in the events executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;

  /// Context (associated executor)
  void * executor_context_ = nullptr;

  /// Events callback
  Event_callback executor_callback_ = nullptr;

  /// Function pointers to push and clear timers from the timers heap
  TimerFn push_timer_ = nullptr;
  TimerFn clear_timer_ = nullptr;
  ClearTimersFn clear_all_timers_ = nullptr;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_ENTITIES_COLLECTOR_HPP_
