// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EXECUTOR_NOTIFY_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__EXECUTOR_NOTIFY_WAITABLE_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <set>

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{

/// Maintain a collection of guard conditions from associated nodes and callback groups
/// to signal to the executor when associated entities have changed.
class ExecutorNotifyWaitable : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ExecutorNotifyWaitable)

  // Constructor
  /**
   * \param[in] on_execute_callback Callback to execute when one of the conditions
   *   of this waitable has signaled the wait_set.
   */
  RCLCPP_PUBLIC
  explicit ExecutorNotifyWaitable(std::function<void(void)> on_execute_callback = {});

  // Destructor
  RCLCPP_PUBLIC
  ~ExecutorNotifyWaitable() override = default;

  RCLCPP_PUBLIC
  ExecutorNotifyWaitable(ExecutorNotifyWaitable & other);

  RCLCPP_PUBLIC
  ExecutorNotifyWaitable & operator=(ExecutorNotifyWaitable & other);

  /// Add conditions to the wait set
  /**
   * \param[inout] wait_set structure that conditions will be added to
   */
  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override;

  /// Check conditions against the wait set
  /**
   * \param[inout] wait_set structure that internal elements will be checked against.
   * \return true if this waitable is ready to be executed, false otherwise.
   */
  RCLCPP_PUBLIC
  bool
  is_ready(const rcl_wait_set_t & wait_set) override;

  /// Perform work associated with the waitable.
  /**
   * This will call the callback provided in the constructor.
   * \param[in] data Data to be use for the execute, if available, else nullptr.
   */
  RCLCPP_PUBLIC
  void
  execute(const std::shared_ptr<void> & data) override;

  /// Retrieve data to be used in the next execute call.
  /**
   * \return If available, data to be used, otherwise nullptr
   */
  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data() override;

  /// Take the data from an entity ID so that it can be consumed with `execute`.
  /**
   * \param[in] id ID of the entity to take data from.
   * \return If available, data to be used, otherwise nullptr
   * \sa rclcpp::Waitable::take_data_by_entity_id
   */
  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override;

  /// Set a callback to be called whenever the waitable becomes ready.
  /**
   * \param[in] callback callback to set
   * \sa rclcpp::Waitable::set_on_ready_callback
   */
  RCLCPP_PUBLIC
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /// Add a guard condition to be waited on.
  /**
   * \param[in] guard_condition The guard condition to add.
   */
  RCLCPP_PUBLIC
  void
  add_guard_condition(rclcpp::GuardCondition::WeakPtr guard_condition);

  /// Unset any callback registered via set_on_ready_callback.
  /**
   * \sa rclcpp::Waitable::clear_on_ready_callback
   */
  RCLCPP_PUBLIC
  void
  clear_on_ready_callback() override;

  /// Remove a guard condition from being waited on.
  /**
   * \param[in] weak_guard_condition The guard condition to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_guard_condition(rclcpp::GuardCondition::WeakPtr weak_guard_condition);

  /// Get the number of ready guard_conditions
  /**
   * \return The number of guard_conditions associated with the Waitable.
   */
  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

private:
  /// Callback to run when waitable executes
  std::function<void(void)> execute_callback_;

  std::mutex guard_condition_mutex_;

  std::function<void(size_t)> on_ready_callback_;

  /// The collection of guard conditions to be waited on.
  std::set<rclcpp::GuardCondition::WeakPtr,
    std::owner_less<rclcpp::GuardCondition::WeakPtr>> notify_guard_conditions_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EXECUTOR_NOTIFY_WAITABLE_HPP_
