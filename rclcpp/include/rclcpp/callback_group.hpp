// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CALLBACK_GROUP_HPP_
#define RCLCPP__CALLBACK_GROUP_HPP_

#include <atomic>
#include <string>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"
#include "rcpputils/mutex.hpp"

namespace rclcpp
{

// Forward declarations for friend statement in class CallbackGroup
namespace node_interfaces
{
class NodeServices;
class NodeTimers;
class NodeTopics;
class NodeWaitables;
}  // namespace node_interfaces

enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};

class CallbackGroup
{
  friend class rclcpp::node_interfaces::NodeServices;
  friend class rclcpp::node_interfaces::NodeTimers;
  friend class rclcpp::node_interfaces::NodeTopics;
  friend class rclcpp::node_interfaces::NodeWaitables;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(CallbackGroup)

  /// Constructor for CallbackGroup.
  /**
   * Callback Groups have a type, either 'Mutually Exclusive' or 'Reentrant'
   * and when creating one the type must be specified.
   *
   * Callbacks in Reentrant Callback Groups must be able to:
   *   - run at the same time as themselves (reentrant)
   *   - run at the same time as other callbacks in their group
   *   - run at the same time as other callbacks in other groups
   *
   * Callbacks in Mutually Exclusive Callback Groups:
   *   - will not be run multiple times simultaneously (non-reentrant)
   *   - will not be run at the same time as other callbacks in their group
   *   - but must run at the same time as callbacks in other groups
   *
   * Additionally, callback groups have a property which determines whether or
   * not they are added to an executor with their associated node automatically.
   * When creating a callback group the automatically_add_to_executor_with_node
   * argument determines this behavior, and if true it will cause the newly
   * created callback group to be added to an executor with the node when the
   * Executor::add_node method is used.
   * If false, this callback group will not be added automatically and would
   * have to be added to an executor manually using the
   * Executor::add_callback_group method.
   *
   * Whether the node was added to the executor before creating the callback
   * group, or after, is irrelevant; the callback group will be automatically
   * added to the executor in either case.
   *
   * \param[in] group_type The type of the callback group.
   * \param[in] automatically_add_to_executor_with_node A boolean that
   *   determines whether a callback group is automatically added to an executor
   *   with the node with which it is associated.
   */
  RCLCPP_PUBLIC
  explicit CallbackGroup(
    CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true);

  /// Default destructor.
  RCLCPP_PUBLIC
  ~CallbackGroup();

  template<typename Function>
  rclcpp::SubscriptionBase::SharedPtr
  find_subscription_ptrs_if(Function func) const
  {
    return _find_ptrs_if_impl<rclcpp::SubscriptionBase, Function>(func, subscription_ptrs_);
  }

  template<typename Function>
  rclcpp::TimerBase::SharedPtr
  find_timer_ptrs_if(Function func) const
  {
    return _find_ptrs_if_impl<rclcpp::TimerBase, Function>(func, timer_ptrs_);
  }

  template<typename Function>
  rclcpp::ServiceBase::SharedPtr
  find_service_ptrs_if(Function func) const
  {
    return _find_ptrs_if_impl<rclcpp::ServiceBase, Function>(func, service_ptrs_);
  }

  template<typename Function>
  rclcpp::ClientBase::SharedPtr
  find_client_ptrs_if(Function func) const
  {
    return _find_ptrs_if_impl<rclcpp::ClientBase, Function>(func, client_ptrs_);
  }

  template<typename Function>
  rclcpp::Waitable::SharedPtr
  find_waitable_ptrs_if(Function func) const
  {
    return _find_ptrs_if_impl<rclcpp::Waitable, Function>(func, waitable_ptrs_);
  }

  RCLCPP_PUBLIC
  std::atomic_bool &
  can_be_taken_from();

  RCLCPP_PUBLIC
  const CallbackGroupType &
  type() const;

  RCLCPP_PUBLIC
  void collect_all_ptrs(
    std::function<void(const rclcpp::SubscriptionBase::SharedPtr &)> sub_func,
    std::function<void(const rclcpp::ServiceBase::SharedPtr &)> service_func,
    std::function<void(const rclcpp::ClientBase::SharedPtr &)> client_func,
    std::function<void(const rclcpp::TimerBase::SharedPtr &)> timer_func,
    std::function<void(const rclcpp::Waitable::SharedPtr &)> waitable_func) const;

  /// Return a reference to the 'associated with executor' atomic boolean.
  /**
   * When a callback group is added to an executor this boolean is checked
   * to ensure it has not already been added to another executor.
   * If it has not been, then this boolean is set to true to indicate it is
   * now associated with an executor.
   *
   * When the callback group is removed from the executor, this atomic boolean
   * is set back to false.
   *
   * \return the 'associated with executor' atomic boolean
   */
  RCLCPP_PUBLIC
  std::atomic_bool &
  get_associated_with_executor_atomic();

  /// Return true if this callback group should be automatically added to an executor by the node.
  /**
   * \return boolean true if this callback group should be automatically added
   *   to an executor when the associated node is added, otherwise false.
   */
  RCLCPP_PUBLIC
  bool
  automatically_add_to_executor_with_node() const;

  /// Defer creating the notify guard condition and return it.
  RCLCPP_PUBLIC
  rclcpp::GuardCondition::SharedPtr
  get_notify_guard_condition(const rclcpp::Context::SharedPtr context_ptr);

  /// Trigger the notify guard condition.
  RCLCPP_PUBLIC
  void
  trigger_notify_guard_condition();

protected:
  RCLCPP_DISABLE_COPY(CallbackGroup)

  RCLCPP_PUBLIC
  void
  add_publisher(const rclcpp::PublisherBase::SharedPtr publisher_ptr);

  RCLCPP_PUBLIC
  void
  add_subscription(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr);

  RCLCPP_PUBLIC
  void
  add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr);

  RCLCPP_PUBLIC
  void
  add_service(const rclcpp::ServiceBase::SharedPtr service_ptr);

  RCLCPP_PUBLIC
  void
  add_client(const rclcpp::ClientBase::SharedPtr client_ptr);

  RCLCPP_PUBLIC
  void
  add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr);

  RCLCPP_PUBLIC
  void
  remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept;

  CallbackGroupType type_;
  // Mutex to protect the subsequent vectors of pointers.
  mutable rcpputils::PIMutex mutex_;
  std::atomic_bool associated_with_executor_;
  std::vector<rclcpp::SubscriptionBase::WeakPtr> subscription_ptrs_;
  std::vector<rclcpp::TimerBase::WeakPtr> timer_ptrs_;
  std::vector<rclcpp::ServiceBase::WeakPtr> service_ptrs_;
  std::vector<rclcpp::ClientBase::WeakPtr> client_ptrs_;
  std::vector<rclcpp::Waitable::WeakPtr> waitable_ptrs_;
  std::atomic_bool can_be_taken_from_;
  const bool automatically_add_to_executor_with_node_;
  // defer the creation of the guard condition
  std::shared_ptr<rclcpp::GuardCondition> notify_guard_condition_ = nullptr;
  std::recursive_mutex notify_guard_condition_mutex_;

private:
  template<typename TypeT, typename Function>
  typename TypeT::SharedPtr _find_ptrs_if_impl(
    Function func, const std::vector<typename TypeT::WeakPtr> & vect_ptrs) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & weak_ptr : vect_ptrs) {
      auto ref_ptr = weak_ptr.lock();
      if (ref_ptr && func(ref_ptr)) {
        return ref_ptr;
      }
    }
    return typename TypeT::SharedPtr();
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__CALLBACK_GROUP_HPP_
