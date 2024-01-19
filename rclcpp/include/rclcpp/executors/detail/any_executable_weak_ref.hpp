#pragma once
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/guard_condition.hpp"

namespace rclcpp::executors
{

/**
 * A holder class for an executor entitiy
 */
struct AnyExecutableWeakRef
{
  AnyExecutableWeakRef(const rclcpp::SubscriptionBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {}

  AnyExecutableWeakRef(const rclcpp::TimerBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {}

  AnyExecutableWeakRef(const rclcpp::ServiceBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {}

  AnyExecutableWeakRef(const rclcpp::ClientBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {}

  AnyExecutableWeakRef(const rclcpp::Waitable::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {}

  AnyExecutableWeakRef(
    const rclcpp::GuardCondition::WeakPtr & p,
    const std::function<void(void)> & fun)
  : executable(p),
    handle_guard_condition_fun(fun),
    callback_group_index(-1),
    processed(!fun)
  {
    //special case, guard conditions are auto processed by waking up the wait set
    // therefore they shall never create a real executable
  }

 AnyExecutableWeakRef(const AnyExecutableWeakRef &) = delete;

 AnyExecutableWeakRef(AnyExecutableWeakRef &&) = default;

 AnyExecutableWeakRef& operator= (const AnyExecutableWeakRef &) = delete;

  std::variant<const rclcpp::SubscriptionBase::WeakPtr, const rclcpp::TimerBase::WeakPtr,
    const rclcpp::ServiceBase::WeakPtr, const rclcpp::ClientBase::WeakPtr,
    const rclcpp::Waitable::WeakPtr, const rclcpp::GuardCondition::WeakPtr> executable;

  enum ExecutableIndex
  {
    Subscription = 0,
    Timer,
    Service,
    Client,
    Waitable,
    GuardCondition,
  };

  // shared_ptr holding the rcl handle during wait
  std::shared_ptr<const void> rcl_handle_shr_ptr;

  // A function that should be executed if the executable is a guard condition and ready
  std::function<void(void)> handle_guard_condition_fun;

  // temporary callback group index of the executable
  int16_t callback_group_index;

  // Used for tracking, if the executable was already processed
  bool processed = false;
};


/**
 * A cache of AnyExecutableWeakRef
 */
struct AnyExecutableWeakRefCache
{
  std::vector<AnyExecutableWeakRef> executables;

  size_t temporaryCallbackGroupIndex = 0;

  bool cache_ditry = true;

  void regenerate(const CallbackGroupState & state)
  {
    executables.clear();
    executables.reserve(
      state.client_ptrs.size() +
      state.service_ptrs.size() +
      state.subscription_ptrs.size() +
      state.timer_ptrs.size() +
      state.waitable_ptrs.size() + 1);

    for (const rclcpp::SubscriptionBase::WeakPtr & weak_ptr: state.subscription_ptrs) {
      executables.emplace_back(weak_ptr, temporaryCallbackGroupIndex);
    }

    for (const rclcpp::ClientBase::WeakPtr & weak_ptr: state.client_ptrs) {
      executables.emplace_back(weak_ptr, temporaryCallbackGroupIndex);
    }

    for (const rclcpp::ServiceBase::WeakPtr & weak_ptr: state.service_ptrs) {
      executables.emplace_back(weak_ptr, temporaryCallbackGroupIndex);
    }

    for (const rclcpp::TimerBase::WeakPtr & weak_ptr: state.timer_ptrs) {
      executables.emplace_back(weak_ptr, temporaryCallbackGroupIndex);
    }

    for (const rclcpp::Waitable::WeakPtr & weak_ptr: state.waitable_ptrs) {
      executables.emplace_back(weak_ptr, temporaryCallbackGroupIndex);
    }

    executables.emplace_back(
      state.trigger_ptr, [this]() {
        cache_ditry = true;
      }
    );

    cache_ditry = false;
  }

};


}
