#pragma once
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/executors/callback_group_state.hpp"

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
  {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr->get_subscription_handle();
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }

  AnyExecutableWeakRef(const rclcpp::TimerBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr->get_timer_handle();
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }

  AnyExecutableWeakRef(const rclcpp::ServiceBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr->get_service_handle();
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }

  AnyExecutableWeakRef(const rclcpp::ClientBase::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr->get_client_handle();
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }

  AnyExecutableWeakRef(const rclcpp::Waitable::WeakPtr & p, int16_t callback_group_index)
  : executable(p),
    callback_group_index(callback_group_index)
  {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr;
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }

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

    {
    if(auto shr = p.lock())
    {
      rcl_handle_shr_ptr = shr;
    }
    else
    {
      rcl_handle_shr_ptr = std::monostate();
    }
  }


  }

  /**
   * Checks, if the executable still exists, or if was deleted
   */
  bool executable_alive()
  {
    auto check_valid = [this] (const auto &shr_ptr)
    {
      auto use_cnt = shr_ptr.use_count();
       if(use_cnt <= 1)
       {
         rcl_handle_shr_ptr = std::monostate();
         return false;
       }

       return true;
    };

    switch(rcl_handle_shr_ptr.index()) {
      case AnyExecutableWeakRef::ExecutableIndex::Subscription:
          {
            return check_valid(std::get<std::shared_ptr<rcl_subscription_t>>(rcl_handle_shr_ptr));
          }
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Timer:
          {
            return check_valid(std::get<std::shared_ptr<const rcl_timer_t>>(rcl_handle_shr_ptr));
          }
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Service:
          {
            return check_valid(std::get<std::shared_ptr<rcl_service_t>>(rcl_handle_shr_ptr));
          }
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Client:
          {
            return check_valid(std::get<std::shared_ptr<rcl_client_t>>(rcl_handle_shr_ptr));
          }
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Waitable:
          {
            return check_valid(std::get<std::shared_ptr<rclcpp::Waitable>>(rcl_handle_shr_ptr));
          }
          break;
        case AnyExecutableWeakRef::ExecutableIndex::GuardCondition:
          {
            return check_valid(std::get<std::shared_ptr<rclcpp::GuardCondition>>(rcl_handle_shr_ptr));
          }
        case AnyExecutableWeakRef::ExecutableIndex::Deleted:
          {
            return false;
          }
          break;

      }
      return false;
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
    Deleted,
  };

  // shared_ptr holding the rcl handle during wait

  using RclHandleVariant = std::variant<std::shared_ptr<rcl_subscription_t>, std::shared_ptr<const rcl_timer_t>,
    std::shared_ptr<rcl_service_t>, std::shared_ptr<rcl_client_t>,
    rclcpp::Waitable::SharedPtr, rclcpp::GuardCondition::SharedPtr, std::monostate>;

  RclHandleVariant rcl_handle_shr_ptr;

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
