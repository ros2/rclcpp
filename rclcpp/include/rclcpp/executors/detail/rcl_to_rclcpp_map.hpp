#pragma once

#include "any_executable_weak_ref.hpp"

namespace rclcpp::executors
{

/// Helper class to compute the size of a waitset
struct WaitSetSize
{
  size_t subscriptions = 0;
  size_t clients = 0;
  size_t services = 0;
  size_t timers = 0;
  size_t guard_conditions = 0;
  size_t events = 0;

  void addWaitable(const rclcpp::Waitable::WeakPtr & waitable_weak_ptr)
  {
      rclcpp::Waitable::SharedPtr waitable_ptr = waitable_weak_ptr.lock();
      if (!waitable_ptr) {
        return;
      }

      subscriptions += waitable_ptr->get_number_of_ready_subscriptions();
      clients += waitable_ptr->get_number_of_ready_clients();
      services += waitable_ptr->get_number_of_ready_services();
      timers += waitable_ptr->get_number_of_ready_timers();
      guard_conditions += waitable_ptr->get_number_of_ready_guard_conditions();
      events += waitable_ptr->get_number_of_ready_events();
  }

  void addExecutableWeakPtrCache(const AnyExecutableWeakRefCache & cache)
  {
    for(const AnyExecutableWeakRef &entry : cache.executables)
    {
      switch(entry.executable.index())
      {
        case AnyExecutableWeakRef::ExecutableIndex::Subscription:
          subscriptions++;
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Timer:
          timers++;
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Service:
          services++;
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Client:
          clients++;
          break;
        case AnyExecutableWeakRef::ExecutableIndex::Waitable:
          addWaitable(std::get<const rclcpp::Waitable::WeakPtr>(entry.executable));
          break;
        case AnyExecutableWeakRef::ExecutableIndex::GuardCondition:
          guard_conditions++;
          break;
      }
    }
  }


  void addCallbackGroupState(const CallbackGroupState & state)
  {
    subscriptions += state.subscription_ptrs.size();
    clients += state.client_ptrs.size();
    services += state.service_ptrs.size();
    timers += state.timer_ptrs.size();
    // A callback group contains one guard condition
    guard_conditions++;

    for (const rclcpp::Waitable::WeakPtr & waitable_weak_ptr: state.waitable_ptrs) {
      addWaitable(waitable_weak_ptr);
    }
  }

  void clear_and_resize_wait_set(rcl_wait_set_s & wait_set) const
  {
    // clear wait set
    rcl_ret_t ret = rcl_wait_set_clear(&wait_set);
    if (ret != RCL_RET_OK) {
      exceptions::throw_from_rcl_error(ret, "Couldn't clear wait set");
    }

    // The size of waitables are accounted for in size of the other entities
    ret = rcl_wait_set_resize(
      &wait_set, subscriptions,
      guard_conditions, timers,
      clients, services, events);
    if (RCL_RET_OK != ret) {
      exceptions::throw_from_rcl_error(ret, "Couldn't resize the wait set");
    }
  }
};


struct RCLToRCLCPPMap
{
  RCLToRCLCPPMap(const WaitSetSize & wait_set_size)
  {
    subscription_map.reserve(wait_set_size.subscriptions);
    guard_conditions_map.reserve(wait_set_size.guard_conditions);
    timer_map.reserve(wait_set_size.timers);
    clients_map.reserve(wait_set_size.clients);
    services_map.reserve(wait_set_size.services);
    events_map.reserve(wait_set_size.events);
  }

  std::vector<AnyExecutableWeakRef *> subscription_map;
  std::vector<AnyExecutableWeakRef *> guard_conditions_map;
  std::vector<AnyExecutableWeakRef *> timer_map;
  std::vector<AnyExecutableWeakRef *> clients_map;
  std::vector<AnyExecutableWeakRef *> services_map;
  std::vector<AnyExecutableWeakRef *> events_map;

  bool add_to_wait_set_and_mapping(rcl_wait_set_s & ws, AnyExecutableWeakRef & executable_ref)
  {
    switch (executable_ref.executable.index()) {
      case AnyExecutableWeakRef::ExecutableIndex::Subscription:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::SubscriptionBase::WeakPtr>(
              executable_ref.executable), executable_ref);
        }
        break;
      case AnyExecutableWeakRef::ExecutableIndex::Timer:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::TimerBase::WeakPtr>(executable_ref.executable), executable_ref);
        }
        break;
      case AnyExecutableWeakRef::ExecutableIndex::Service:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::ServiceBase::WeakPtr>(executable_ref.executable),
            executable_ref);
        }
        break;
      case AnyExecutableWeakRef::ExecutableIndex::Client:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::ClientBase::WeakPtr>(executable_ref.executable), executable_ref);
        }
        break;
      case AnyExecutableWeakRef::ExecutableIndex::Waitable:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::Waitable::WeakPtr>(executable_ref.executable), executable_ref);
        }
        break;
      case AnyExecutableWeakRef::ExecutableIndex::GuardCondition:
        {
          return add_to_wait_set_and_mapping(
            ws,
            std::get<const rclcpp::GuardCondition::WeakPtr>(
              executable_ref.executable), executable_ref);
        }
        break;
    }

    return false;
  }


  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws,
    const rclcpp::SubscriptionBase::WeakPtr & sub_weak_ptr,
    AnyExecutableWeakRef & executable_ref)
  {
    const rclcpp::SubscriptionBase::SharedPtr & sub_ptr = sub_weak_ptr.lock();
    if (!sub_ptr) {
      // got deleted, we just ignore it
      return true;
    }

    auto handle_shr_ptr = sub_ptr->get_subscription_handle();
    executable_ref.rcl_handle_shr_ptr = handle_shr_ptr;
    subscription_map.emplace_back(&executable_ref);

    size_t idx;

    if (rcl_wait_set_add_subscription(
        &ws, handle_shr_ptr.get(),
        &idx) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    // verify that our mapping is correct
    assert(idx == subscription_map.size() - 1);

    return true;

  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws,
    const rclcpp::ClientBase::WeakPtr & client_weak_ptr,
    AnyExecutableWeakRef & any_exec)
  {
    const rclcpp::ClientBase::SharedPtr & client_ptr = client_weak_ptr.lock();
    if (!client_ptr) {
      // got deleted, we just ignore it from now on
      return true;
    }

    auto handle_shr_ptr = client_ptr->get_client_handle();
    any_exec.rcl_handle_shr_ptr = handle_shr_ptr;
    clients_map.emplace_back(&any_exec);

    size_t idx;

    if (rcl_wait_set_add_client(&ws, handle_shr_ptr.get(), &idx) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add client to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    // verify that our mapping is correct
    assert(idx == clients_map.size() - 1);
    return true;
  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws,
    const rclcpp::ServiceBase::WeakPtr & weak_ptr,
    AnyExecutableWeakRef & any_exec)
  {
    const rclcpp::ServiceBase::SharedPtr & shr_ptr = weak_ptr.lock();
    if (!shr_ptr) {
      // got deleted, we just ignore it from now on
      return true;
    }

    auto handle_shr_ptr = shr_ptr->get_service_handle();
    any_exec.rcl_handle_shr_ptr = handle_shr_ptr;
    services_map.emplace_back(&any_exec);

    size_t idx;

    if (rcl_wait_set_add_service(&ws, handle_shr_ptr.get(), &idx) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add service to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    // verify that our mapping is correct
    assert(idx == services_map.size() - 1);
    return true;
  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws, const rclcpp::TimerBase::WeakPtr & weak_ptr,
    AnyExecutableWeakRef & any_exec)
  {
    const rclcpp::TimerBase::SharedPtr & shr_ptr = weak_ptr.lock();
    if (!shr_ptr) {
      // got deleted, we just ignore it from now on
      return true;
    }

    auto handle_shr_ptr = shr_ptr->get_timer_handle();
    any_exec.rcl_handle_shr_ptr = handle_shr_ptr;
    timer_map.emplace_back(&any_exec);

    size_t idx;

    if (rcl_wait_set_add_timer(&ws, handle_shr_ptr.get(), &idx) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add timer to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    // verify that our mapping is correct
    assert(idx == timer_map.size() - 1);
    return true;
  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws, const rclcpp::Waitable::WeakPtr & weak_ptr,
    AnyExecutableWeakRef & any_exec)
  {
    const rclcpp::Waitable::SharedPtr & waitable_ptr = weak_ptr.lock();
    if (!waitable_ptr) {
      // got deleted, we just ignore it from now on
      return true;
    }

    const size_t old_client_index = ws.client_index;
    const size_t old_event_index = ws.event_index;
    const size_t old_guard_condition_index = ws.guard_condition_index;
    const size_t old_service_index = ws.service_index;
    const size_t old_subscription_index = ws.subscription_index;
    const size_t old_timer_index = ws.timer_index;

    waitable_ptr->add_to_wait_set(&ws);

    {
      const size_t diff = ws.client_index - old_client_index;
      for (size_t i = 0; i < diff; i++) {
        clients_map.emplace_back(&any_exec);
      }
    }

    {
      const size_t diff = ws.event_index - old_event_index;
      for (size_t i = 0; i < diff; i++) {
        events_map.emplace_back(&any_exec);
      }
    }

    {
      const size_t diff = ws.guard_condition_index -
        old_guard_condition_index;
      for (size_t i = 0; i < diff; i++) {
        guard_conditions_map.emplace_back(&any_exec);
      }
    }

    {
      const size_t diff = ws.service_index - old_service_index;
      for (size_t i = 0; i < diff; i++) {
        services_map.emplace_back(&any_exec);
      }
    }

    {
      const size_t diff = ws.subscription_index - old_subscription_index;
      for (size_t i = 0; i < diff; i++) {
        subscription_map.emplace_back(&any_exec);
      }
    }

    {
      const size_t diff = ws.timer_index - old_timer_index;
      for (size_t i = 0; i < diff; i++) {
        timer_map.emplace_back(&any_exec);
      }
    }

    return true;
  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws,
    const rclcpp::GuardCondition::WeakPtr & weak_ptr,
    AnyExecutableWeakRef & any_exec)
  {
    rclcpp::GuardCondition::SharedPtr shr_ptr = weak_ptr.lock();

    if (!shr_ptr) {
      return false;
    }

    const auto & gc = shr_ptr->get_rcl_guard_condition();

    size_t idx = 200;
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(&ws, &gc, &idx);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
    }

    guard_conditions_map.emplace_back(&any_exec);

    // verify that our mapping is correct
    assert(idx == guard_conditions_map.size() - 1);
    return true;
  }

  bool add_to_wait_set_and_mapping(
    rcl_wait_set_s & ws, AnyExecutableWeakRefCache & exec_cache,
    int16_t callback_group_idx)
  {
    for (AnyExecutableWeakRef & ref: exec_cache.executables) {
      ref.callback_group_index = callback_group_idx;
      ref.processed = false;

      if (!add_to_wait_set_and_mapping(ws, ref)) {
        return false;
      }
    }

    return true;
  }

};

}
