#pragma once
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/callback_group.hpp"
#include <set>
#include <deque>
#include <type_traits>

namespace rclcpp::executors
{

class CallbackGroupScheduler;

template <class ExecutableType_T, class RclHandleType_T>
struct WeakExecutableWithRclHandle
{
  WeakExecutableWithRclHandle(const std::shared_ptr<ExecutableType_T> &executable,  std::shared_ptr<RclHandleType_T> &&rcl_handle, CallbackGroupScheduler *scheduler)
    : executable(executable), executable_ptr(executable.get()), rcl_handle(std::move(rcl_handle)), scheduler(scheduler), processed(false)
  {
  }
/*
  template <class getHandleFun>
  WeakExecutableWithRclHandle(const std::shared_ptr<ExecutableType_T> &executableIn,  getHandleFun, CallbackGroupScheduler *scheduler)
    : executable(executableIn), executable_ptr(executableIn.get()), rcl_handle(getHandleFun(executableIn)), scheduler(scheduler), processed(false)
  {
  }*/

  using RclHandleType = RclHandleType_T;
  using ExecutableType = ExecutableType_T;

//  WeakExecutableWithRclHandle(WeakExecutableWithRclHandle &&) = default;
/*
 WeakExecutableWithRclHandle& operator= (const WeakExecutableWithRclHandle &) = delete;*/


  std::weak_ptr<ExecutableType> executable;
  const ExecutableType *executable_ptr;
  std::shared_ptr<RclHandleType> rcl_handle;
  CallbackGroupScheduler *scheduler;
  bool processed;

  bool executable_alive()
  {
    auto use_cnt = rcl_handle.use_count();
    if(use_cnt == 0)
    {
      return false;
    }

    if(use_cnt <= 1)
    {
      executable.reset();
      rcl_handle.reset();
      return false;
    }

    return true;
  }

};

struct GuardConditionWithFunction
{
  GuardConditionWithFunction(rclcpp::GuardCondition::SharedPtr gc, std::function<void(void)> fun) :
    guard_condition(std::move(gc)), handle_guard_condition_fun(std::move(fun))
  {
  }

  rclcpp::GuardCondition::SharedPtr guard_condition;

  // A function that should be executed if the guard_condition is ready
  std::function<void(void)> handle_guard_condition_fun;
};

using TimerRef = WeakExecutableWithRclHandle<rclcpp::TimerBase, const rcl_timer_t>;

using SubscriberRef = WeakExecutableWithRclHandle<rclcpp::SubscriptionBase, rcl_subscription_t>;
using ClientRef = WeakExecutableWithRclHandle<rclcpp::ClientBase, rcl_client_t>;
using ServiceRef = WeakExecutableWithRclHandle<rclcpp::ServiceBase, rcl_service_t>;
using WaitableRef = WeakExecutableWithRclHandle<rclcpp::Waitable, rclcpp::Waitable>;

using AnyRef = std::variant<TimerRef, SubscriberRef, ClientRef, ServiceRef, WaitableRef, GuardConditionWithFunction>;

/// Helper class to compute the size of a waitset
struct WaitSetSize
{
  size_t subscriptions = 0;
  size_t clients = 0;
  size_t services = 0;
  size_t timers = 0;
  size_t guard_conditions = 0;
  size_t events = 0;

  void clear()
  {
    subscriptions = 0;
  clients = 0;
  services = 0;
  timers = 0;
  guard_conditions = 0;
  events = 0;
  }

  void addWaitable(const rclcpp::Waitable::SharedPtr & waitable_ptr)
  {
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

  void add(const WaitSetSize &waitset)
  {
    subscriptions += waitset.subscriptions;
    clients += waitset.clients;
    services += waitset.services;
    timers += waitset.timers;
    guard_conditions += waitset.guard_conditions;
    events += waitset.events;
//     RCUTILS_LOG_ERROR_NAMED("rclcpp", "add: waitset new size : t %lu, s %lu, c %lu, s %lu, gc %lu", timers, subscriptions, clients, services, guard_conditions);
  }

  void clear_and_resize_wait_set(rcl_wait_set_s & wait_set) const
  {
    // clear wait set
    rcl_ret_t ret = rcl_wait_set_clear(&wait_set);
    if (ret != RCL_RET_OK) {
      exceptions::throw_from_rcl_error(ret, "Couldn't clear wait set");
    }

//     RCUTILS_LOG_ERROR_NAMED("rclcpp", "Needed size of waitset : t %lu, s %lu, c %lu, s %lu, gc %lu", timers, subscriptions, clients, services, guard_conditions);
    if(wait_set.size_of_subscriptions < subscriptions || wait_set.size_of_guard_conditions < guard_conditions ||
      wait_set.size_of_timers < timers || wait_set.size_of_clients < clients || wait_set.size_of_services < services || wait_set.size_of_events < events)
    {
      // The size of waitables are accounted for in size of the other entities
      ret = rcl_wait_set_resize(
        &wait_set, subscriptions,
        guard_conditions, timers,
        clients, services, events);
      if (RCL_RET_OK != ret) {
        exceptions::throw_from_rcl_error(ret, "Couldn't resize the wait set");
      }
    }
  }
};


class InsertHelper
{


  size_t idx = 0;
  size_t final_size = 0;
  bool valid = true;
  std::vector<AnyRef> &container;
public:

  InsertHelper(std::vector<AnyRef> &con) :
    container(con)
    {
    };

    template< class RefType, class Pointer>
  inline bool exists(const Pointer &s)
  {
    final_size++;
//     std::cout << "exists called " << final_size << "times" << std::endl;
      if(valid && idx < container.size())
      {
        if(s.get() == std::get<RefType>(container[idx]).executable_ptr)
        {
//           std::cout << idx << " found reusing element" << std::endl;
          idx++;
          return true;
        }
        else
        {
//           std::cout << idx << " Not found, deleting old size " << container.size() << std::endl;
          container.erase(container.begin() + idx, container.end());
//           std::cout << idx << " Not found, deleting new size " << container.size() << std::endl;
          valid = false;
//           final_size = idx;
          return false;
        }

      }
      valid = false;

//       final_size++;

      return false;
  }

  void resize()
  {
//     std::cout << "Container has size " << container.size() << " should be " << final_size << std::endl;
    container.erase(container.begin() + final_size, container.end());
  }

};

struct WeakExecutableWithRclHandleCache
{
  WeakExecutableWithRclHandleCache(CallbackGroupScheduler &scheduler);
  WeakExecutableWithRclHandleCache();

  std::vector<AnyRef> timers;
  std::vector<AnyRef> subscribers;
  std::vector<AnyRef> clients;
  std::vector<AnyRef> services;
  std::vector<AnyRef> waitables;
  std::vector<AnyRef> guard_conditions;
//   std::vector<AnyRef> timers_tmp;
//   std::vector<AnyRef> subscribers_tmp;
//   std::vector<AnyRef> clients_tmp;
//   std::vector<AnyRef> services_tmp;
//   std::vector<AnyRef> waitables_tmp;
//
//   InsertHelper<rclcpp::SubscriptionBase> subHelper;
//   InsertHelper<rclcpp::TimerBase> timerHelper;
//   InsertHelper<rclcpp::ClientBase> clientHelper;
//   InsertHelper<rclcpp::ServiceBase> serviceHelper;
//   InsertHelper<rclcpp::Waitable> waitableHelper;


  CallbackGroupScheduler &scheduler;

  bool cache_ditry = true;

  WaitSetSize wait_set_size;

  void clear()
  {
    timers.clear();
    subscribers.clear();
    clients.clear();
    services.clear();
    waitables.clear();
    guard_conditions.clear();

    wait_set_size.clear();
  }

  template <class RefType>
  inline bool move_matching_element(const std::shared_ptr<typename RefType::ExecutableType> &exec_ptr, std::deque<std::unique_ptr<AnyRef>> &source, std::deque<std::unique_ptr<AnyRef>> &target)
  {
      //we run with the assumption, that the ordering of elements did not change since the last call
      //therefore we just delete the first elements in case we did not find anything
      while(!source.empty())
      {
        if(exec_ptr.get() == std::get<RefType>(*source.front()).executable_ptr)
        {
          target.push_back(std::move(source.front()));
          source.pop_front();
          return true;
        }

//         std::cout << "Elemente not found " << std::endl;
        source.pop_front();
      }

      return false;
  }


  void regenerate(rclcpp::CallbackGroup & callback_group)
  {
//     std::cout << "Regenerate callback_group_ptr " << &callback_group << std::endl;

//     timers.clear();
//     subscribers.clear();
//     clients.clear();
//     services.clear();
//     waitables.clear();
//     guard_conditions.clear();

    wait_set_size.clear();

    // we reserve to much memory here, this this should be fine
    if(timers.capacity() == 0)
    {
      timers.reserve(callback_group.size());
      subscribers.reserve(callback_group.size());
      clients.reserve(callback_group.size());
      services.reserve(callback_group.size());
      waitables.reserve(callback_group.size());

//       timers_tmp.reserve(callback_group.size());
//       subscribers_tmp.reserve(callback_group.size());
//       clients_tmp.reserve(callback_group.size());
//       services_tmp.reserve(callback_group.size());
//       waitables_tmp.reserve(callback_group.size());
    }

//     timers.swap(timers_tmp);
//     subscribers.swap(subscribers_tmp);
//     clients.swap(clients_tmp);
//     services.swap(services_tmp);
//     waitables.swap(waitables_tmp);

//     timers.clear();
//     subscribers.clear();
//     clients.clear();
//     services.clear();
//     waitables.clear();


    InsertHelper sub_helper(subscribers);
    InsertHelper timer_helper(timers);
    InsertHelper client_helper(clients);
    InsertHelper service_helper(services);
    InsertHelper waitable_helper(waitables);

    const auto add_sub = [this, &sub_helper](const rclcpp::SubscriptionBase::SharedPtr &s)
    {
//       std::cout << "add_sub called" << std::endl;

      if(sub_helper.exists<SubscriberRef>(s))
      {
        return;
      }

//       if(sub_idx >= 0 && sub_idx < static_cast<int>(subscribers.size()))
//       {
//         if(s.get() && s.get() == std::get<SubscriberRef>(subscribers[sub_idx]).executable_ptr)
//         {
// //           std::cout << sub_idx << " Reusiong sub" << std::endl;
//           sub_idx++;
//           return;
//         }
//         else
//         {
// //           std::cout << sub_idx << " Not found, deleting" << std::endl;
//           subscribers.erase(subscribers.begin() + sub_idx, subscribers.end());
//           sub_idx = -1;
//         }
//
//       }

//       if(move_matching_element<SubscriberRef>(s, subscribers_tmp, subscribers))
//       {
//         return;
//       }

      //element not found, add new one
//       auto handle_shr_ptr = ;
      auto &entry = subscribers.emplace_back(SubscriberRef(s, s->get_subscription_handle(), &scheduler));
      const_cast<typename std::remove_const<typename SubscriberRef::RclHandleType>::type *>(std::get<SubscriberRef>(entry).rcl_handle.get())->user_data = &entry;
//       if(subHelper.exists(s.get()))
//       {
//         return;
//       }

//       std::cout << "Adding new subscriber" << std::endl;

//       auto handle_shr_ptr = s->get_subscription_handle();
//       auto &entry = subscribers.emplace_back(SubscriberRef(s, std::move(handle_shr_ptr), &scheduler));
//       const_cast<typename std::remove_const<typename SubscriberRef::RclHandleType>::type *>(std::get<SubscriberRef>(entry).rcl_handle.get())->user_data = &entry;
    };
    const auto add_timer = [this, &timer_helper](const rclcpp::TimerBase::SharedPtr &s)
    {
      if(timer_helper.exists<TimerRef>(s))
      {
        return;
      }
//       if(move_matching_element<TimerRef>(s, timers_tmp, timers))
//       {
//         return;
//       }

//       auto handle_shr_ptr = ;
      auto &entry = timers.emplace_back(TimerRef(s, s->get_timer_handle(), &scheduler));
      const_cast<typename std::remove_const<typename TimerRef::RclHandleType>::type *>(std::get<TimerRef>(entry).rcl_handle.get())->user_data = &entry;
    };

    const auto add_client = [this, &client_helper](const rclcpp::ClientBase::SharedPtr &s)
    {
      if(client_helper.exists<ClientRef>(s))
      {
        return;
      }
//       if(move_matching_element<ClientRef>(s, clients_tmp, clients))
//       {
//         return;
//       }

      auto &entry = clients.emplace_back(ClientRef(s, s->get_client_handle(), &scheduler));
      const_cast<typename std::remove_const<typename ClientRef::RclHandleType>::type *>(std::get<ClientRef>(entry).rcl_handle.get())->user_data = &entry;
    };
    const auto add_service = [this, &service_helper](const rclcpp::ServiceBase::SharedPtr &s)
    {
      if(service_helper.exists<ServiceRef>(s))
      {
        return;
      }
//       if(move_matching_element<ServiceRef>(s, services_tmp, services))
//       {
//         return;
//       }

//       auto handle_shr_ptr = s->get_service_handle();
      auto &entry = services.emplace_back(ServiceRef(s, s->get_service_handle(), &scheduler));
      const_cast<typename std::remove_const<typename ServiceRef::RclHandleType>::type *>(std::get<ServiceRef>(entry).rcl_handle.get())->user_data = &entry;
    };

    wait_set_size.guard_conditions = 0;

    if(guard_conditions.empty())
    {
//       guard_conditions.reserve(1);

      add_guard_condition(callback_group.get_notify_guard_condition(), [this]() {
  //         RCUTILS_LOG_ERROR_NAMED("rclcpp", "GC: Callback group was changed");

          cache_ditry = true;
        });
    }
    else
    {
      wait_set_size.guard_conditions++;
    }

    const auto add_waitable = [this, &waitable_helper](const rclcpp::Waitable::SharedPtr &s)
    {
      if(waitable_helper.exists<WaitableRef>(s))
      {
        wait_set_size.addWaitable(s);
        return;
      }
//       if(move_matching_element<WaitableRef>(s, waitables_tmp, waitables))
//       {
//         wait_set_size.addWaitable(s);
//         return;
//       }

      rclcpp::Waitable::SharedPtr cpy(s);
      wait_set_size.addWaitable(s);
      waitables.emplace_back(WaitableRef(s, std::move(cpy), &scheduler));
    };


    // as we hold waitable pointers, we keep them implicitly alive. We need to drop them before the recollect
    for(AnyRef &any : waitables)
    {
      std::get<WaitableRef>(any).executable_alive();
    }

    callback_group.collect_all_ptrs(add_sub, add_service, add_client, add_timer, add_waitable);

//     subHelper.remove_missing<SubscriberRef>(subscribers);
//     timerHelper.remove_missing<TimerRef>(timers);
//     clientHelper.remove_missing<ClientRef>(clients);
//     serviceHelper.remove_missing<ServiceRef>(services);
//     waitableHelper.remove_missing<WaitableRef>(waitables);

    sub_helper.resize();
    timer_helper.resize();
    client_helper.resize();
    service_helper.resize();
    waitable_helper.resize();
// //     subscribers_tmp.clear();
//     timers_tmp.clear();
//     services_tmp.clear();
//     clients_tmp.clear();
//     waitables_tmp.clear();

    wait_set_size.timers += timers.size();
    wait_set_size.subscriptions += subscribers.size();
    wait_set_size.clients += clients.size();
    wait_set_size.services += services.size();

//     RCUTILS_LOG_ERROR_NAMED("rclcpp", "GC: regeneraetd, new size : t %lu, sub %lu, c %lu, s %lu, gc %lu, waitables %lu", wait_set_size.timers, wait_set_size.subscriptions, wait_set_size.clients, wait_set_size.services, wait_set_size.guard_conditions, waitables.size());


    cache_ditry = false;
  }

  void add_guard_condition(rclcpp::GuardCondition::SharedPtr ptr, std::function<void(void)> fun)
  {
    guard_conditions.emplace_back(GuardConditionWithFunction(std::move(ptr), std::move(fun)));

    for(auto &entry : guard_conditions)
    {
      GuardConditionWithFunction &gcwf = std::get<GuardConditionWithFunction>(entry);
      gcwf.guard_condition->get_rcl_guard_condition().user_data = &entry;
    }

    wait_set_size.guard_conditions++;
  }

  template <class RefType, class ContainerType>
  inline bool add_container_to_wait_set(const ContainerType &container, rcl_wait_set_s & ws) const
  {
    for(const auto &any_ref: container)
    {
      const RefType &ref = std::get<RefType>(*any_ref);
      if(!add_to_wait_set(ws,  ref.rcl_handle.get()))
      {
        return false;
      }
    }

    return true;
  }

  template <class RefType>
  inline bool add_container_to_wait_set(const std::vector<AnyRef> &container, rcl_wait_set_s & ws) const
  {
    for(const auto &any_ref: container)
    {
      const RefType &ref = std::get<RefType>(any_ref);
      if(!add_to_wait_set(ws,  ref.rcl_handle.get()))
      {
        return false;
      }
    }

    return true;
  }

  bool add_to_wait_set(rcl_wait_set_s & ws)
  {
    if(!add_container_to_wait_set<TimerRef>(timers, ws))
    {
      return false;
    }
    if(!add_container_to_wait_set<SubscriberRef>(subscribers, ws))
    {
      return false;
    }
    if(!add_container_to_wait_set<ServiceRef>(services, ws))
    {
      return false;
    }
    if(!add_container_to_wait_set<ClientRef>(clients, ws))
    {
      return false;
    }
    for(auto &any_ref: waitables)
    {
      WaitableRef &ref = std::get<WaitableRef>(any_ref);
      add_to_wait_set(ws,  ref);
    }

    //RCUTILS_LOG_ERROR_NAMED("rclcpp", "Adding %lu guard conditions", guard_conditions.size());

    for(auto &any_ref: guard_conditions)
    {
      const GuardConditionWithFunction &ref = std::get<GuardConditionWithFunction>(any_ref);
      add_to_wait_set(ws,  ref.guard_condition);
    }

    return true;
  }

  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    rcl_subscription_t *handle_ptr) const
  {
    if (rcl_wait_set_add_subscription(
        &ws, handle_ptr,
        nullptr) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    return true;
  }

  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    const rcl_timer_t *handle_ptr) const
  {
    if (rcl_wait_set_add_timer(&ws, handle_ptr, nullptr) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add timer to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    return true;
  }
  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    const rcl_service_t *handle_ptr) const
  {
    if (rcl_wait_set_add_service(&ws, handle_ptr, nullptr) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add service to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    return true;
  }
  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    const rclcpp::GuardCondition::SharedPtr &gc_ptr) const
  {
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(&ws, &(gc_ptr->get_rcl_guard_condition()), nullptr);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
    }

    return true;
  }


  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    const rcl_client_t *handle_ptr) const
  {
    if (rcl_wait_set_add_client(&ws, handle_ptr, nullptr) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add client to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    return true;
  }
  inline bool add_to_wait_set(
    rcl_wait_set_s & ws,
    WaitableRef &waitable)
  {
    const size_t nr_of_added_clients = ws.nr_of_added_clients;
    const size_t nr_of_added_events = ws.nr_of_added_events;
    const size_t old_nr_of_added_guard_conditions = ws.nr_of_added_guard_conditions;
    const size_t old_nr_of_added_services = ws.nr_of_added_services;
    const size_t old_nr_of_added_subscriptions = ws.nr_of_added_subscriptions;
    const size_t old_nr_of_added_timers = ws.nr_of_added_timers;

    // reset the processed flag, used to make sure, that a waitable
    // will not get added two times to the scheduler
    waitable.processed = false;
    waitable.rcl_handle->add_to_wait_set(&ws);

    {
      for (size_t i = nr_of_added_clients; i < ws.nr_of_added_clients; i++) {
        const_cast<rcl_client_t *>(ws.clients[i])->user_data = &waitable;
      }
    }

    {
      for (size_t i = nr_of_added_events; i < ws.nr_of_added_events; i++) {
        const_cast<rcl_event_t *>(ws.events[i])->user_data = &waitable;
      }
    }

    {
      for (size_t i = old_nr_of_added_guard_conditions; i < ws.nr_of_added_guard_conditions; i++) {
        const_cast<rcl_guard_condition_t *>(ws.guard_conditions[i])->user_data = &waitable;
      }
    }

    {
      for (size_t i = old_nr_of_added_services; i < ws.nr_of_added_services; i++) {
        const_cast<rcl_service_t *>(ws.services[i])->user_data = &waitable;
      }
    }

    {
      for (size_t i = old_nr_of_added_subscriptions; i < ws.nr_of_added_subscriptions; i++) {
        const_cast<rcl_subscription_t *>(ws.subscriptions[i])->user_data = &waitable;
      }
    }

    {
      for (size_t i = old_nr_of_added_timers; i < ws.nr_of_added_timers; i++) {
        const_cast<rcl_timer_t *>(ws.timers[i])->user_data = &waitable;
      }
    }

    return true;
  }

  static void collect_ready_from_waitset(rcl_wait_set_s & ws);
};



}
