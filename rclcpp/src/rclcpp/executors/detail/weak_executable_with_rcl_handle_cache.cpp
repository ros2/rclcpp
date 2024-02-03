#include <rclcpp/executors/detail/weak_executable_with_rcl_handle.hpp>
#include <rclcpp/executors/cbg_executor.hpp>
namespace rclcpp::executors
{

WeakExecutableWithRclHandleCache::WeakExecutableWithRclHandleCache(CallbackGroupScheduler &scheduler) : scheduler(scheduler)
{
}

WeakExecutableWithRclHandleCache::WeakExecutableWithRclHandleCache() : scheduler(*static_cast<CallbackGroupScheduler *>(nullptr))
{
}


template <class RefType, class RclType>
inline void add_to_scheduler(const RclType *entry, rcl_wait_set_s & ws)
{
    if(entry)
    {
        AnyRef *any_ref = static_cast<AnyRef *>(entry->user_data);

//         if(RefType *ref_ptr = std::get_if<RefType>(any_ref))
//         {
        if(any_ref->index() != 4)
        {
            RefType &ref_ptr = std::get<RefType>(*any_ref);
//             RCUTILS_LOG_ERROR_NAMED("rclcpp", "Found ready event of type %s", typeid(ref_ptr->executable).name());

            ref_ptr.scheduler->add_ready_executable(ref_ptr);
        }
        else
        {
            WaitableRef &ref = std::get<WaitableRef>(*any_ref);

//             RCUTILS_LOG_ERROR_NAMED("rclcpp", "Found ready waitable");

            if(ref.processed)
            {
                return;
            }

            ref.processed = true;

            if(ref.rcl_handle->is_ready(&ws))
            {
                ref.scheduler->add_ready_executable(ref);
            }
        }
    }

}

void WeakExecutableWithRclHandleCache::collect_ready_from_waitset(rcl_wait_set_s & ws)
{
//     RCUTILS_LOG_ERROR_NAMED("rclcpp", "collect_ready_from_waitset Waitset has %lu timers", ws.nr_of_added_timers);

//     return;

    for (size_t i = 0; i < ws.nr_of_added_timers; i++) {
        add_to_scheduler<TimerRef>(ws.timers[i], ws);
    }
    for (size_t i = 0; i < ws.nr_of_added_subscriptions; i++) {
        add_to_scheduler<SubscriberRef>(ws.subscriptions[i], ws);
    }
    for (size_t i = 0; i < ws.nr_of_added_clients; i++) {
        add_to_scheduler<ClientRef>(ws.clients[i], ws);
    }
    for (size_t i = 0; i < ws.nr_of_added_services; i++) {
        add_to_scheduler<ServiceRef>(ws.services[i], ws);
    }
    for (size_t i = 0; i < ws.nr_of_added_events; i++) {
        add_to_scheduler<WaitableRef>(ws.events[i], ws);
    }
    for (size_t i = 0; i < ws.nr_of_added_guard_conditions; i++) {
        const auto &entry = ws.guard_conditions[i];
        if(!entry)
        {
            continue;
        }

        AnyRef *any_ref = static_cast<AnyRef *>(entry->user_data);

        if(GuardConditionWithFunction *ref_ptr = std::get_if<GuardConditionWithFunction>(any_ref))
        {
//             RCUTILS_LOG_ERROR_NAMED("rclcpp", "Found ready GuardConditionWithFunction");

            // if it is a guard condition, see if we have a function
            // attached to it and execute it
            if(ref_ptr->handle_guard_condition_fun)
            {
//                 RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing GuardConditionWithFunction");
                ref_ptr->handle_guard_condition_fun();
            }
        }
        else
        {
//             RCUTILS_LOG_ERROR_NAMED("rclcpp", "Found ready GuardCondition of waitable");
            WaitableRef &ref = std::get<WaitableRef>(*any_ref);

            if(ref.processed)
            {
                return;
            }

            ref.processed = true;

            if(ref.rcl_handle->is_ready(&ws))
            {
                ref.scheduler->add_ready_executable(ref);
            }
        }
    }
}
}
