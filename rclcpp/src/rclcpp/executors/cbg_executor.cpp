// Copyright 2024 Cellumation GmbH
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

#include "rclcpp/executors/cbg_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include <inttypes.h>
// #include "rclcpp/executors/detail/rcl_to_rclcpp_map.hpp"
// #include "rclcpp/executors/detail/any_executable_weak_ref.hpp"

namespace rclcpp::executors
{

CBGExecutor::CBGExecutor(
    const rclcpp::ExecutorOptions & options,
    size_t number_of_threads,
    std::chrono::nanoseconds next_exec_timeout)
    : next_exec_timeout_(next_exec_timeout),

      spinning(false),
      interrupt_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
      shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
      context_(options.context),
      global_executable_cache(std::make_unique<WeakExecutableWithRclHandleCache>()),
      nodes_executable_cache(std::make_unique<WeakExecutableWithRclHandleCache>())
{

    global_executable_cache->add_guard_condition(
        interrupt_guard_condition_,
        std::function<void(void)>());
    global_executable_cache->add_guard_condition(
        shutdown_guard_condition_, [this] ()
    {
        work_ready_conditional.notify_all();
    });


    number_of_threads_ = number_of_threads > 0 ?
                         number_of_threads :
                         std::max(std::thread::hardware_concurrency(), 2U);

    shutdown_callback_handle_ = context_->add_on_shutdown_callback(
    [weak_gc = std::weak_ptr<rclcpp::GuardCondition> {shutdown_guard_condition_}]() {
        auto strong_gc = weak_gc.lock();
        if (strong_gc) {
            strong_gc->trigger();
        }
    });

    rcl_allocator_t allocator = options.memory_strategy->get_allocator();

    rcl_ret_t ret = rcl_wait_set_init(
                        &wait_set_,
                        0, 0, 0, 0, 0, 0,
                        context_->get_rcl_context().get(),
                        allocator);
    if (RCL_RET_OK != ret) {
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp",
            "failed to create wait set: %s", rcl_get_error_string().str);
        rcl_reset_error();
        exceptions::throw_from_rcl_error(ret, "Failed to create wait set in Executor constructor");
    }

}

CBGExecutor::~CBGExecutor()
{

    work_ready_conditional.notify_all();

    std::vector<node_interfaces::NodeBaseInterface::WeakPtr> added_nodes_cpy;
    {
        std::lock_guard lock{added_nodes_mutex_};
        added_nodes_cpy = added_nodes;
    }

    for (const node_interfaces::NodeBaseInterface::WeakPtr & node_weak_ptr : added_nodes_cpy) {
        const node_interfaces::NodeBaseInterface::SharedPtr & node_ptr = node_weak_ptr.lock();
        if (node_ptr) {
            remove_node(node_ptr, false);
        }
    }

    std::vector<rclcpp::CallbackGroup::WeakPtr> added_cbgs_cpy;
    {
        std::lock_guard lock{added_callback_groups_mutex_};
        added_cbgs_cpy = added_callback_groups;
    }

    for (const auto & weak_ptr : added_cbgs_cpy) {
        auto shr_ptr = weak_ptr.lock();
        if (shr_ptr) {
            remove_callback_group(shr_ptr, false);
        }
    }

    // Remove shutdown callback handle registered to Context
    if (!context_->remove_on_shutdown_callback(shutdown_callback_handle_)) {
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp",
            "failed to remove registered on_shutdown callback");
        rcl_reset_error();
    }

}

void CallbackGroupScheduler::clear_and_prepare(const size_t max_timer, const size_t max_subs, const size_t max_services, const size_t max_clients, const size_t max_waitables)
{
    ready_timers.clear_and_prepare(max_timer);
    ready_subscriptions.clear_and_prepare(max_subs);
    ready_services.clear_and_prepare(max_services);
    ready_clients.clear_and_prepare(max_clients);
    ready_waitables.clear_and_prepare(max_waitables);
}

bool CallbackGroupScheduler::execute_unprocessed_executable_until(const std::chrono::time_point<std::chrono::steady_clock> &stop_time, enum Priorities for_priority)
{
    switch (for_priority) {
    case Client:
        return ready_clients.execute_unprocessed_executable_until(stop_time);
        break;
    case Service:
        return ready_services.execute_unprocessed_executable_until(stop_time);
        break;
    case Subscription:
        return ready_subscriptions.execute_unprocessed_executable_until(stop_time);
        break;
    case Timer:
        return ready_timers.execute_unprocessed_executable_until(stop_time);
        break;
    case Waitable:
        return ready_waitables.execute_unprocessed_executable_until(stop_time);
        break;
    }
    return false;
}

bool CallbackGroupScheduler::get_unprocessed_executable(
    AnyExecutable & any_executable,
    enum Priorities for_priority)
{
    switch (for_priority) {
    case Client:
        return ready_clients.get_unprocessed_executable(any_executable);
        break;
    case Service:
        return ready_services.get_unprocessed_executable(any_executable);
        break;
    case Subscription:
        return ready_subscriptions.get_unprocessed_executable(any_executable);
        break;
    case Timer:
        return ready_timers.get_unprocessed_executable(any_executable);
        break;
    case Waitable:
        return ready_waitables.get_unprocessed_executable(any_executable);
        break;
    }
    return false;
}

bool CallbackGroupScheduler::has_unprocessed_executables()
{
    return ready_subscriptions.has_unprocessed_executables() ||
           ready_timers.has_unprocessed_executables() ||
           ready_clients.has_unprocessed_executables() ||
           ready_services.has_unprocessed_executables() ||
           ready_waitables.has_unprocessed_executables();
}


void CallbackGroupScheduler::add_ready_executable(SubscriberRef & executable)
{
    ready_subscriptions.add_ready_executable(executable);
}
void CallbackGroupScheduler::add_ready_executable(ServiceRef & executable)
{
    ready_services.add_ready_executable(executable);
}
void CallbackGroupScheduler::add_ready_executable(TimerRef & executable)
{
    ready_timers.add_ready_executable(executable);
}
void CallbackGroupScheduler::add_ready_executable(ClientRef & executable)
{
    ready_clients.add_ready_executable(executable);
}
void CallbackGroupScheduler::add_ready_executable(WaitableRef & executable)
{
    ready_waitables.add_ready_executable(executable);
}

bool CBGExecutor::execute_ready_executables_until(const std::chrono::time_point<std::chrono::steady_clock> &stop_time)
{
    bool found_work = false;

    for (size_t i = CallbackGroupScheduler::Priorities::Timer;
            i <= CallbackGroupScheduler::Priorities::Waitable; i++)
    {
        CallbackGroupScheduler::Priorities cur_prio(static_cast<CallbackGroupScheduler::Priorities>(i));

        for (CallbackGroupData & cbg_with_data : callback_groups)
        {
            rclcpp::AnyExecutable any_executable;
            if (cbg_with_data.scheduler->execute_unprocessed_executable_until(stop_time, cur_prio)) {
                if(std::chrono::steady_clock::now() >= stop_time)
                {
                    return true;
                }
            }
        }
    }
    return found_work;
}


bool CBGExecutor::get_next_ready_executable(AnyExecutable & any_executable)
{
    if(!spinning.load())
    {
      return false;
    }

    std::lock_guard g(callback_groups_mutex);
//     RCUTILS_LOG_ERROR_NAMED("rclcpp", "get_next_ready_executable");

    struct ReadyCallbacksWithSharedPtr
    {
        CallbackGroupData * data;
        rclcpp::CallbackGroup::SharedPtr callback_group;
        bool ready = true;
    };

    std::vector<ReadyCallbacksWithSharedPtr> ready_callbacks;
    ready_callbacks.reserve(callback_groups.size());


    for (auto it = callback_groups.begin(); it != callback_groups.end(); ) {
        CallbackGroupData & cbg_with_data(*it);

        ReadyCallbacksWithSharedPtr e;
        e.callback_group = cbg_with_data.callback_group.lock();
        if (!e.callback_group) {
            it = callback_groups.erase(it);
            continue;
        }

        if (e.callback_group->can_be_taken_from().load()) {
            e.data = &cbg_with_data;
            ready_callbacks.push_back(std::move(e));
        }

        it++;
    }

    bool found_work = false;

    for (size_t i = CallbackGroupScheduler::Priorities::Timer;
            i <= CallbackGroupScheduler::Priorities::Waitable; i++)
    {
        CallbackGroupScheduler::Priorities cur_prio(static_cast<CallbackGroupScheduler::Priorities>(i));
        for (ReadyCallbacksWithSharedPtr & ready_elem: ready_callbacks) {

            if(!found_work)
            {
                if (ready_elem.data->scheduler->get_unprocessed_executable(any_executable, cur_prio)) {
                    any_executable.callback_group = ready_elem.callback_group;
                    // mark callback group as in use
                    any_executable.callback_group->can_be_taken_from().store(false);
                    found_work = true;
                    ready_elem.ready = false;
//                     RCUTILS_LOG_ERROR_NAMED("rclcpp", "get_next_ready_executable : found ready executable");
                }
            }
            else
            {
                if (ready_elem.ready && ready_elem.data->scheduler->has_unprocessed_executables())
                {
                    //wake up worker thread
                    work_ready_conditional.notify_one();
                    return true;
                }
            }
        }
    }

    return found_work;
}

size_t
CBGExecutor::get_number_of_threads()
{
    return number_of_threads_;
}

void CBGExecutor::sync_callback_groups()
{
    if (!needs_callback_group_resync.exchange(false)) {
        return;
    }
//   RCUTILS_LOG_ERROR_NAMED("rclcpp", "sync_callback_groups");

    std::vector<std::pair<CallbackGroupData *, rclcpp::CallbackGroup::SharedPtr>> cur_group_data;
    cur_group_data.reserve(callback_groups.size());

    for (CallbackGroupData & d : callback_groups) {
        auto p = d.callback_group.lock();
        if (p) {
            cur_group_data.emplace_back(&d, std::move(p));
        }
    }

    std::scoped_lock<std::mutex> lk(callback_groups_mutex);
    std::vector<CallbackGroupData> next_group_data;

    std::set<CallbackGroup *> added_cbgs;

    auto insert_data = [&cur_group_data, &next_group_data, &added_cbgs](rclcpp::CallbackGroup::SharedPtr &&cbg)
    {
        // nodes may share callback groups, therefore we need to make sure we only add them once
        if(added_cbgs.find(cbg.get()) != added_cbgs.end())
        {
          return;
        }

        added_cbgs.insert(cbg.get());

        for (const auto & pair : cur_group_data) {
            if (pair.second == cbg) {
//                 RCUTILS_LOG_INFO("Using existing callback group");
                next_group_data.push_back(std::move(*pair.first));
                return;
            }
        }

//         RCUTILS_LOG_INFO("Using new callback group");

        CallbackGroupData new_entry;
        new_entry.scheduler = std::make_unique<CallbackGroupScheduler>();
        new_entry.executable_cache = std::make_unique<WeakExecutableWithRclHandleCache>(*new_entry.scheduler);
        new_entry.executable_cache->regenerate(*cbg);
        new_entry.callback_group = std::move(cbg);
        next_group_data.push_back(std::move(new_entry));
    };

    {
        std::vector<rclcpp::CallbackGroup::WeakPtr> added_cbgs_cpy;
        {
            std::lock_guard lock{added_callback_groups_mutex_};
            added_cbgs_cpy = added_callback_groups;
        }

        std::vector<node_interfaces::NodeBaseInterface::WeakPtr> added_nodes_cpy;
        {
            std::lock_guard lock{added_nodes_mutex_};
            added_nodes_cpy = added_nodes;
        }

        // *3 ist a rough estimate of how many callback_group a node may have
        next_group_data.reserve(added_cbgs_cpy.size() + added_nodes_cpy.size() * 3);

        nodes_executable_cache->clear();
//         nodes_executable_cache->guard_conditions.reserve(added_nodes_cpy.size());

//         RCUTILS_LOG_INFO("Added node size is %lu", added_nodes_cpy.size());

        for (const node_interfaces::NodeBaseInterface::WeakPtr & node_weak_ptr : added_nodes_cpy) {
            auto node_ptr = node_weak_ptr.lock();
            if (node_ptr) {
                node_ptr->for_each_callback_group(
                    [&insert_data](rclcpp::CallbackGroup::SharedPtr cbg)
                {
                    if (cbg->automatically_add_to_executor_with_node()) {
                        insert_data(std::move(cbg));
                    }
                });

                // register node guard condition, and trigger resync on node change event
                nodes_executable_cache->add_guard_condition(node_ptr->get_shared_notify_guard_condition(),
                [this] () {
//                                     RCUTILS_LOG_INFO("Node changed GC triggered");
                    needs_callback_group_resync.store(true);
                });
            }
        }

        for (const rclcpp::CallbackGroup::WeakPtr & cbg : added_cbgs_cpy) {
            auto p = cbg.lock();
            if (p) {
                insert_data(std::move(p));
            }
        }
    }

    callback_groups.swap(next_group_data);
}

void CBGExecutor::wait_for_work(
    std::chrono::nanoseconds timeout,
    bool do_not_wait_if_all_groups_busy)
{
    using namespace rclcpp::exceptions;

    WaitSetSize wait_set_size;

    sync_callback_groups();

    std::vector<CallbackGroupData *> idle_callback_groups;
    idle_callback_groups.reserve(callback_groups.size());

    {
        std::lock_guard g(callback_groups_mutex);
        for (CallbackGroupData & cbg_with_data: callback_groups) {

            // don't add anything to a waitset that has unprocessed data
            if (cbg_with_data.scheduler->has_unprocessed_executables()) {
                continue;
            }

            auto cbg_shr_ptr = cbg_with_data.callback_group.lock();
            if (!cbg_shr_ptr) {
                continue;
            }

            if (!cbg_shr_ptr->can_be_taken_from()) {
                continue;
            }

            //     CallbackGroupState & cbg_state = *cbg_with_data.callback_group_state;
            // regenerate the state data
            if (cbg_with_data.executable_cache->cache_ditry)
            {
                //       RCUTILS_LOG_INFO("Regenerating callback group");
                // Regenerating clears the dirty flag
                cbg_with_data.executable_cache->regenerate(*cbg_shr_ptr);
            }

            wait_set_size.add(cbg_with_data.executable_cache->wait_set_size);


            // setup the groups for the next round
            cbg_with_data.scheduler->clear_and_prepare(cbg_with_data.executable_cache->timers.size(),
                    cbg_with_data.executable_cache->subscribers.size(),
                    cbg_with_data.executable_cache->services.size(),
                    cbg_with_data.executable_cache->clients.size(),
                    cbg_with_data.executable_cache->waitables.size());

            idle_callback_groups.push_back(&cbg_with_data);
        }
    }

    if (do_not_wait_if_all_groups_busy && idle_callback_groups.empty()) {
        return;
    }

//     RCUTILS_LOG_WARN("Adding global_executable_cache");

    // interrupt_guard_condition_ and shutdown_guard_condition_
    wait_set_size.add(global_executable_cache->wait_set_size);

//     RCUTILS_LOG_WARN("Adding nodes_executable_cache");
    // guard conditions of the nodes
    wait_set_size.add(nodes_executable_cache->wait_set_size);

    // prepare the wait set
    wait_set_size.clear_and_resize_wait_set(wait_set_);

    // add all ready callback groups
    for (CallbackGroupData  *cbg_with_data: idle_callback_groups) {
        // no locking needed here, as we don't access the scheduler

        if(!cbg_with_data->executable_cache->add_to_wait_set(wait_set_))
        {
            RCUTILS_LOG_WARN_NAMED(
                "rclcpp",
                "Adding of objects to waitset failed. This should never happen.");
        }
    }

    // add guard_conditions for node or callback change
    global_executable_cache->add_to_wait_set(wait_set_);
    nodes_executable_cache->add_to_wait_set(wait_set_);

    if(!spinning.load())
    {
      return;
    }

    rcl_ret_t status =
        rcl_wait(&wait_set_, timeout.count());
    if (status == RCL_RET_WAIT_SET_EMPTY) {
        RCUTILS_LOG_WARN_NAMED(
            "rclcpp",
            "empty wait set received in rcl_wait(). This should never happen.");
    } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
        using rclcpp::exceptions::throw_from_rcl_error;
        throw_from_rcl_error(status, "rcl_wait() failed");
    }

    {
        std::lock_guard g(callback_groups_mutex);
        // super duper expensive
        // This will add all ready events to the associated schedulers
        WeakExecutableWithRclHandleCache::collect_ready_from_waitset(wait_set_);
    }

    //at this point we don't need the wait_set_ any more
}

void
CBGExecutor::run(size_t this_thread_number)
{
    (void)this_thread_number;

    bool had_work = false;

    while (rclcpp::ok(this->context_) && spinning.load()) {
        rclcpp::AnyExecutable any_exec;

        if (!get_next_ready_executable(any_exec))
        {
            if(wait_mutex_.try_lock())
            {
//                 RCUTILS_LOG_ERROR_NAMED("rclcpp", "calling wait_for_work");
                wait_for_work(next_exec_timeout_, false);
                wait_mutex_.unlock();
            }
            else
            {
                //some other thread is blocking the wait_for_work function.

                if(had_work)
                {
//                     RCUTILS_LOG_ERROR_NAMED("rclcpp", "triggering waitset");

                    // Wake the wait, because it may need to be recalculated
                    try {
                        interrupt_guard_condition_->trigger();
                    } catch (const rclcpp::exceptions::RCLError & ex) {
                        throw std::runtime_error(
                            std::string(
                                "Failed to trigger guard condition from execute_any_executable: ") + ex.what());
                    }
                }

                had_work = false;
                std::unique_lock lk(conditional_mutex);

                if(spinning.load())
                {
//                 RCUTILS_LOG_ERROR_NAMED("rclcpp", "going to sleep");
                  work_ready_conditional.wait(lk);
                }
//                 RCUTILS_LOG_ERROR_NAMED("rclcpp", "woken up");
            }
            continue;
        }

        had_work = true;

        execute_any_executable(any_exec);

        // Clear the callback_group to prevent the AnyExecutable destructor from
        // resetting the callback group `can_be_taken_from`
        any_exec.callback_group.reset();
    }

//     RCUTILS_LOG_INFO("Stopping execution thread");
}

void CBGExecutor::spin_once_internal(std::chrono::nanoseconds timeout)
{
    rclcpp::AnyExecutable any_exec;
    {
        if (!rclcpp::ok(this->context_) || !spinning.load()) {
            return;
        }

        if (!get_next_ready_executable(any_exec)) {

            wait_for_work(timeout);

            if (!get_next_ready_executable(any_exec)) {
                return;
            }
        }

        any_exec.callback_group->can_be_taken_from().store(false);
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
}

void
CBGExecutor::spin_once(std::chrono::nanoseconds timeout)
{
    if (spinning.exchange(true)) {
        throw std::runtime_error("spin_once() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

    spin_once_internal(timeout);
}

void
CBGExecutor::execute_any_executable(AnyExecutable & any_exec)
{
    if (any_exec.timer) {
//         RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing Timer");
        rclcpp::Executor::execute_timer(any_exec.timer);//, any_exec.data);
    }
    if (any_exec.subscription) {
//         RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing subscription");
        rclcpp::Executor::execute_subscription(any_exec.subscription);
    }
    if (any_exec.service) {
//         RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing service");
        rclcpp::Executor::execute_service(any_exec.service);
    }
    if (any_exec.client) {
//         RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing client");
        rclcpp::Executor::execute_client(any_exec.client);
    }
    if (any_exec.waitable) {
//         RCUTILS_LOG_ERROR_NAMED("rclcpp", "Executing waitable");
        any_exec.waitable->execute(any_exec.data);
    }
    // Reset the callback_group, regardless of type
    if(any_exec.callback_group)
    {
        any_exec.callback_group->can_be_taken_from().store(true);
    }
}

bool CBGExecutor::collect_and_execute_ready_events(
    std::chrono::nanoseconds max_duration,
    bool recollect_if_no_work_available)
{
    if (spinning.exchange(true)) {
        throw std::runtime_error("collect_and_execute_ready_events() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

    const auto start = std::chrono::steady_clock::now();
    const auto end_time = start + max_duration;
    auto cur_time = start;

    // collect any work, that is already ready
//   wait_for_work(std::chrono::nanoseconds::zero(), true);

    bool got_work_since_collect = false;
    bool first_collect = true;
    bool had_work = false;

    while (rclcpp::ok(this->context_) && spinning && cur_time < end_time) {
        if (!execute_ready_executables_until(end_time)) {

            if (!first_collect && !recollect_if_no_work_available) {
                // we are done
                return had_work;
            }

            if (first_collect || got_work_since_collect) {
                // collect any work, that is already ready
                wait_for_work(std::chrono::nanoseconds::zero(), !first_collect);

                first_collect = false;
                got_work_since_collect = false;
                continue;
            }

            return had_work;
        } else {
            got_work_since_collect = true;
        }

        cur_time = std::chrono::steady_clock::now();
    }

    return had_work;
}

void
CBGExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
    collect_and_execute_ready_events(max_duration, false);
}

void CBGExecutor::spin_all(std::chrono::nanoseconds max_duration)
{
    if (max_duration < std::chrono::nanoseconds::zero()) {
        throw std::invalid_argument("max_duration must be greater than or equal to 0");
    }

    collect_and_execute_ready_events(max_duration, true);
}

void
CBGExecutor::spin()
{
//     RCUTILS_LOG_ERROR_NAMED(
//         "rclcpp",
//         "CBGExecutor::spin()");


    if (spinning.exchange(true)) {
        throw std::runtime_error("spin() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
    std::vector<std::thread> threads;
    size_t thread_id = 0;
    {
        std::lock_guard wait_lock{wait_mutex_};
        for (; thread_id < number_of_threads_ - 1; ++thread_id) {
            auto func = std::bind(&CBGExecutor::run, this, thread_id);
            threads.emplace_back(func);
        }
    }

    run(thread_id);
    for (auto & thread : threads) {
        thread.join();
    }
}

void
CBGExecutor::add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr /*node_ptr*/,
    bool notify)
{
    {
        std::lock_guard lock{added_callback_groups_mutex_};
        added_callback_groups.push_back(group_ptr);
    }
    needs_callback_group_resync = true;
    if (notify) {
        // Interrupt waiting to handle new node
        try {
            interrupt_guard_condition_->trigger();
        } catch (const rclcpp::exceptions::RCLError & ex) {
            throw std::runtime_error(
                std::string(
                    "Failed to trigger guard condition on callback group add: ") + ex.what());
        }
    }
}

void
CBGExecutor::cancel()
{
    spinning.store(false);

    work_ready_conditional.notify_all();

    try {
        interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
        throw std::runtime_error(
            std::string("Failed to trigger guard condition in cancel: ") + ex.what());
    }
    work_ready_conditional.notify_all();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CBGExecutor::get_all_callback_groups()
{
    std::lock_guard lock{added_callback_groups_mutex_};
    return added_callback_groups;
}

void
CBGExecutor::remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    bool notify)
{
    bool found = false;
    {
        std::lock_guard lock{added_callback_groups_mutex_};
        added_callback_groups.erase(
            std::remove_if(
                added_callback_groups.begin(), added_callback_groups.end(),
                [&group_ptr, &found](const auto & weak_ptr)
        {
            auto shr_ptr = weak_ptr.lock();
            if (!shr_ptr) {
                return true;
            }

            if (group_ptr == shr_ptr) {
                found = true;
                return true;
            }
            return false;
        }), added_callback_groups.end());
        added_callback_groups.push_back(group_ptr);
    }

    if (found) {
        needs_callback_group_resync = true;
    }

    if (notify) {
        // Interrupt waiting to handle new node
        try {
            interrupt_guard_condition_->trigger();
        } catch (const rclcpp::exceptions::RCLError & ex) {
            throw std::runtime_error(
                std::string(
                    "Failed to trigger guard condition on callback group add: ") + ex.what());
        }
    }
}

void
CBGExecutor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
    // If the node already has an executor
    std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
    if (has_executor.exchange(true)) {
        throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' has already been added to an executor.");
    }

    {
        std::lock_guard lock{added_nodes_mutex_};
        added_nodes.push_back(node_ptr);
    }

    needs_callback_group_resync = true;

    if (notify) {
        // Interrupt waiting to handle new node
        try {
            interrupt_guard_condition_->trigger();
        } catch (const rclcpp::exceptions::RCLError & ex) {
            throw std::runtime_error(
                std::string(
                    "Failed to trigger guard condition on callback group add: ") + ex.what());
        }
    }
}

void
CBGExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
    add_node(node_ptr->get_node_base_interface(), notify);
}

void
CBGExecutor::remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify)
{
    {
        std::lock_guard lock{added_nodes_mutex_};
        added_nodes.erase(
            std::remove_if(
        added_nodes.begin(), added_nodes.end(), [&node_ptr](const auto & weak_ptr) {
            const auto shr_ptr = weak_ptr.lock();
            if (shr_ptr && shr_ptr == node_ptr) {
                return true;
            }
            return false;
        }), added_nodes.end());
    }

    needs_callback_group_resync = true;

    if (notify) {
        // Interrupt waiting to handle new node
        try {
            interrupt_guard_condition_->trigger();
        } catch (const rclcpp::exceptions::RCLError & ex) {
            throw std::runtime_error(
                std::string(
                    "Failed to trigger guard condition on callback group add: ") + ex.what());
        }
    }

    node_ptr->get_associated_with_executor_atomic().store(false);
}

void
CBGExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
    remove_node(node_ptr->get_node_base_interface(), notify);
}

// add a callback group to the executor, not bound to any node
void CBGExecutor::add_callback_group_only(rclcpp::CallbackGroup::SharedPtr group_ptr)
{
    add_callback_group(group_ptr, nullptr, true);
}
}
