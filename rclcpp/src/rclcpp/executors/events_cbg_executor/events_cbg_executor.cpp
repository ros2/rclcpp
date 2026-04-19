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

#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/node.hpp"

#include "first_in_first_out_scheduler.hpp"
#include "timer_manager.hpp"
#include "registered_entity_cache.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"


namespace rclcpp::executors
{
namespace cbg_executor
{
struct GlobalWeakExecutableCache
{
  std::vector<cbg_executor::GuardConditionWithFunction> guard_conditions;

  GlobalWeakExecutableCache() = default;
  GlobalWeakExecutableCache(const GlobalWeakExecutableCache &) = default;
  GlobalWeakExecutableCache(GlobalWeakExecutableCache &&) = default;
  ~GlobalWeakExecutableCache()
  {
    for (const auto & gc_ref : guard_conditions) {
      gc_ref.guard_condition->set_on_trigger_callback(nullptr);
    }
  }

  GlobalWeakExecutableCache & operator=(const GlobalWeakExecutableCache &) = default;
  GlobalWeakExecutableCache & operator=(GlobalWeakExecutableCache &&) = default;

  void add_guard_condition_event(
    const rclcpp::GuardCondition::SharedPtr & ptr,
    std::function<void(void)> fun)
  {
    guard_conditions.emplace_back(ptr, std::move(fun));

    // reset all lambdas in case the guard_conditions vector
    // was resized and the entry ptrs were moved
    for (auto & entry : guard_conditions) {
      entry.guard_condition->set_on_trigger_callback(
        [ptr = &entry](size_t nr_events) {
          for (size_t i = 0; i < nr_events; i++) {
            if (ptr->handle_guard_condition_fun) {
              ptr->handle_guard_condition_fun();
            }
          }
        });
    }
  }

  void clear()
  {
    guard_conditions.clear();
  }
};
}  // namespace cbg_executor

EventsCBGExecutor::EventsCBGExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  std::chrono::nanoseconds next_exec_timeout)
: scheduler(std::make_unique<cbg_executor::FirstInFirstOutScheduler>([this] () {
      needs_callback_group_resync = true;
  })),
  next_exec_timeout_(next_exec_timeout),
  spinning(false),
  interrupt_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context) ),
  shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context) ),
  context_(options.context),
  timer_manager(std::make_unique<cbg_executor::TimerManager>(context_)),
  global_executable_cache(std::make_unique<cbg_executor::GlobalWeakExecutableCache>() ),
  nodes_executable_cache(std::make_unique<cbg_executor::GlobalWeakExecutableCache>() )
{
  global_executable_cache->add_guard_condition_event (
        interrupt_guard_condition_,
        std::function<void(void)>() );

  global_executable_cache->add_guard_condition_event(
    shutdown_guard_condition_, [this]() {
      shutdown();
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
}

EventsCBGExecutor::~EventsCBGExecutor()
{
  shutdown();
}

void EventsCBGExecutor::shutdown()
{
  if(!timer_manager) {
    // already shut down
    return;
  }

  // we need to shut down the timer manager first, as it might access the Schedulers
  timer_manager->stop();

  in_shutdown = true;
  bool was_spinning = spinning;

  // signal all processing threads to shut down
  spinning = false;

  if(was_spinning) {
    scheduler->release_all_worker_threads();
  }

  remove_all_nodes_and_callback_groups();

  {
    std::scoped_lock l(callback_groups_mutex);
    callback_groups.clear();
  }

  // Remove shutdown callback handle registered to Context
  if (!context_->remove_on_shutdown_callback(shutdown_callback_handle_) ) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to remove registered on_shutdown callback");
    rcl_reset_error();
  }

  // now we may release the memory of the timer_manager,
  // as we know no thread is working on it any more
  timer_manager.reset();
}

void EventsCBGExecutor::remove_all_nodes_and_callback_groups()
{
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
}

bool EventsCBGExecutor::execute_previous_ready_executables_until(
  const std::chrono::time_point<std::chrono::steady_clock> & stop_time)
{
  bool found_work = false;

  const uint64_t last_ready_id = cbg_executor::GlobalEventIdProvider::get_last_id();

  while(true) {
    auto ready_entity = scheduler->get_next_ready_entity(last_ready_id);
    if(!ready_entity.entity) {
      break;
    }

    found_work = true;

    ready_entity.entity->execute_function();

    scheduler->mark_entity_as_executed(*ready_entity.entity);

    if(std::chrono::steady_clock::now() >= stop_time) {
      break;
    }
  }

  return found_work;
}


size_t
EventsCBGExecutor::get_number_of_threads() const
{
  return number_of_threads_;
}

void EventsCBGExecutor::trigger_callback_group_sync()
{
  if(in_shutdown) {
    return;
  }

  needs_callback_group_resync = true;

  if (!spinning) {
    sync_callback_groups();
  } else {
    scheduler->unblock_one_worker_thread();
  }
}

void EventsCBGExecutor::sync_callback_groups()
{
  if (!needs_callback_group_resync.exchange(false) ) {
    return;
  }

  std::scoped_lock l(callback_groups_mutex);


  std::vector<std::pair<CallbackGroupData *, rclcpp::CallbackGroup::SharedPtr>> cur_group_data;
  cur_group_data.reserve(callback_groups.size() );

  for (CallbackGroupData & d : callback_groups) {
    auto p = d.callback_group.lock();
    if (p) {
      cur_group_data.emplace_back(&d, std::move(p) );
    }
  }

  std::vector<CallbackGroupData> next_group_data;

  std::set<CallbackGroup *> added_cbgs;

  auto insert_data =
    [&cur_group_data, &next_group_data, &added_cbgs,
      this](rclcpp::CallbackGroup::SharedPtr && cbg, CallbackGroupData::Origin origin) {
      // nodes may share callback groups, therefore we need to make sure we only add them once
      if (added_cbgs.find(cbg.get() ) != added_cbgs.end() ) {
        return;
      }

      added_cbgs.insert(cbg.get() );

      for (const auto & pair : cur_group_data) {
        if (pair.second == cbg) {
          next_group_data.push_back(std::move(*pair.first) );
          // call regenerate, in case something changed in the group
          next_group_data.back().registered_entities->regenerate_events();
          return;
        }
      }

      CallbackGroupData new_entry{.callback_group = cbg,
        .registered_entities = std::make_unique<cbg_executor::RegisteredEntityCache>(*scheduler,
        *timer_manager, cbg), .origin = origin};
      new_entry.registered_entities->regenerate_events();
      next_group_data.push_back(std::move(new_entry) );
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

    // *3 is a rough estimate of how many callback_group a node may have
    next_group_data.reserve(added_cbgs_cpy.size() + (added_nodes_cpy.size() * 3));

    nodes_executable_cache->clear();

    for (const node_interfaces::NodeBaseInterface::WeakPtr & node_weak_ptr : added_nodes_cpy) {
      auto node_ptr = node_weak_ptr.lock();
      if (node_ptr) {
        node_ptr->for_each_callback_group(
          [&insert_data](rclcpp::CallbackGroup::SharedPtr cbg) {
            if (cbg->automatically_add_to_executor_with_node() ) {
              insert_data(std::move(cbg), CallbackGroupData::Origin::Node);
            }
          });

        // register node guard condition, and trigger
        // resync on node change event
        nodes_executable_cache->add_guard_condition_event(
          node_ptr->get_shared_notify_guard_condition(),
          [this]() {
            scheduler->trigger_sync();
          });
      }
    }

    for (const rclcpp::CallbackGroup::WeakPtr & cbg : added_cbgs_cpy) {
      auto p = cbg.lock();
      if (p) {
        insert_data(std::move(p), CallbackGroupData::Origin::ManualAdded);
      }
    }
  }

  // FIXME inform scheduler about remove cbgs

  callback_groups.swap(next_group_data);
}

void
EventsCBGExecutor::run(size_t this_thread_number, bool block_initially)
{
  (void) this_thread_number;

  while (rclcpp::ok(this->context_) && spinning.load() ) {
    if(block_initially) {
      block_initially = false;
      scheduler->block_worker_thread();
    }

    sync_callback_groups();

    auto ready_entity = scheduler->get_next_ready_entity();
    if(!ready_entity.entity) {
      scheduler->block_worker_thread();
      continue;
    }

    if(ready_entity.moreEntitiesReady) {
      scheduler->unblock_one_worker_thread();
    }

    ready_entity.entity->execute_function();

    scheduler->mark_entity_as_executed(*ready_entity.entity);
  }
}

void
EventsCBGExecutor::run(
  size_t this_thread_number,
  const std::function<void(const std::exception & e)> & exception_handler)
{
  (void) this_thread_number;

  while (rclcpp::ok(this->context_) && spinning.load() ) {
    sync_callback_groups();

    auto ready_entity = scheduler->get_next_ready_entity();
    if(!ready_entity.entity) {
      scheduler->block_worker_thread();
      continue;
    }

    try {
      ready_entity.entity->execute_function();
    } catch (const std::exception & e) {
      exception_handler(e);
    }

    scheduler->mark_entity_as_executed(*ready_entity.entity);
  }
}


void EventsCBGExecutor::spin_once_internal(std::chrono::nanoseconds timeout)
{
  if (!rclcpp::ok(this->context_) || !spinning.load() ) {
    return;
  }

  sync_callback_groups();

  auto ready_entity = scheduler->get_next_ready_entity();
  if(!ready_entity.entity) {
    if (timeout < std::chrono::nanoseconds::zero()) {
      // can't use std::chrono::nanoseconds::max, as wait_for
      // internally computes end time by using ::now() + timeout
      // as a workaround, we use some absurd high timeout
      timeout = std::chrono::hours(10000);
    }

    scheduler->block_worker_thread_for(timeout);

    ready_entity = scheduler->get_next_ready_entity();

    if (!ready_entity.entity) {
      return;
    }
  }

  ready_entity.entity->execute_function();

  scheduler->mark_entity_as_executed(*ready_entity.entity);
}

void
EventsCBGExecutor::spin_once(std::chrono::nanoseconds timeout)
{
  if (spinning.exchange(true) ) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  spin_once_internal(timeout);
}


void
EventsCBGExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  collect_and_execute_ready_events(max_duration, false);
}

void EventsCBGExecutor::spin_all(std::chrono::nanoseconds max_duration)
{
  if (max_duration < std::chrono::nanoseconds::zero() ) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }

  collect_and_execute_ready_events(max_duration, true);
}

bool EventsCBGExecutor::collect_and_execute_ready_events(
  std::chrono::nanoseconds max_duration,
  bool recollect_if_no_work_available)
{
  if (spinning.exchange(true) ) {
    throw std::runtime_error("collect_and_execute_ready_events() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  const auto start = std::chrono::steady_clock::now();
  const auto end_time = start + max_duration;
  auto cur_time = start;

  bool had_work = false;

  while (rclcpp::ok(this->context_) && spinning && cur_time <= end_time) {
    sync_callback_groups();

    if (!execute_previous_ready_executables_until(end_time) ) {
      return had_work;
    }

    had_work = true;

    if (!recollect_if_no_work_available) {
      // we are done
      return had_work;
    }

    cur_time = std::chrono::steady_clock::now();
  }

  return had_work;
}
void
EventsCBGExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  for ( ; thread_id < number_of_threads_ - 1; ++thread_id) {
    threads.emplace_back([this, thread_id]()
      {
        run(thread_id, true);
    });
  }

  run(thread_id, false);
  for (auto & thread : threads) {
    thread.join();
  }
}

void EventsCBGExecutor::spin(
  const std::function<void(const std::exception & e)> & exception_handler)
{
  if (spinning.exchange(true) ) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  for ( ; thread_id < number_of_threads_ - 1; ++thread_id) {
    threads.emplace_back([this, thread_id, exception_handler]()
      {
        run(thread_id, exception_handler);
      }
    );
  }

  run(thread_id, exception_handler);
  for (auto & thread : threads) {
    thread.join();
  }
}

void
EventsCBGExecutor::add_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group_ptr,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & /*node_ptr*/,
  bool notify)
{
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }

  {
    std::lock_guard lock{added_callback_groups_mutex_};
    added_callback_groups.push_back(group_ptr);
  }
  trigger_callback_group_sync();

  if (notify) {
    // Interrupt waiting to handle new node
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group add: ") + ex.what() );
    }
  }
}

void
EventsCBGExecutor::cancel()
{
  bool was_spinning = spinning;

  spinning.store(false);

  if(was_spinning) {
    scheduler->release_all_worker_threads();
  }

  try {
    interrupt_guard_condition_->trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("Failed to trigger guard condition in cancel: ") + ex.what() );
  }
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsCBGExecutor::get_all_callback_groups()
{
  if (!spinning) {
    sync_callback_groups();
  }
  std::lock_guard lock{callback_groups_mutex};
  std::vector<rclcpp::CallbackGroup::WeakPtr> ret;
  ret.reserve(callback_groups.size());
  for( auto & cbg_data : callback_groups) {
    ret.push_back(cbg_data.callback_group);
  }
  return ret;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsCBGExecutor::get_manually_added_callback_groups()
{
  if (!spinning) {
    sync_callback_groups();
  }
  std::lock_guard lock{callback_groups_mutex};
  std::vector<rclcpp::CallbackGroup::WeakPtr> ret;
  ret.reserve(callback_groups.size());
  for( auto & cbg_data : callback_groups) {
    if(cbg_data.origin == CallbackGroupData::Origin::ManualAdded) {
      ret.push_back(cbg_data.callback_group);
    }
  }
  return ret;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsCBGExecutor::get_automatically_added_callback_groups_from_nodes()
{
  if (!spinning) {
    sync_callback_groups();
  }
  std::lock_guard lock{callback_groups_mutex};
  std::vector<rclcpp::CallbackGroup::WeakPtr> ret;
  ret.reserve(callback_groups.size());
  for( auto & cbg_data : callback_groups) {
    if(cbg_data.origin == CallbackGroupData::Origin::Node) {
      ret.push_back(cbg_data.callback_group);
    }
  }
  return ret;
}

void EventsCBGExecutor::unregister_event_callbacks(const rclcpp::CallbackGroup::SharedPtr & cbg)
const
{
  const auto remove_sub = [](const rclcpp::SubscriptionBase::SharedPtr & s) {
      s->clear_on_new_message_callback();
    };
  const auto remove_timer = [this](const rclcpp::TimerBase::SharedPtr & s) {
      timer_manager->remove_timer(s);
    };

  const auto remove_client = [](const rclcpp::ClientBase::SharedPtr & s) {
      s->clear_on_new_response_callback();
    };

  const auto remove_service = [](const rclcpp::ServiceBase::SharedPtr & s) {
      s->clear_on_new_request_callback();
    };

  auto gc_ptr = cbg->get_notify_guard_condition();
  if (gc_ptr) {
    gc_ptr->set_on_trigger_callback(std::function<void(size_t)>());
  }

  const auto remove_waitable = [](const rclcpp::Waitable::SharedPtr & s) {
      s->clear_on_ready_callback();
    };


  cbg->collect_all_ptrs(remove_sub, remove_service, remove_client, remove_timer, remove_waitable);
}


void
EventsCBGExecutor::remove_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group_ptr,
  bool notify)
{
  if (!group_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Callback group needs to be associated with an executor.");
  }
  bool found = false;
  {
    std::lock_guard lock{added_callback_groups_mutex_};
    added_callback_groups.erase(
      std::remove_if(
        added_callback_groups.begin(), added_callback_groups.end(),
        [&group_ptr, &found](const auto & weak_ptr) {
          auto shr_ptr = weak_ptr.lock();
          if (!shr_ptr) {
            return true;
          }

          if (group_ptr == shr_ptr) {
            found = true;
            return true;
          }
          return false;
        }), added_callback_groups.end() );
  }

  if(!found) {
    throw std::runtime_error("Callback group needs to be associated with this executor.");
  }

  // we need to unregister all callbacks
  unregister_event_callbacks(group_ptr);

  if (found) {
    trigger_callback_group_sync();
  }

  group_ptr->get_associated_with_executor_atomic().exchange(false);

  if (notify) {
    // Interrupt waiting to handle new node
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group add: ") + ex.what() );
    }
  }
}

void
EventsCBGExecutor::add_node(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
  bool /*notify*/)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true) ) {
    throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' has already been added to an executor.");
  }

  {
    std::lock_guard lock{added_nodes_mutex_};
    added_nodes.push_back(node_ptr);
  }

  trigger_callback_group_sync();
}

void
EventsCBGExecutor::add_node(const std::shared_ptr<rclcpp::Node> & node_ptr, bool notify)
{
  add_node(node_ptr->get_node_base_interface(), notify);
}

void
EventsCBGExecutor::remove_node(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
  bool notify)
{
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (!has_executor.exchange(false)) {
    throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' needs to be associated with an executor.");
  }

  {
    std::lock_guard lock{added_nodes_mutex_};
    added_nodes.erase(
      std::remove_if(
        added_nodes.begin(), added_nodes.end(), [&node_ptr](const auto & weak_ptr) {
          const auto shr_ptr = weak_ptr.lock();
          return shr_ptr && shr_ptr == node_ptr;
        }), added_nodes.end());
  }

  node_ptr->for_each_callback_group(
    [this](const rclcpp::CallbackGroup::SharedPtr & cbg)
    {
      unregister_event_callbacks(cbg);
    }
  );

  node_ptr->get_shared_notify_guard_condition()->set_on_trigger_callback(
      std::function<void(size_t)>());

  trigger_callback_group_sync();

  if (notify) {
    scheduler->unblock_one_worker_thread();
    // Interrupt waiting to handle new node
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group add: ") + ex.what() );
    }
  }

  node_ptr->get_associated_with_executor_atomic().store(false);
}

void
EventsCBGExecutor::remove_node(const std::shared_ptr<rclcpp::Node> & node_ptr, bool notify)
{
  remove_node(node_ptr->get_node_base_interface(), notify);
}

// add a callback group to the executor, not bound to any node
void EventsCBGExecutor::add_callback_group_only(const rclcpp::CallbackGroup::SharedPtr & group_ptr)
{
  add_callback_group(group_ptr, nullptr, true);
}
}  // namespace rclcpp::executors
