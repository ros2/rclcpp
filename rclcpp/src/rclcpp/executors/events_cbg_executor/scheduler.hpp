// Copyright 2024 Cellumation GmbH.
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
#pragma once

#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include "global_event_id_provider.hpp"

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
class CBGScheduler
{
public:
  struct WaitableWithEventType
  {
    rclcpp::Waitable::WeakPtr waitable;
    int internal_event_type;

    bool expired() const
    {
      return waitable.expired();
    }
  };

  struct CallbackEventType
  {
    explicit CallbackEventType(std::function<void()> callback)
    : callback(std::move(callback))
    {
    }

    std::function<void()> callback;

    bool expired() const
    {
      return false;
    }
  };

  struct CallbackGroupHandle
  {
    explicit CallbackGroupHandle(CBGScheduler & scheduler)
    : scheduler(scheduler) {}

    CallbackGroupHandle(const CallbackGroupHandle &) = delete;
    CallbackGroupHandle(CallbackGroupHandle &&) = delete;

    virtual ~CallbackGroupHandle() = default;

    CallbackGroupHandle & operator=(const CallbackGroupHandle &) = delete;
    CallbackGroupHandle & operator=(CallbackGroupHandle &&) = delete;

    virtual std::function<void(size_t)> get_ready_callback_for_entity(
      const rclcpp::SubscriptionBase::WeakPtr & entity) = 0;
    virtual std::function<void(std::function<void()> executed_callback)>
    get_ready_callback_for_entity(const rclcpp::TimerBase::WeakPtr & entity) = 0;
    virtual std::function<void(size_t)> get_ready_callback_for_entity(
      const rclcpp::ClientBase::WeakPtr & entity) = 0;
    virtual std::function<void(size_t)> get_ready_callback_for_entity(
      const rclcpp::ServiceBase::WeakPtr & entity) = 0;
    virtual std::function<void(size_t,
      int)> get_ready_callback_for_entity(const rclcpp::Waitable::WeakPtr & entity) = 0;
    virtual std::function<void(size_t)> get_ready_callback_for_entity(
      const CallbackEventType & entity) = 0;

      /**
       * Marks the last removed ready entity as executed.
       */
    void mark_as_executed()
    {
      {
        std::lock_guard l(ready_mutex);
        not_ready = false;

        if(!has_ready_entities()) {
          idle = true;
          return;
        }
      }
      // inform scheduler that we have more work
      scheduler.callback_group_ready(this, false);
    }

    bool is_ready();

protected:
    CBGScheduler & scheduler;

    /**
     * Will always be called under lock of ready_mutex
     */
    virtual bool has_ready_entities() const = 0;

    /**
     * Executes the given function to add an entity
     * under the ready mutex. Afterwards the function
     * checks if the scheduler needs to be informed
     * that the callback group got ready and informs
     * it if needed.
     */
    template<typename add_fun>
    void add_ready_entity(const add_fun & fun)
    {
      {
        std::lock_guard l(ready_mutex);

        fun();

        if(not_ready || !idle) {
          return;
        }

        idle = false;
      }

      // If we reached this point, we were idle and now have work,
      // therefore we need to move this callback group into the list
      // of ready callback groups.
      scheduler.callback_group_ready(this, true);
    }

    void mark_as_skipped()
    {
      if(!has_ready_entities()) {
        idle = true;
      }
    }

    /**
     * Must be called by derived classes if a ready entity is
     * returned. This call must happen under a lock holding the
     * ready_mutex.
     */
    void mark_as_executing()
    {
      not_ready = true;
    }

    std::mutex ready_mutex;

private:
    // will be set if cbg is mutual exclusive and something is executing
    bool not_ready = false;

    // true, if nothing is beeing executed, and there are no pending events
    bool idle = true;
  };

  struct ExecutableEntity
  {
    // if called executes the entity
    std::function<void()> execute_function;
    // The callback group associated with the entity. Can be nullptr.
    CallbackGroupHandle *callback_handle = nullptr;
  };

  /**
   * @param sync_function A special purpose sync function, that shall be
   *                      executed with high priority if triggered by
   *                      trigger_sync();
   */
  explicit CBGScheduler(std::function<void ()> sync_function)
  : sync_function(sync_function) {}
  CBGScheduler(const CBGScheduler &) = delete;
  CBGScheduler(CBGScheduler &&) = delete;
  virtual ~CBGScheduler() = default;

  CBGScheduler & operator=(const CBGScheduler &) = delete;
  CBGScheduler & operator=(CBGScheduler &&) = delete;

  CallbackGroupHandle * add_callback_group(const rclcpp::CallbackGroup::SharedPtr & callback_group)
  {
    auto uPtr = get_handle_for_callback_group(callback_group);
    CallbackGroupHandle * ret = uPtr.get();

    std::lock_guard lk(ready_callback_groups_mutex);

    callback_groups.push_back(std::move(uPtr));
    return ret;
  }

  void remove_callback_group(const CallbackGroupHandle *callback_handle)
  {
    std::lock_guard lk(ready_callback_groups_mutex);
    ready_callback_groups.erase(std::find(ready_callback_groups.begin(),
          ready_callback_groups.end(), callback_handle));

    callback_groups.remove_if([&callback_handle] (const auto & e) {
        return e.get() == callback_handle;
    });
  }

  /** Will be called, by CallbackGroupHandle if any entity in the cb group is ready for execution
   * and the cb group was idle before
   * @param callback_group_was_idle Is false, if no entity of the callback group was executed,
   *                                before this call was made. This means we need to wakeup a
   *                                a new thread.
   */
  void callback_group_ready(CallbackGroupHandle *handle, bool callback_group_was_idle)
  {
    {
      std::lock_guard l(ready_callback_groups_mutex);
      ready_callback_groups.push_back(handle);
    }

    if(callback_group_was_idle) {
      unblock_one_worker_thread();
    }
  }

  struct ExecutableEntityWithInfo
  {
    std::optional<ExecutableEntity> entity;
    bool moreEntitiesReady{};
  };

  /**
   * Returns the next ready entity that shall be executed.
   * If a entity is removed here, the scheduler may assume that
   * it will be executed, and that the function mark_entity_as_executed
   * will be called afterwards.
   */
  ExecutableEntityWithInfo get_next_ready_entity()
  {
    {
      std::lock_guard l(ready_callback_groups_mutex);
      if(needs_sync) {
        needs_sync = false;
        return ExecutableEntityWithInfo{.entity =
            ExecutableEntity{.execute_function = sync_function, .callback_handle = nullptr},
          .moreEntitiesReady = false};
      }
    }

    return get_next_ready_entity_intern();
  }

  ExecutableEntityWithInfo get_next_ready_entity(
    GlobalEventIdProvider::MonotonicId max_id)
  {
    {
      std::lock_guard l(ready_callback_groups_mutex);
      if(needs_sync) {
        needs_sync = false;
        return ExecutableEntityWithInfo{.entity =
            ExecutableEntity{.execute_function = sync_function, .callback_handle = nullptr},
          .moreEntitiesReady = false};
      }
    }

    return get_next_ready_entity_intern(max_id);
  }

  virtual ExecutableEntityWithInfo get_next_ready_entity_intern() = 0;
  virtual ExecutableEntityWithInfo get_next_ready_entity_intern(
    GlobalEventIdProvider::MonotonicId max_id) = 0;

  /**
   * If this function was triggered, a worker thread must
   * be woken up, and the next call to get_next_ready_entity
   * must return a ExecutableEntityWithInfo with the sync
   * function in it.Or reworded if this function is triggered
   * the sync function will be executed as the next exent by
   * the executor.
   */
  void trigger_sync()
  {
    bool wake_worker = false;
    {
      std::lock_guard l(ready_callback_groups_mutex);
      if(!needs_sync) {
        wake_worker = true;
      }
      needs_sync = true;
    }
    if(wake_worker) {
      unblock_one_worker_thread();
    }
  }

  /**
   * Must be called, after a entity was executed. This function will
   * normally be used, to mark the associated callback group as ready
   * again.
   */
  void mark_entity_as_executed(const ExecutableEntity & e)
  {
    if(e.callback_handle != nullptr) {
      e.callback_handle->mark_as_executed();
    }
  }

  /**
   * Wakes up a worker thread
   */
  void unblock_one_worker_thread()
  {
    {
      std::lock_guard lk(ready_callback_groups_mutex);
      release_worker_once = true;
    }
    work_ready_conditional.notify_one();
  }

  void block_worker_thread()
  {
    std::unique_lock lk(ready_callback_groups_mutex);
    work_ready_conditional.wait(lk, [this]() -> bool {
        return !ready_callback_groups.empty() || release_worker_once || release_workers;
    });
    release_worker_once = false;
  }

  void block_worker_thread_for(std::chrono::nanoseconds timeout)
  {
    std::unique_lock lk(ready_callback_groups_mutex);
    work_ready_conditional.wait_for(lk, timeout, [this]() -> bool {
        return !ready_callback_groups.empty() || release_worker_once || release_workers;
    });
    release_worker_once = false;
  }

  void release_all_worker_threads()
  {
    {
      std::lock_guard lk(ready_callback_groups_mutex);
      release_workers = true;
    }
    work_ready_conditional.notify_all();
  }

protected:
  virtual std::unique_ptr<CallbackGroupHandle> get_handle_for_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) = 0;

  // sync function, will be triggered if the executor needs
  // resync. E.g. if entities / cbg or nodes were added / removed
  std::function<void ()> sync_function;

  bool needs_sync = false;

  std::mutex ready_callback_groups_mutex;
  std::deque<CallbackGroupHandle *> ready_callback_groups;

  bool release_workers = false;
  bool release_worker_once = false;

  std::condition_variable work_ready_conditional;

  std::list<std::unique_ptr<CallbackGroupHandle>> callback_groups;
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
