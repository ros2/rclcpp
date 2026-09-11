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

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include "global_event_id_provider.hpp"

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
struct Worker
{
  std::mutex mutex;
  std::condition_variable condition_variable;
  std::atomic<bool> wakeup = false;

  void prepare_block()
  {
    wakeup = false;
  }

  void block()
  {
    std::unique_lock lk(mutex);
    condition_variable.wait(lk, [this]() -> bool {
        return wakeup;
    });
  }

  void block_for(std::chrono::nanoseconds timeout)
  {
    std::unique_lock lk(mutex);
    condition_variable.wait_for(lk, timeout, [this]() -> bool {
        return wakeup;
    });
  }

  void unblock()
  {
    {
      std::unique_lock lk(mutex);
      wakeup = true;
    }
    condition_variable.notify_one();
  }
};

/**
 * Queue of blocked (idle) worker threads.
 *
 * All member functions must be called while holding workers_mutex.
 * The mutex is exposed so that the scheduler can lock it together
 * with its own ready_callback_groups_mutex, which is needed to make
 * "check for work and enqueue self" atomic with respect to
 * "add work and wake a worker".
 */
struct WorkerQueue
{
  std::mutex workers_mutex;
  std::deque<Worker *> workers;

  bool release_workers = false;

  /**
   * Removes and returns the most recently blocked worker, or nullptr
   * if no worker is blocked.
   */
  Worker * pop_blocked_worker_thread()
  {
    if(workers.empty()) {
      // no threads available
      return nullptr;
    }
    Worker * worker = workers.front();
    workers.pop_front();
    return worker;
  }

  /**
   * Registers the worker as blocked. Returns false if the queue was
   * released and the worker must not block.
   */
  bool push_blocked_worker_thread(Worker * worker)
  {
    if(release_workers) {
      return false;
    }
    workers.push_front(worker);
    return true;
  }

  /**
   * Removes the given worker from the queue, if it is still in there.
   * Needed for timed blocks, where the worker may wake up on its own
   * without anyone having removed it from the queue.
   */
  void remove_worker_thread(Worker * worker)
  {
    auto it = std::find(workers.begin(), workers.end(), worker);
    if(it != workers.end()) {
      workers.erase(it);
    }
  }

  /**
   * Removes all blocked workers from the queue and marks the queue as
   * released. Returns the removed workers, the caller must unblock them
   * after dropping the lock.
   */
  std::deque<Worker *> release_all_worker_threads()
  {
    std::deque<Worker *> cpy;
    cpy.swap(workers);
    release_workers = true;
    return cpy;
  }
};

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
    explicit CallbackGroupHandle(CBGScheduler & scheduler, CallbackGroupType type)
    : scheduler(scheduler), type(type)
    {
    }

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

    CallbackGroupType get_type() {return type;}

    bool is_ready();

    // true if this cbg is inside the scheduler's queue
    bool in_queue = false;

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
      if (type != CallbackGroupType::Reentrant) {
        not_ready = true;
      }
    }

    std::mutex ready_mutex;

private:
    // will be set if cbg is mutual exclusive and something is executing
    bool not_ready = false;

    // true, if nothing is beeing executed, and there are no pending events
    bool idle = true;

    // type of the underlying callback group
    CallbackGroupType type;
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

    auto cbg_it = std::find(ready_callback_groups.begin(),
          ready_callback_groups.end(), callback_handle);
    if (cbg_it != ready_callback_groups.end()) {
      ready_callback_groups.erase(cbg_it);
    }

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

      if (!handle->in_queue) {
        ready_callback_groups.push_back(handle);
        handle->in_queue = true;
      }
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
    std::lock_guard l(ready_callback_groups_mutex);
    worker_checking_for_work = false;

    if(needs_sync) {
      needs_sync = false;
      return ExecutableEntityWithInfo{.entity =
          ExecutableEntity{.execute_function = sync_function, .callback_handle = nullptr},
        .moreEntitiesReady = false};
    }

    return get_next_ready_entity_intern();
  }

  ExecutableEntityWithInfo get_next_ready_entity(
    GlobalEventIdProvider::MonotonicId max_id)
  {
    std::lock_guard l(ready_callback_groups_mutex);
    worker_checking_for_work = false;

    if(needs_sync) {
      needs_sync = false;
      return ExecutableEntityWithInfo{.entity =
          ExecutableEntity{.execute_function = sync_function, .callback_handle = nullptr},
        .moreEntitiesReady = false};
    }

    return get_next_ready_entity_intern(max_id);
  }

  /**
   * Will be called while holding the ready_callback_groups_mutex lock
   */
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
   * Suppresses the next thread wakeup. This is useful when we know that
   * a thread is about to check for work anyway. In this case it would
   * be harmful to wake another thread for nothing
   */
  void suppress_thread_wakeup()
  {
    std::lock_guard l(ready_callback_groups_mutex);
    worker_checking_for_work = true;
  }

  /**
   * Wakes up one blocked worker thread, unless a worker is already
   * on its way to get_next_ready_entity(). The flag is only set if
   * a worker was actually woken, otherwise no one would ever clear it.
   */
  void unblock_one_worker_thread()
  {
    Worker * worker = nullptr;
    {
      std::scoped_lock l(ready_callback_groups_mutex, worker_queue.workers_mutex);
      if(worker_checking_for_work) {
        return;
      }
      worker = worker_queue.pop_blocked_worker_thread();
      if(worker == nullptr) {
        return;
      }
      worker_checking_for_work = true;
    }
    worker->unblock();
  }

  /**
   * Blocks the worker until work is available. Checking for work and
   * registering as blocked happens atomically under the same lock that
   * callback_group_ready / trigger_sync use to add work, so no wakeup
   * can be lost in between. Returns immediately if work showed up,
   * or if the worker queue was released.
   */
  void block_worker_thread(Worker * worker)
  {
    if(!prepare_and_enqueue_worker(worker)) {
      return;
    }
    worker->block();
  }

  void block_worker_thread_for(Worker * worker, std::chrono::nanoseconds timeout)
  {
    if(!prepare_and_enqueue_worker(worker)) {
      return;
    }
    worker->block_for(timeout);
    // on timeout the worker is still in the queue, remove it, as the
    // worker may not be valid any more after this call
    std::lock_guard lk(worker_queue.workers_mutex);
    worker_queue.remove_worker_thread(worker);
  }

  void release_all_worker_threads()
  {
    std::deque<Worker *> workers;
    {
      std::lock_guard lk(worker_queue.workers_mutex);
      workers = worker_queue.release_all_worker_threads();
    }
    for(Worker * worker : workers) {
      worker->unblock();
    }
  }

protected:
  virtual std::unique_ptr<CallbackGroupHandle> get_handle_for_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) = 0;

  /**
   * Returns true if the worker was enqueued and shall block.
   * Returns false if there is work pending or the queue was released.
   */
  bool prepare_and_enqueue_worker(Worker * worker)
  {
    worker->prepare_block();
    std::scoped_lock l(ready_callback_groups_mutex, worker_queue.workers_mutex);
    if(needs_sync || !ready_callback_groups.empty()) {
      return false;
    }
    return worker_queue.push_blocked_worker_thread(worker);
  }

  // sync function, will be triggered if the executor needs
  // resync. E.g. if entities / cbg or nodes were added / removed
  std::function<void ()> sync_function;

  bool needs_sync = false;

  std::mutex ready_callback_groups_mutex;
  std::deque<CallbackGroupHandle *> ready_callback_groups;

  bool worker_checking_for_work = false;

  WorkerQueue worker_queue;

  std::list<std::unique_ptr<CallbackGroupHandle>> callback_groups;
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
