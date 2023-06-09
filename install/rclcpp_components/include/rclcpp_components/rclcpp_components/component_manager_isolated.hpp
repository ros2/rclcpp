// Copyright 2021 Open Source Robotics Foundation, Inc.
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


#ifndef RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__
#define RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp_components/component_manager.hpp"


namespace rclcpp_components
{
/// ComponentManagerIsolated uses dedicated single-threaded executors for each components.
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ComponentManagerIsolated : public rclcpp_components::ComponentManager
{
  using rclcpp_components::ComponentManager::ComponentManager;

  struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
    std::atomic_bool thread_initialized;

    /// Constructor for the wrapper.
    /// This is necessary as atomic variables don't have copy/move operators
    /// implemented so this structure is not copyable/movable by default
    explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
    : executor(exec),
      thread_initialized(false)
    {
    }
  };

public:
  ~ComponentManagerIsolated()
  {
    if (node_wrappers_.size()) {
      for (auto & executor_wrapper : dedicated_executor_wrappers_) {
        cancel_executor(executor_wrapper.second);
      }
      node_wrappers_.clear();
    }
  }

protected:
  /// Add component node to executor model, it's invoked in on_load_node()
  /**
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void
  add_node_to_executor(uint64_t node_id) override
  {
    auto exec = std::make_shared<ExecutorT>();
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());

    // Emplace rather than std::move since move operations are deleted for atomics
    auto result = dedicated_executor_wrappers_.emplace(std::make_pair(node_id, exec));
    DedicatedExecutorWrapper & wrapper = result.first->second;
    wrapper.executor = exec;
    auto & thread_initialized = wrapper.thread_initialized;
    wrapper.thread = std::thread(
      [exec, &thread_initialized]() {
        thread_initialized = true;
        exec->spin();
      });
  }
  /// Remove component node from executor model, it's invoked in on_unload_node()
  /**
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void
  remove_node_from_executor(uint64_t node_id) override
  {
    auto executor_wrapper = dedicated_executor_wrappers_.find(node_id);
    if (executor_wrapper != dedicated_executor_wrappers_.end()) {
      cancel_executor(executor_wrapper->second);
      dedicated_executor_wrappers_.erase(executor_wrapper);
    }
  }

private:
  /// Stops a spinning executor avoiding race conditions.
  /**
   * @param executor_wrapper executor to stop and its associated thread
   */
  void cancel_executor(DedicatedExecutorWrapper & executor_wrapper)
  {
    // Verify that the executor thread has begun spinning.
    // If it has not, then wait until the thread starts to ensure
    // that cancel() will fully stop the execution
    //
    // This prevents a previous race condition that occurs between the
    // creation of the executor spin thread and cancelling an executor

    if (!executor_wrapper.thread_initialized) {
      auto context = this->get_node_base_interface()->get_context();

      // Guarantee that either the executor is spinning or we are shutting down.
      while (!executor_wrapper.executor->is_spinning() && rclcpp::ok(context)) {
        // This is an arbitrarily small delay to avoid busy looping
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
    }

    // After the while loop we are sure that the executor is now spinning, so
    // the call to cancel() will work.
    executor_wrapper.executor->cancel();
    // Wait for the thread task to return
    executor_wrapper.thread.join();
  }

  std::unordered_map<uint64_t, DedicatedExecutorWrapper> dedicated_executor_wrappers_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__
