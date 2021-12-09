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
    std::promise<void> promise;
  };

public:
  ~ComponentManagerIsolated()
  {
    if (node_wrappers_.size()) {
      for (auto & executor_wrapper : dedicated_executor_wrappers_) {
        executor_wrapper.second.promise.set_value();
        executor_wrapper.second.executor->cancel();
        executor_wrapper.second.thread.join();
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
    DedicatedExecutorWrapper executor_wrapper;
    auto exec = std::make_shared<ExecutorT>();
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());
    executor_wrapper.executor = exec;
    executor_wrapper.thread = std::thread(
      [exec, cancel_token = executor_wrapper.promise.get_future()]() {
        exec->spin_until_future_complete(cancel_token);
      });
    dedicated_executor_wrappers_[node_id] = std::move(executor_wrapper);
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
      executor_wrapper->second.promise.set_value();
      executor_wrapper->second.executor->cancel();
      executor_wrapper->second.thread.join();
      dedicated_executor_wrappers_.erase(executor_wrapper);
    }
  }

private:
  std::unordered_map<uint64_t, DedicatedExecutorWrapper> dedicated_executor_wrappers_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__
