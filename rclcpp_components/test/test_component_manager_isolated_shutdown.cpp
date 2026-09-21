// Copyright 2026 Chaijz888
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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "composition_interfaces/srv/load_node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

using namespace std::chrono_literals;

class ShutdownTrackingExecutor : public rclcpp::executors::SingleThreadedExecutor
{
public:
  void spin() override
  {
    rclcpp::executors::SingleThreadedExecutor::spin();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      spin_exited_ = true;
    }
    condition_.notify_one();
  }

  static bool wait_for_spin_to_exit(std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, []() {return spin_exited_;});
  }

private:
  static std::condition_variable condition_;
  static std::mutex mutex_;
  static bool spin_exited_;
};

std::condition_variable ShutdownTrackingExecutor::condition_;
std::mutex ShutdownTrackingExecutor::mutex_;
bool ShutdownTrackingExecutor::spin_exited_ = false;

TEST(TestComponentManagerIsolatedShutdown, destruction_after_context_shutdown_does_not_hang)
{
  rclcpp::init(0, nullptr);

  auto container_executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  using IsolatedManager =
    rclcpp_components::ComponentManagerIsolated<ShutdownTrackingExecutor>;
  auto manager = std::make_shared<IsolatedManager>(
    container_executor, "ShutdownTestComponentManager");
  auto client_node = rclcpp::Node::make_shared("isolated_shutdown_test_client");

  container_executor->add_node(manager);
  container_executor->add_node(client_node);

  auto client = client_node->create_client<composition_interfaces::srv::LoadNode>(
    "/ShutdownTestComponentManager/_container/load_node");
  ASSERT_TRUE(client->wait_for_service(20s));

  auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  request->package_name = "rclcpp_components";
  request->plugin_name = "test_rclcpp_components::TestComponentFoo";
  auto future = client->async_send_request(request);

  ASSERT_EQ(
    container_executor->spin_until_future_complete(future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);

  rclcpp::shutdown();
  ASSERT_TRUE(ShutdownTrackingExecutor::wait_for_spin_to_exit(5s));

  container_executor->remove_node(client_node);
  container_executor->remove_node(manager);
  manager.reset();
}
