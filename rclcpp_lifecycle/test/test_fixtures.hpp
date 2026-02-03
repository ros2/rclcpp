// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef TEST_FIXTURES_HPP_
#define TEST_FIXTURES_HPP_

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace lifecycle_test
{

/**
 * @brief State history entry for tracking state transitions
 */
struct StateHistoryEntry
{
  uint8_t state_id;
  std::string state_label;
  std::chrono::steady_clock::time_point timestamp;
  std::string transition_label;
};

/**
 * @brief Test lifecycle node with enhanced features for testing
 *
 * This class extends LifecycleNode to add state history tracking
 * and configurable callback behaviors for testing purposes.
 */
class TestLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TestLifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(node_name, options),
    configure_should_fail_(false),
    activate_should_fail_(false),
    deactivate_should_fail_(false),
    cleanup_should_fail_(false),
    shutdown_should_fail_(false),
    configure_delay_ms_(0),
    activate_delay_ms_(0),
    configure_callback_count_(0),
    activate_callback_count_(0),
    deactivate_callback_count_(0),
    cleanup_callback_count_(0),
    shutdown_callback_count_(0),
    error_callback_count_(0)
  {
    record_state_transition("initial", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  /**
   * @brief Get the state history
   */
  std::vector<StateHistoryEntry> get_state_history() const
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return state_history_;
  }

  /**
   * @brief Get the size of state history
   */
  size_t get_state_history_size() const
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return state_history_.size();
  }

  /**
   * @brief Clear the state history
   */
  void clear_state_history()
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    state_history_.clear();
  }

  /**
   * @brief Get the previous state
   */
  uint8_t get_previous_state_id() const
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    if (state_history_.size() < 2) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    return state_history_[state_history_.size() - 2].state_id;
  }

  // Convenience methods
  bool is_active() const
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  bool is_inactive() const
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }

  bool is_configured() const
  {
    auto state = get_current_state().id();
    return state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
           state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  bool is_unconfigured() const
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  }

  bool is_finalized() const
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
  }

  // Configuration methods for test behavior
  void set_configure_should_fail(bool should_fail)
  {
    configure_should_fail_ = should_fail;
  }

  void set_activate_should_fail(bool should_fail)
  {
    activate_should_fail_ = should_fail;
  }

  void set_deactivate_should_fail(bool should_fail)
  {
    deactivate_should_fail_ = should_fail;
  }

  void set_cleanup_should_fail(bool should_fail)
  {
    cleanup_should_fail_ = should_fail;
  }

  void set_shutdown_should_fail(bool should_fail)
  {
    shutdown_should_fail_ = should_fail;
  }

  void set_configure_delay(int ms)
  {
    configure_delay_ms_ = ms;
  }

  void set_activate_delay(int ms)
  {
    activate_delay_ms_ = ms;
  }

  // Callback count accessors
  int get_configure_callback_count() const { return configure_callback_count_; }
  int get_activate_callback_count() const { return activate_callback_count_; }
  int get_deactivate_callback_count() const { return deactivate_callback_count_; }
  int get_cleanup_callback_count() const { return cleanup_callback_count_; }
  int get_shutdown_callback_count() const { return shutdown_callback_count_; }
  int get_error_callback_count() const { return error_callback_count_; }

  // Custom callback registration
  using TransitionCallback = std::function<void()>;

  void set_on_configure_callback(TransitionCallback callback)
  {
    custom_configure_callback_ = callback;
  }

  void set_on_activate_callback(TransitionCallback callback)
  {
    custom_activate_callback_ = callback;
  }

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    configure_callback_count_++;

    if (configure_delay_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(configure_delay_ms_));
    }

    if (custom_configure_callback_) {
      custom_configure_callback_();
    }

    if (configure_should_fail_) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    record_state_transition("configure", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    activate_callback_count_++;

    if (activate_delay_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(activate_delay_ms_));
    }

    if (custom_activate_callback_) {
      custom_activate_callback_();
    }

    if (activate_should_fail_) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    record_state_transition("activate", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    deactivate_callback_count_++;

    if (deactivate_should_fail_) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    record_state_transition("deactivate", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    cleanup_callback_count_++;

    if (cleanup_should_fail_) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    record_state_transition("cleanup", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    shutdown_callback_count_++;

    if (shutdown_should_fail_) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    record_state_transition("shutdown", lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state) override
  {
    (void)previous_state;
    error_callback_count_++;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void record_state_transition(const std::string & transition_label, uint8_t new_state_id)
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    StateHistoryEntry entry;
    entry.state_id = new_state_id;
    entry.timestamp = std::chrono::steady_clock::now();
    entry.transition_label = transition_label;

    switch (new_state_id) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        entry.state_label = "unconfigured";
        break;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        entry.state_label = "inactive";
        break;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        entry.state_label = "active";
        break;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
        entry.state_label = "finalized";
        break;
      default:
        entry.state_label = "unknown";
        break;
    }

    state_history_.push_back(entry);
  }

  mutable std::mutex history_mutex_;
  std::vector<StateHistoryEntry> state_history_;

  std::atomic<bool> configure_should_fail_;
  std::atomic<bool> activate_should_fail_;
  std::atomic<bool> deactivate_should_fail_;
  std::atomic<bool> cleanup_should_fail_;
  std::atomic<bool> shutdown_should_fail_;

  std::atomic<int> configure_delay_ms_;
  std::atomic<int> activate_delay_ms_;

  std::atomic<int> configure_callback_count_;
  std::atomic<int> activate_callback_count_;
  std::atomic<int> deactivate_callback_count_;
  std::atomic<int> cleanup_callback_count_;
  std::atomic<int> shutdown_callback_count_;
  std::atomic<int> error_callback_count_;

  TransitionCallback custom_configure_callback_;
  TransitionCallback custom_activate_callback_;
};

/**
 * @brief Base test fixture for lifecycle node tests
 *
 * Provides common setup/teardown for ROS2 context initialization
 * and lifecycle node creation.
 */
class LifecycleNodeTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestLifecycleNode>("test_lifecycle_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(node_->get_node_base_interface());
    node_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief Helper to spin executor for a duration
   */
  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some(10ms);
    }
  }

  /**
   * @brief Helper to wait for a condition with timeout
   */
  template<typename Predicate>
  bool wait_for_condition(Predicate pred, std::chrono::milliseconds timeout)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  /**
   * @brief Transition node through full lifecycle to active state
   */
  bool transition_to_active()
  {
    auto result = node_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return false;
    }

    result = node_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    return result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  /**
   * @brief Transition node from active back to inactive
   */
  bool transition_to_inactive()
  {
    auto result = node_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    return result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }

  /**
   * @brief Transition node from inactive back to unconfigured
   */
  bool transition_to_unconfigured()
  {
    auto result = node_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    return result.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

/**
 * @brief Thread-safe test fixture for concurrent testing
 *
 * Uses multi-threaded executor and provides utilities for
 * testing thread safety.
 */
class ThreadSafeLifecycleTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestLifecycleNode>("test_thread_safe_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    executor_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    node_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief Run multiple functions concurrently
   */
  template<typename Func>
  void run_concurrent(Func func, int num_threads)
  {
    std::vector<std::thread> threads;
    std::atomic<int> ready_count{0};
    std::atomic<bool> start{false};

    for (int i = 0; i < num_threads; ++i) {
      threads.emplace_back([&func, &ready_count, &start, i]() {
        ready_count++;
        while (!start) {
          std::this_thread::yield();
        }
        func(i);
      });
    }

    // Wait for all threads to be ready
    while (ready_count < num_threads) {
      std::this_thread::yield();
    }

    // Start all threads simultaneously
    start = true;

    // Join all threads
    for (auto & t : threads) {
      t.join();
    }
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
};

/**
 * @brief Custom exception for transition errors in tests
 */
class TransitionException : public std::runtime_error
{
public:
  TransitionException(
    const std::string & message,
    uint8_t from_state,
    uint8_t to_state,
    uint8_t transition_id)
  : std::runtime_error(message),
    from_state_(from_state),
    to_state_(to_state),
    transition_id_(transition_id)
  {
  }

  uint8_t get_from_state() const { return from_state_; }
  uint8_t get_to_state() const { return to_state_; }
  uint8_t get_transition_id() const { return transition_id_; }

private:
  uint8_t from_state_;
  uint8_t to_state_;
  uint8_t transition_id_;
};

}  // namespace lifecycle_test

#endif  // TEST_FIXTURES_HPP_
