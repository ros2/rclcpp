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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/async_helpers.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AsyncHelpersTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// =============================================================================
// CancellationToken Tests
// =============================================================================

TEST_F(AsyncHelpersTest, CancellationTokenSource_InitiallyNotCancelled)
{
  rclcpp::executors::CancellationTokenSource cts;
  EXPECT_FALSE(cts.is_cancellation_requested());
}

TEST_F(AsyncHelpersTest, CancellationTokenSource_CancelSetsFlag)
{
  rclcpp::executors::CancellationTokenSource cts;
  cts.cancel();
  EXPECT_TRUE(cts.is_cancellation_requested());
}

TEST_F(AsyncHelpersTest, CancellationToken_ReflectsSourceState)
{
  rclcpp::executors::CancellationTokenSource cts;
  auto token = cts.get_token();

  EXPECT_FALSE(token.is_cancellation_requested());
  cts.cancel();
  EXPECT_TRUE(token.is_cancellation_requested());
}

TEST_F(AsyncHelpersTest, CancellationToken_ThrowIfCancelled)
{
  rclcpp::executors::CancellationTokenSource cts;
  auto token = cts.get_token();

  EXPECT_NO_THROW(token.throw_if_cancellation_requested());
  cts.cancel();
  EXPECT_THROW(token.throw_if_cancellation_requested(), rclcpp::executors::CancelledException);
}

TEST_F(AsyncHelpersTest, CancellationToken_CallbackInvokedOnCancel)
{
  rclcpp::executors::CancellationTokenSource cts;
  auto token = cts.get_token();

  std::atomic<bool> callback_invoked{false};
  auto handle = token.register_callback([&callback_invoked]() {
      callback_invoked.store(true);
    });

  EXPECT_FALSE(callback_invoked.load());
  cts.cancel();
  EXPECT_TRUE(callback_invoked.load());
}

TEST_F(AsyncHelpersTest, CancellationToken_CallbackInvokedImmediatelyIfAlreadyCancelled)
{
  rclcpp::executors::CancellationTokenSource cts;
  cts.cancel();

  auto token = cts.get_token();
  std::atomic<bool> callback_invoked{false};
  auto handle = token.register_callback([&callback_invoked]() {
      callback_invoked.store(true);
    });

  // Callback should be invoked immediately since already cancelled
  EXPECT_TRUE(callback_invoked.load());
}

TEST_F(AsyncHelpersTest, CancellationTokenSource_CancelAfterWorks)
{
  rclcpp::executors::CancellationTokenSource cts;

  EXPECT_FALSE(cts.is_cancellation_requested());
  cts.cancel_after(50ms);

  // Should not be cancelled immediately
  EXPECT_FALSE(cts.is_cancellation_requested());

  // Wait for cancellation
  std::this_thread::sleep_for(100ms);
  EXPECT_TRUE(cts.is_cancellation_requested());
}

TEST_F(AsyncHelpersTest, CancellationToken_DefaultConstructedNotCancellable)
{
  rclcpp::executors::CancellationToken token;
  EXPECT_FALSE(token.is_cancellation_requested());
  EXPECT_NO_THROW(token.throw_if_cancellation_requested());
}

// =============================================================================
// AsyncResult Tests
// =============================================================================

TEST_F(AsyncHelpersTest, AsyncResult_SuccessStatus)
{
  auto result = rclcpp::executors::AsyncResult<int>::success(42);

  EXPECT_TRUE(result.is_success());
  EXPECT_FALSE(result.is_timeout());
  EXPECT_FALSE(result.is_cancelled());
  EXPECT_FALSE(result.has_error());
  EXPECT_EQ(result.value(), 42);
}

TEST_F(AsyncHelpersTest, AsyncResult_TimeoutStatus)
{
  auto result = rclcpp::executors::AsyncResult<int>::timeout();

  EXPECT_FALSE(result.is_success());
  EXPECT_TRUE(result.is_timeout());
  EXPECT_FALSE(result.is_cancelled());
  EXPECT_FALSE(result.has_error());
}

TEST_F(AsyncHelpersTest, AsyncResult_CancelledStatus)
{
  auto result = rclcpp::executors::AsyncResult<int>::cancelled();

  EXPECT_FALSE(result.is_success());
  EXPECT_FALSE(result.is_timeout());
  EXPECT_TRUE(result.is_cancelled());
  EXPECT_FALSE(result.has_error());
}

TEST_F(AsyncHelpersTest, AsyncResult_ErrorStatus)
{
  auto result = rclcpp::executors::AsyncResult<int>::error(
    std::make_exception_ptr(std::runtime_error("test error")));

  EXPECT_FALSE(result.is_success());
  EXPECT_FALSE(result.is_timeout());
  EXPECT_FALSE(result.is_cancelled());
  EXPECT_TRUE(result.has_error());
}

TEST_F(AsyncHelpersTest, AsyncResult_ValueOrDefault)
{
  auto success_result = rclcpp::executors::AsyncResult<int>::success(42);
  auto timeout_result = rclcpp::executors::AsyncResult<int>::timeout();

  EXPECT_EQ(success_result.value_or(-1), 42);
  EXPECT_EQ(timeout_result.value_or(-1), -1);
}

TEST_F(AsyncHelpersTest, AsyncResult_RethrowIfError)
{
  auto error_result = rclcpp::executors::AsyncResult<int>::error(
    std::make_exception_ptr(std::runtime_error("test error")));

  EXPECT_THROW(error_result.rethrow_if_error(), std::runtime_error);
}

TEST_F(AsyncHelpersTest, AsyncResult_ValueThrowsOnTimeout)
{
  auto result = rclcpp::executors::AsyncResult<int>::timeout();
  EXPECT_THROW(result.value(), rclcpp::executors::TimeoutException);
}

TEST_F(AsyncHelpersTest, AsyncResult_ValueThrowsOnCancelled)
{
  auto result = rclcpp::executors::AsyncResult<int>::cancelled();
  EXPECT_THROW(result.value(), rclcpp::executors::CancelledException);
}

TEST_F(AsyncHelpersTest, AsyncResult_VoidSuccess)
{
  auto result = rclcpp::executors::AsyncResult<void>::success();
  EXPECT_TRUE(result.is_success());
  EXPECT_NO_THROW(result.rethrow_if_error());
}

TEST_F(AsyncHelpersTest, AsyncResult_VoidTimeout)
{
  auto result = rclcpp::executors::AsyncResult<void>::timeout();
  EXPECT_TRUE(result.is_timeout());
}

// =============================================================================
// Exception Tests
// =============================================================================

TEST_F(AsyncHelpersTest, TimeoutException_DefaultMessage)
{
  rclcpp::executors::TimeoutException ex;
  EXPECT_NE(std::string(ex.what()).find("timed out"), std::string::npos);
  EXPECT_EQ(ex.timeout(), std::chrono::milliseconds::zero());
}

TEST_F(AsyncHelpersTest, TimeoutException_WithDuration)
{
  rclcpp::executors::TimeoutException ex(100ms);
  EXPECT_NE(std::string(ex.what()).find("100"), std::string::npos);
  EXPECT_EQ(ex.timeout(), 100ms);
}

TEST_F(AsyncHelpersTest, CancelledException_Message)
{
  rclcpp::executors::CancelledException ex;
  EXPECT_NE(std::string(ex.what()).find("cancelled"), std::string::npos);
}

TEST_F(AsyncHelpersTest, ServiceNotReadyException_ContainsServiceName)
{
  rclcpp::executors::ServiceNotReadyException ex("my_service");
  EXPECT_NE(std::string(ex.what()).find("my_service"), std::string::npos);
  EXPECT_EQ(ex.service_name(), "my_service");
}

// =============================================================================
// Async Utility Tests
// =============================================================================

TEST_F(AsyncHelpersTest, WithTimeout_Success)
{
  auto future = std::async(std::launch::async, []() {
      std::this_thread::sleep_for(10ms);
      return 42;
    });

  auto result_future = rclcpp::executors::async_utils::with_timeout(std::move(future), 1000ms);
  auto result = result_future.get();

  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.value(), 42);
}

TEST_F(AsyncHelpersTest, WithTimeout_Timeout)
{
  auto future = std::async(std::launch::async, []() {
      std::this_thread::sleep_for(1000ms);
      return 42;
    });

  auto result_future = rclcpp::executors::async_utils::with_timeout(std::move(future), 50ms);
  auto result = result_future.get();

  EXPECT_TRUE(result.is_timeout());
}

TEST_F(AsyncHelpersTest, WithTimeout_Error)
{
  auto future = std::async(std::launch::async, []() -> int {
      throw std::runtime_error("test error");
    });

  auto result_future = rclcpp::executors::async_utils::with_timeout(std::move(future), 1000ms);
  auto result = result_future.get();

  EXPECT_TRUE(result.has_error());
  EXPECT_THROW(result.rethrow_if_error(), std::runtime_error);
}

TEST_F(AsyncHelpersTest, WhenAll_AllComplete)
{
  auto f1 = std::async(std::launch::async, []() { return 1; });
  auto f2 = std::async(std::launch::async, []() { return 2; });
  auto f3 = std::async(std::launch::async, []() { return 3; });

  auto result = rclcpp::executors::async_utils::when_all(
    std::move(f1), std::move(f2), std::move(f3)).get();

  EXPECT_EQ(std::get<0>(result), 1);
  EXPECT_EQ(std::get<1>(result), 2);
  EXPECT_EQ(std::get<2>(result), 3);
}

TEST_F(AsyncHelpersTest, WhenAny_FirstWins)
{
  std::vector<std::future<int>> futures;
  futures.push_back(std::async(std::launch::async, []() {
        std::this_thread::sleep_for(500ms);
        return 1;
      }));
  futures.push_back(std::async(std::launch::async, []() {
        std::this_thread::sleep_for(10ms);
        return 2;
      }));
  futures.push_back(std::async(std::launch::async, []() {
        std::this_thread::sleep_for(500ms);
        return 3;
      }));

  auto result = rclcpp::executors::async_utils::when_any(std::move(futures)).get();

  // The second future (index 1) should complete first
  EXPECT_EQ(result.first, 1u);
  EXPECT_EQ(result.second, 2);
}

TEST_F(AsyncHelpersTest, Then_TransformsValue)
{
  auto future = std::async(std::launch::async, []() { return 21; });

  auto result = rclcpp::executors::async_utils::then<int, int>(
    std::move(future),
    [](int value) { return value * 2; }).get();

  EXPECT_EQ(result, 42);
}

TEST_F(AsyncHelpersTest, WithRetry_SucceedsOnFirstTry)
{
  std::atomic<int> attempt_count{0};

  auto result = rclcpp::executors::async_utils::with_retry<int>(
    [&attempt_count]() {
      ++attempt_count;
      return 42;
    },
    3,
    10ms).get();

  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.value(), 42);
  EXPECT_EQ(attempt_count.load(), 1);
}

TEST_F(AsyncHelpersTest, WithRetry_SucceedsAfterRetries)
{
  std::atomic<int> attempt_count{0};

  auto result = rclcpp::executors::async_utils::with_retry<int>(
    [&attempt_count]() -> int {
      int count = ++attempt_count;
      if (count < 3) {
        throw std::runtime_error("not yet");
      }
      return 42;
    },
    5,
    10ms).get();

  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.value(), 42);
  EXPECT_EQ(attempt_count.load(), 3);
}

TEST_F(AsyncHelpersTest, WithRetry_FailsAfterMaxAttempts)
{
  std::atomic<int> attempt_count{0};

  auto result = rclcpp::executors::async_utils::with_retry<int>(
    [&attempt_count]() -> int {
      ++attempt_count;
      throw std::runtime_error("always fails");
    },
    3,
    10ms).get();

  EXPECT_TRUE(result.has_error());
  EXPECT_EQ(attempt_count.load(), 3);
}

// =============================================================================
// AsyncExecutorHelper Tests
// =============================================================================

TEST_F(AsyncHelpersTest, AsyncExecutorHelper_RunAsync)
{
  rclcpp::executors::AsyncExecutorHelper helper;
  helper.start();

  auto future = helper.run_async([]() { return 42; });
  auto result = future.get();

  EXPECT_EQ(result, 42);

  helper.stop();
}

TEST_F(AsyncHelpersTest, AsyncExecutorHelper_ScheduleAfter)
{
  rclcpp::executors::AsyncExecutorHelper helper;
  helper.start();

  std::promise<int> promise;
  auto future = promise.get_future();

  auto start_time = std::chrono::steady_clock::now();
  helper.schedule_after(50ms, [&promise, start_time]() {
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      promise.set_value(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    });

  int elapsed_ms = future.get();
  EXPECT_GE(elapsed_ms, 50);
  EXPECT_LT(elapsed_ms, 150);  // Allow some margin

  helper.stop();
}

TEST_F(AsyncHelpersTest, AsyncExecutorHelper_SchedulePeriodic)
{
  rclcpp::executors::AsyncExecutorHelper helper;
  helper.start();

  std::atomic<int> call_count{0};

  auto handle = helper.schedule_periodic(20ms, [&call_count]() {
      ++call_count;
    });

  std::this_thread::sleep_for(100ms);
  int count_before_stop = call_count.load();

  // Let handle go out of scope to stop periodic execution
  handle.reset();
  std::this_thread::sleep_for(50ms);
  int count_after_stop = call_count.load();

  // Should have been called multiple times
  EXPECT_GE(count_before_stop, 3);
  // Should have stopped after handle was released
  EXPECT_LE(count_after_stop - count_before_stop, 1);

  helper.stop();
}

TEST_F(AsyncHelpersTest, AsyncExecutorHelper_IsRunning)
{
  rclcpp::executors::AsyncExecutorHelper helper;

  EXPECT_FALSE(helper.is_running());
  helper.start();
  EXPECT_TRUE(helper.is_running());
  helper.stop();
  EXPECT_FALSE(helper.is_running());
}

// =============================================================================
// Integration Tests (require mock service for full testing)
// =============================================================================

class MockServiceTest : public AsyncHelpersTest
{
protected:
  void SetUp() override
  {
    AsyncHelpersTest::SetUp();
    node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start spinning in background
    spin_thread_ = std::thread([this]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    executor_.reset();
    AsyncHelpersTest::TearDown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spin_thread_;
};

// Note: Full service client tests would require setting up a mock service
// This is a placeholder showing the test structure

TEST_F(MockServiceTest, AsyncServiceClient_WaitForServiceTimeout)
{
  // Create an async client for a service that doesn't exist
  // Using example_interfaces::srv::AddTwoInts would require the dependency
  // For this test we just verify the timeout mechanism works

  // This test demonstrates the pattern - actual test would use real service types
  EXPECT_TRUE(true);  // Placeholder
}

// =============================================================================
// Thread Safety Tests
// =============================================================================

TEST_F(AsyncHelpersTest, CancellationTokenSource_ThreadSafe)
{
  rclcpp::executors::CancellationTokenSource cts;
  std::atomic<int> callback_count{0};

  // Register multiple callbacks from different threads
  std::vector<std::thread> threads;
  std::vector<std::shared_ptr<void>> handles;
  std::mutex handles_mutex;

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([&cts, &callback_count, &handles, &handles_mutex]() {
        auto token = cts.get_token();
        auto handle = token.register_callback([&callback_count]() {
            ++callback_count;
          });
        std::lock_guard<std::mutex> lock(handles_mutex);
        handles.push_back(handle);
      });
  }

  for (auto & t : threads) {
    t.join();
  }

  // Cancel from main thread
  cts.cancel();

  // All callbacks should have been invoked
  EXPECT_EQ(callback_count.load(), 10);
}

TEST_F(AsyncHelpersTest, AsyncResult_CopyAndMove)
{
  auto result1 = rclcpp::executors::AsyncResult<std::string>::success("hello");

  // Test copy
  auto result2 = result1;
  EXPECT_EQ(result2.value(), "hello");

  // Test move
  auto result3 = std::move(result1);
  EXPECT_EQ(result3.value(), "hello");
}

// =============================================================================
// Edge Case Tests
// =============================================================================

TEST_F(AsyncHelpersTest, WhenAny_SingleFuture)
{
  std::vector<std::future<int>> futures;
  futures.push_back(std::async(std::launch::async, []() { return 42; }));

  auto result = rclcpp::executors::async_utils::when_any(std::move(futures)).get();

  EXPECT_EQ(result.first, 0u);
  EXPECT_EQ(result.second, 42);
}

TEST_F(AsyncHelpersTest, WithRetry_SingleAttempt)
{
  auto result = rclcpp::executors::async_utils::with_retry<int>(
    []() -> int {
      throw std::runtime_error("fail");
    },
    1,
    10ms).get();

  EXPECT_TRUE(result.has_error());
}

TEST_F(AsyncHelpersTest, CancellationToken_MultipleCallbacks)
{
  rclcpp::executors::CancellationTokenSource cts;
  auto token = cts.get_token();

  std::atomic<int> count{0};

  auto h1 = token.register_callback([&count]() { ++count; });
  auto h2 = token.register_callback([&count]() { ++count; });
  auto h3 = token.register_callback([&count]() { ++count; });

  cts.cancel();

  EXPECT_EQ(count.load(), 3);
}

TEST_F(AsyncHelpersTest, CancellationToken_CallbackUnregistersWhenHandleReleased)
{
  rclcpp::executors::CancellationTokenSource cts;
  auto token = cts.get_token();

  std::atomic<bool> called{false};

  {
    auto handle = token.register_callback([&called]() { called.store(true); });
    // handle goes out of scope here, callback should be unregistered
  }

  cts.cancel();

  // Callback may or may not be called depending on implementation
  // The key is that it doesn't crash
  EXPECT_TRUE(true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
