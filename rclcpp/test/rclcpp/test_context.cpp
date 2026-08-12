// Copyright 2023 Sony Group Corporation.
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
#include <thread>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(TestContext, check_pre_shutdown_callback_order) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  int result[4] = {0, 0, 0, 0};
  int index = 0;

  auto callback1 = [&result, &index]() {
      result[index] = 1;
      index++;
    };
  auto callback2 = [&result, &index]() {
      result[index] = 2;
      index++;
    };
  auto callback3 = [&result, &index]() {
      result[index] = 3;
      index++;
    };
  auto callback4 = [&result, &index]() {
      result[index] = 4;
      index++;
    };

  context->add_pre_shutdown_callback(callback1);
  context->add_pre_shutdown_callback(callback2);
  context->add_pre_shutdown_callback(callback3);
  context->add_pre_shutdown_callback(callback4);

  context->shutdown("for test");

  EXPECT_TRUE(result[0] == 1 && result[1] == 2 && result[2] == 3 && result[3] == 4);
}

TEST(TestContext, check_on_shutdown_callback_order) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  int result[4] = {0, 0, 0, 0};
  int index = 0;

  auto callback1 = [&result, &index]() {
      result[index] = 1;
      index++;
    };
  auto callback2 = [&result, &index]() {
      result[index] = 2;
      index++;
    };
  auto callback3 = [&result, &index]() {
      result[index] = 3;
      index++;
    };
  auto callback4 = [&result, &index]() {
      result[index] = 4;
      index++;
    };

  context->add_on_shutdown_callback(callback1);
  context->add_on_shutdown_callback(callback2);
  context->add_on_shutdown_callback(callback3);
  context->add_on_shutdown_callback(callback4);

  context->shutdown("for test");

  EXPECT_TRUE(result[0] == 1 && result[1] == 2 && result[2] == 3 && result[3] == 4);
}

TEST(TestContext, check_mixed_register_shutdown_callback_order) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  int result[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int index = 0;

  auto callback1 = [&result, &index]() {
      result[index] = 1;
      index++;
    };
  auto callback2 = [&result, &index]() {
      result[index] = 2;
      index++;
    };
  auto callback3 = [&result, &index]() {
      result[index] = 3;
      index++;
    };
  auto callback4 = [&result, &index]() {
      result[index] = 4;
      index++;
    };
  auto callback5 = [&result, &index]() {
      result[index] = 5;
      index++;
    };
  auto callback6 = [&result, &index]() {
      result[index] = 6;
      index++;
    };
  auto callback7 = [&result, &index]() {
      result[index] = 7;
      index++;
    };
  auto callback8 = [&result, &index]() {
      result[index] = 8;
      index++;
    };

  // Mixed register
  context->add_pre_shutdown_callback(callback1);
  context->add_on_shutdown_callback(callback5);
  context->add_pre_shutdown_callback(callback2);
  context->add_on_shutdown_callback(callback6);
  context->add_pre_shutdown_callback(callback3);
  context->add_on_shutdown_callback(callback7);
  context->add_pre_shutdown_callback(callback4);
  context->add_on_shutdown_callback(callback8);

  context->shutdown("for test");

  EXPECT_TRUE(
    result[0] == 1 && result[1] == 2 && result[2] == 3 && result[3] == 4 &&
    result[4] == 5 && result[5] == 6 && result[6] == 7 && result[7] == 8);
}

TEST(TestContext, check_pre_shutdown_callback_order_after_del) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  int result[4] = {0, 0, 0, 0};
  int index = 0;

  auto callback1 = [&result, &index]() {
      result[index] = 1;
      index++;
    };
  auto callback2 = [&result, &index]() {
      result[index] = 2;
      index++;
    };
  auto callback3 = [&result, &index]() {
      result[index] = 3;
      index++;
    };
  auto callback4 = [&result, &index]() {
      result[index] = 4;
      index++;
    };

  context->add_pre_shutdown_callback(callback1);
  auto callback_handle = context->add_pre_shutdown_callback(callback2);
  context->add_pre_shutdown_callback(callback3);
  context->add_pre_shutdown_callback(callback4);

  EXPECT_TRUE(context->remove_pre_shutdown_callback(callback_handle));
  EXPECT_FALSE(context->remove_pre_shutdown_callback(callback_handle));

  context->shutdown("for test");

  EXPECT_TRUE(result[0] == 1 && result[1] == 3 && result[2] == 4 && result[3] == 0);
}

TEST(TestContext, check_on_shutdown_callback_order_after_del) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  int result[4] = {0, 0, 0, 0};
  int index = 0;

  auto callback1 = [&result, &index]() {
      result[index] = 1;
      index++;
    };
  auto callback2 = [&result, &index]() {
      result[index] = 2;
      index++;
    };
  auto callback3 = [&result, &index]() {
      result[index] = 3;
      index++;
    };
  auto callback4 = [&result, &index]() {
      result[index] = 4;
      index++;
    };

  context->add_on_shutdown_callback(callback1);
  auto callback_handle = context->add_on_shutdown_callback(callback2);
  context->add_on_shutdown_callback(callback3);
  context->add_on_shutdown_callback(callback4);

  EXPECT_TRUE(context->remove_on_shutdown_callback(callback_handle));
  EXPECT_FALSE(context->remove_on_shutdown_callback(callback_handle));

  context->shutdown("for test");

  EXPECT_TRUE(result[0] == 1 && result[1] == 3 && result[2] == 4 && result[3] == 0);
}

// This test checks that a reentrant call to shutdown(), e.g. issued from a
// pre_shutdown callback, returns false instead of running the entire shutdown
// sequence again (which used to recurse into the pre_shutdown callbacks and
// call rcl_shutdown() twice).
TEST(TestContext, reentrant_shutdown_from_pre_shutdown_callback) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  size_t pre_shutdown_calls = 0;
  size_t on_shutdown_calls = 0;
  bool reentrant_shutdown_result = true;

  context->add_pre_shutdown_callback(
    [&context, &pre_shutdown_calls, &reentrant_shutdown_result]() {
      pre_shutdown_calls++;
      reentrant_shutdown_result = context->shutdown("reentrant shutdown");
    });
  context->add_on_shutdown_callback(
    [&on_shutdown_calls]() {
      on_shutdown_calls++;
    });

  EXPECT_TRUE(context->shutdown("for test"));
  EXPECT_FALSE(reentrant_shutdown_result);
  EXPECT_EQ(pre_shutdown_calls, 1u);
  EXPECT_EQ(on_shutdown_calls, 1u);
  EXPECT_FALSE(context->is_valid());

  // shutdown() must work again after re-initialization
  context->init(0, nullptr);
  EXPECT_TRUE(context->is_valid());
  EXPECT_TRUE(context->shutdown("for test again"));
  EXPECT_FALSE(context->is_valid());
}

// This test checks that concurrent calls to shutdown() from multiple threads
// result in exactly one thread running the shutdown sequence, and the other
// threads returning false without error.
TEST(TestContext, concurrent_shutdown) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  std::atomic<size_t> pre_shutdown_calls{0};
  context->add_pre_shutdown_callback(
    [&pre_shutdown_calls]() {
      pre_shutdown_calls++;
    });

  constexpr size_t num_threads = 8;
  std::atomic<size_t> success_count{0};
  std::vector<std::thread> threads;
  threads.reserve(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    threads.emplace_back(
      [&context, &success_count]() {
        if (context->shutdown("concurrent shutdown")) {
          success_count++;
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_EQ(success_count, 1u);
  EXPECT_EQ(pre_shutdown_calls, 1u);
  EXPECT_FALSE(context->is_valid());
}

// This test checks that contexts will be properly destroyed when leaving a scope, after a
// guard condition has been created.
TEST(TestContext, check_context_destroyed) {
  rclcpp::Context::SharedPtr ctx;
  {
    ctx = std::make_shared<rclcpp::Context>();
    ctx->init(0, nullptr);

    auto group = std::make_shared<rclcpp::CallbackGroup>(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      ctx->weak_from_this(),
      false);

    rclcpp::GuardCondition::SharedPtr gc = group->get_notify_guard_condition();
    ASSERT_NE(gc, nullptr);

    ASSERT_EQ(ctx.use_count(), 1u);
  }

  ASSERT_EQ(ctx.use_count(), 1u);
}
