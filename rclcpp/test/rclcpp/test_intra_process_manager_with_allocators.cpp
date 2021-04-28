// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <gmock/gmock.h>

#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "test_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

// For demonstration purposes only, not necessary for allocator_traits
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom allocator. Counts calls to allocate and deallocate.
template<typename T = void>
struct MyAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }

  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }

  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return false;
}

/*
   This tests the case where a custom allocator is used correctly, i.e. the same
   custom allocator on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "custom_allocator_test",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true));

  uint32_t counter = 0;
  auto callback =
    [&counter](std::shared_ptr<const test_msgs::msg::Empty>) {
      ++counter;
    };

  using Alloc = MyAllocator<void>;
  auto alloc = std::make_shared<Alloc>();
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty>("custom_allocator_test", 10, publisher_options);

  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
  subscription_options.allocator = alloc;
  auto msg_mem_strat =
    std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty, Alloc>
    >(alloc);
  auto subscriber = node->create_subscription<test_msgs::msg::Empty>(
    "custom_allocator_test", 10, callback, subscription_options, msg_mem_strat);

  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  executor.add_node(node);

  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, test_msgs::msg::Empty>;
  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
  MessageAllocTraits::construct(message_alloc, ptr);
  std::unique_ptr<test_msgs::msg::Empty, MessageDeleter> msg(ptr, message_deleter);
  EXPECT_NO_THROW(
  {
    publisher->publish(std::move(msg));
    executor.spin_some();
  });
}

/*
   This tests the case where a custom allocator is used incorrectly, i.e. different
   custom allocators on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator_wrong) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "custom_allocator_test",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true));

  uint32_t counter = 0;
  auto callback =
    [&counter](std::shared_ptr<const test_msgs::msg::Empty>) {
      ++counter;
    };

  using Alloc = MyAllocator<void>;
  auto alloc = std::make_shared<Alloc>();
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty>("custom_allocator_test", 10, publisher_options);

  // Explicitly use std::allocator<void>
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subscription_options;
  auto std_alloc = std::make_shared<std::allocator<void>>();
  subscription_options.allocator = std_alloc;
  auto msg_mem_strat =
    std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty,
    std::allocator<void>>
    >(std_alloc);
  auto subscriber = node->create_subscription<test_msgs::msg::Empty>(
    "custom_allocator_test", 10, callback, subscription_options, msg_mem_strat);

  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  executor.add_node(node);

  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, test_msgs::msg::Empty>;
  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
  MessageAllocTraits::construct(message_alloc, ptr);
  std::unique_ptr<test_msgs::msg::Empty, MessageDeleter> msg(ptr, message_deleter);
  EXPECT_THROW(
  {
    publisher->publish(std::move(msg));
    executor.spin_some();
  }, std::runtime_error);
}
