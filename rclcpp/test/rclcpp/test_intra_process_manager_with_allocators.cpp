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

#include <gmock/gmock.h>

#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "test_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

// For demonstration purposes only, not necessary for allocator_traits
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom allocator. Counts calls to allocate and deallocate.
template<typename T>
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

// Explicit specialization for void
template<>
struct MyAllocator<void>
{
public:
  using value_type = void;
  using pointer = void *;
  using const_pointer = const void *;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
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

template<
  typename PublishedMessageAllocatorT,
  typename PublisherAllocatorT,
  typename SubscribedMessageAllocatorT,
  typename SubscriptionAllocatorT,
  typename MessageMemoryStrategyAllocatorT,
  typename MemoryStrategyAllocatorT,
  typename ExpectedExceptionT
>
void
do_custom_allocator_test(
  PublishedMessageAllocatorT published_message_allocator,
  PublisherAllocatorT publisher_allocator,
  SubscribedMessageAllocatorT /* subscribed_message_allocator */,  // intentionally unused
  SubscriptionAllocatorT subscription_allocator,
  MessageMemoryStrategyAllocatorT message_memory_strategy,
  MemoryStrategyAllocatorT memory_strategy_allocator)
{
  using PublishedMessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, PublishedMessageAllocatorT>;
  using PublishedMessageAlloc = typename PublishedMessageAllocTraits::allocator_type;
  using PublishedMessageDeleter =
    rclcpp::allocator::Deleter<PublishedMessageAlloc, test_msgs::msg::Empty>;

  using SubscribedMessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, SubscribedMessageAllocatorT>;
  using SubscribedMessageAlloc = typename SubscribedMessageAllocTraits::allocator_type;
  using SubscribedMessageDeleter =
    rclcpp::allocator::Deleter<SubscribedMessageAlloc, test_msgs::msg::Empty>;

  // init and node
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "custom_allocator_test",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true));

  // publisher
  auto shared_publisher_allocator = std::make_shared<PublisherAllocatorT>(publisher_allocator);
  rclcpp::PublisherOptionsWithAllocator<PublisherAllocatorT> publisher_options;
  publisher_options.allocator = shared_publisher_allocator;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty>("custom_allocator_test", 10, publisher_options);

  // callback for subscription
  uint32_t counter = 0;
  std::promise<std::unique_ptr<test_msgs::msg::Empty, SubscribedMessageDeleter>> received_message;
  auto received_message_future = received_message.get_future();
  auto callback =
    [&counter, &received_message](
    std::unique_ptr<test_msgs::msg::Empty, SubscribedMessageDeleter> msg)
    {
      ++counter;
      received_message.set_value(std::move(msg));
    };

  // subscription
  auto shared_subscription_allocator =
    std::make_shared<SubscriptionAllocatorT>(subscription_allocator);
  rclcpp::SubscriptionOptionsWithAllocator<SubscriptionAllocatorT> subscription_options;
  subscription_options.allocator = shared_subscription_allocator;
  auto shared_message_strategy_allocator =
    std::make_shared<MessageMemoryStrategyAllocatorT>(message_memory_strategy);
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      test_msgs::msg::Empty,
      MessageMemoryStrategyAllocatorT
    >
    >(shared_message_strategy_allocator);
  using CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<decltype(callback)>::type;
  auto subscriber = node->create_subscription<
    test_msgs::msg::Empty,
    decltype(callback),
    SubscriptionAllocatorT,
    rclcpp::Subscription<CallbackMessageT, SubscriptionAllocatorT>,
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      MessageMemoryStrategyAllocatorT
    >
    >(
    "custom_allocator_test",
    10,
    std::forward<decltype(callback)>(callback),
    subscription_options,
    msg_mem_strat);

  // executor memory strategy
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  auto shared_memory_strategy_allocator = std::make_shared<MemoryStrategyAllocatorT>(
    memory_strategy_allocator);
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<MemoryStrategyAllocatorT>>(
    shared_memory_strategy_allocator);

  // executor
  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  executor.add_node(node);

  // rebind message_allocator to ensure correct type
  PublishedMessageDeleter message_deleter;
  PublishedMessageAlloc rebound_message_allocator = published_message_allocator;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &rebound_message_allocator);

  // allocate a message
  auto ptr = PublishedMessageAllocTraits::allocate(rebound_message_allocator, 1);
  PublishedMessageAllocTraits::construct(rebound_message_allocator, ptr);
  std::unique_ptr<test_msgs::msg::Empty, PublishedMessageDeleter> msg(ptr, message_deleter);

  // publisher and receive
  if constexpr (std::is_same_v<ExpectedExceptionT, void>) {
    // no exception expected
    EXPECT_NO_THROW(
    {
      publisher->publish(std::move(msg));
      executor.spin_until_future_complete(received_message_future, std::chrono::seconds(10));
    });
    EXPECT_EQ(ptr, received_message_future.get().get());
    EXPECT_EQ(1u, counter);
  } else {
    // exception expected
    EXPECT_THROW(
    {
      publisher->publish(std::move(msg));
      executor.spin_until_future_complete(received_message_future, std::chrono::seconds(10));
    }, ExpectedExceptionT);
  }
}

/*
   This tests the case where a custom allocator is used correctly, i.e. the same
   custom allocator on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator) {
  using PublishedMessageAllocatorT = std::allocator<void>;
  using PublisherAllocatorT = std::allocator<void>;
  using SubscribedMessageAllocatorT = std::allocator<void>;
  using SubscriptionAllocatorT = std::allocator<void>;
  using MessageMemoryStrategyAllocatorT = std::allocator<void>;
  using MemoryStrategyAllocatorT = std::allocator<void>;
  auto allocator = std::allocator<void>();
  do_custom_allocator_test<
    PublishedMessageAllocatorT,
    PublisherAllocatorT,
    SubscribedMessageAllocatorT,
    SubscriptionAllocatorT,
    MessageMemoryStrategyAllocatorT,
    MemoryStrategyAllocatorT,
    void  // no exception expected
  >(allocator, allocator, allocator, allocator, allocator, allocator);
}

/*
   This tests the case where a custom allocator is used incorrectly, i.e. different
   custom allocators on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator_wrong) {
  // explicitly use a different allocator here to provoke a failure
  using PublishedMessageAllocatorT = std::allocator<void>;
  using PublisherAllocatorT = std::allocator<void>;
  using SubscribedMessageAllocatorT = MyAllocator<void>;
  using SubscriptionAllocatorT = MyAllocator<void>;
  using MessageMemoryStrategyAllocatorT = MyAllocator<void>;
  using MemoryStrategyAllocatorT = std::allocator<void>;
  auto allocator = std::allocator<void>();
  auto my_allocator = MyAllocator<void>();
  do_custom_allocator_test<
    PublishedMessageAllocatorT,
    PublisherAllocatorT,
    SubscribedMessageAllocatorT,
    SubscriptionAllocatorT,
    MessageMemoryStrategyAllocatorT,
    MemoryStrategyAllocatorT,
    std::runtime_error  // expected exception
  >(allocator, allocator, my_allocator, my_allocator, my_allocator, allocator);
}
