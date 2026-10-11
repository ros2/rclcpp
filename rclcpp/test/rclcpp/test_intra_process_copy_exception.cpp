// Copyright 2026 TinyGeek
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
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
struct CopyMessage
{
  int value = 42;
  bool fail_copy = false;
  CopyMessage() = default;
  CopyMessage(const CopyMessage & other)
  : value(other.value), fail_copy(other.fail_copy)
  {
    if (fail_copy) {throw std::runtime_error("controlled copy failure");}
  }
};
struct AllocationCounts
{
  std::size_t allocated = 0;
  std::size_t deallocated = 0;
};
template<typename T>
struct CountingAllocator
{
  using value_type = T;
  std::shared_ptr<AllocationCounts> counts;
  CountingAllocator()
  : counts(std::make_shared<AllocationCounts>()) {}
  explicit CountingAllocator(std::shared_ptr<AllocationCounts> value)
  : counts(std::move(value)) {}
  template<typename U>
  CountingAllocator(const CountingAllocator<U> & other)
  : counts(other.counts) {}
  T * allocate(std::size_t n)
  {
    auto pointer = std::allocator<T>{}.allocate(n);
    if constexpr (std::is_same<T, CopyMessage>::value) {counts->allocated += n;}
    return pointer;
  }
  void deallocate(T * pointer, std::size_t n)
  {
    if constexpr (std::is_same<T, CopyMessage>::value) {counts->deallocated += n;}
    std::allocator<T>{}.deallocate(pointer, n);
  }
  template<typename U>
  struct rebind {using other = CountingAllocator<U>;};
};
template<typename T, typename U>
bool operator==(const CountingAllocator<T> & a, const CountingAllocator<U> & b)
{return a.counts == b.counts;}
template<typename T, typename U>
bool operator!=(const CountingAllocator<T> & a, const CountingAllocator<U> & b)
{return !(a == b);}

using Alloc = CountingAllocator<CopyMessage>;
using Deleter = rclcpp::allocator::Deleter<Alloc, CopyMessage>;
using UniqueMessage = std::unique_ptr<CopyMessage, Deleter>;
using Base = rclcpp::experimental::SubscriptionIntraProcessBase;

class DataBuffer : public rclcpp::experimental::SubscriptionIntraProcessBuffer<
    CopyMessage, Alloc, Deleter, CopyMessage>
{
public:
  explicit DataBuffer(const std::shared_ptr<Alloc> & allocator)
  : SubscriptionIntraProcessBuffer(
      allocator, rclcpp::contexts::get_global_default_context(), "/copy", rclcpp::QoS(10),
      rclcpp::IntraProcessBufferType::UniquePtr) {}
  std::shared_ptr<void> take_data() override {return nullptr;}
  void execute(const std::shared_ptr<void> &) override {}
  UniqueMessage consume() {return buffer_->consume_unique();}
};

class ROSBuffer : public rclcpp::experimental::SubscriptionROSMsgIntraProcessBuffer<
    CopyMessage, Alloc, Deleter>
{
public:
  ROSBuffer()
  : SubscriptionROSMsgIntraProcessBuffer(
      rclcpp::contexts::get_global_default_context(), "/copy", rclcpp::QoS(10)) {}
  bool use_take_shared_method() const override {return false;}
  std::size_t available_capacity() const override {return 10;}
  bool is_ready(const rcl_wait_set_t &) override {return false;}
  std::shared_ptr<void> take_data() override {return nullptr;}
  void execute(const std::shared_ptr<void> &) override {}
  void provide_intra_process_message(ConstMessageSharedPtr) override
  {throw std::logic_error("unexpected shared delivery");}
  void provide_intra_process_message(MessageUniquePtr value) override {message = std::move(value);}
  UniqueMessage message;

protected:
  void trigger_guard_condition() override {gc_.trigger();}
};

struct Case {bool ros_only; bool fail_copy;};
class TestIntraProcessCopyException : public ::testing::TestWithParam<Case>
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_P(TestIntraProcessCopyException, balances_message_allocations)
{
  const auto param = GetParam();
  auto counts = std::make_shared<AllocationCounts>();
  auto allocator = std::make_shared<Alloc>(counts);
  {
    auto node = std::make_shared<rclcpp::Node>("copy_allocation_test");
    auto publisher = node->create_publisher<std_msgs::msg::String>("copy", 10);
    rclcpp::experimental::IntraProcessManager manager;
    const auto id = manager.add_publisher(publisher);
    std::shared_ptr<DataBuffer> data_a;
    std::shared_ptr<DataBuffer> data_b;
    std::shared_ptr<ROSBuffer> ros_a;
    std::shared_ptr<ROSBuffer> ros_b;
    if (param.ros_only) {
      ros_a = std::make_shared<ROSBuffer>();
      ros_b = std::make_shared<ROSBuffer>();
      manager.add_subscription<CopyMessage, CountingAllocator<void>>(ros_a);
      manager.add_subscription<CopyMessage, CountingAllocator<void>>(ros_b);
    } else {
      data_a = std::make_shared<DataBuffer>(allocator);
      data_b = std::make_shared<DataBuffer>(allocator);
      manager.add_subscription<CopyMessage, CountingAllocator<void>>(data_a);
      manager.add_subscription<CopyMessage, CountingAllocator<void>>(data_b);
    }
    auto pointer = std::allocator_traits<Alloc>::allocate(*allocator, 1);
    std::allocator_traits<Alloc>::construct(*allocator, pointer);
    pointer->fail_copy = param.fail_copy;
    Deleter deleter(allocator.get());
    UniqueMessage message(pointer, deleter);
    const auto original = message.get();
    auto publish = [&]() {
        manager.do_intra_process_publish<CopyMessage, CopyMessage, CountingAllocator<void>>(
          id, std::move(message), *allocator);
      };
    if (param.fail_copy) {
      EXPECT_THROW(publish(), std::runtime_error);
    } else {
      ASSERT_NO_THROW(publish());
      auto first = param.ros_only ? std::move(ros_a->message) : data_a->consume();
      auto second = param.ros_only ? std::move(ros_b->message) : data_b->consume();
      ASSERT_NE(nullptr, first);
      ASSERT_NE(nullptr, second);
      EXPECT_EQ(42, first->value);
      EXPECT_EQ(42, second->value);
      EXPECT_NE(first.get(), second.get());
      EXPECT_EQ(original, second.get());
      first->value = 9;
      EXPECT_EQ(42, second->value);
    }
  }
  EXPECT_EQ(2u, counts->allocated);
  EXPECT_EQ(counts->allocated, counts->deallocated);
}
INSTANTIATE_TEST_SUITE_P(
  BufferKinds, TestIntraProcessCopyException,
  ::testing::Values(Case{false, false}, Case{false, true}, Case{true, false}, Case{true, true}));
}  // namespace
