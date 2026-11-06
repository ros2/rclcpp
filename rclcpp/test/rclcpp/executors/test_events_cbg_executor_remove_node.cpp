// Copyright 2026 Fly4Future
// Copyright 2026 Czech Technical University in Prague
// Copyright 2026 Vojtech Spurny
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

#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "./test_waitable.hpp"

class TestEventsCBGExecutorRemoveNode : public testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}
};

/*
 * Allocator, that records when the memory of the shared_ptr control block
 * is released. This only happens after the last weak_ptr is gone.
 */
template<typename T>
struct ReleaseTrackingAllocator
{
  using value_type = T;

  explicit ReleaseTrackingAllocator(bool & released)
  : released(released) {}

  template<typename U>
  ReleaseTrackingAllocator(const ReleaseTrackingAllocator<U> & other)  // NOLINT
  : released(other.released) {}

  T * allocate(std::size_t n)
  {
    return std::allocator<T>{}.allocate(n);
  }

  void deallocate(T * p, std::size_t n)
  {
    released = true;
    std::allocator<T>{}.deallocate(p, n);
  }

  template<typename U>
  bool operator==(const ReleaseTrackingAllocator<U> & other) const
  {
    return &released == &other.released;
  }

  template<typename U>
  bool operator!=(const ReleaseTrackingAllocator<U> & other) const
  {
    return !(*this == other);
  }

  bool & released;
};

/*
 * Test that events, which were queued but not executed, do not keep the
 * entities of a removed node alive until the executor is destroyed.
 *
 * In a component container the executor outlives the components, and their
 * libraries are unloaded before the executor is destroyed. Releasing the
 * last weak_ptr to an entity at that point calls into unloaded code.
 */
TEST_F(TestEventsCBGExecutorRemoveNode, queued_events_released_on_remove_node)
{
  bool released = false;

  auto executor = std::make_shared<rclcpp::executors::EventsCBGExecutor>();

  {
    auto node = std::make_shared<rclcpp::Node>("test_events_cbg_executor_remove_node");
    auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto waitable = std::allocate_shared<TestWaitable>(
      ReleaseTrackingAllocator<TestWaitable>(released));
    node->get_node_waitables_interface()->add_waitable(waitable, cbg);

    executor->add_node(node);

    // not spinning, so the events stay queued in the executor
    waitable->trigger();
    waitable->trigger();

    executor->remove_node(node);

    node->get_node_waitables_interface()->remove_waitable(waitable, cbg);
  }

  // the executor is still alive here
  EXPECT_TRUE(released);
}
