// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/allocator/allocator_common.hpp"

TEST(TestAllocatorCommon, retyped_allocate) {
  std::allocator<int> allocator;
  void * untyped_allocator = &allocator;
  void * allocated_mem =
    rclcpp::allocator::retyped_allocate<std::allocator<char>>(1u, untyped_allocator);
  // The more natural check here is ASSERT_NE(nullptr, ptr), but clang static
  // analysis throws a false-positive memory leak warning.  Use ASSERT_TRUE instead.
  ASSERT_TRUE(nullptr != allocated_mem);

  auto code = [&untyped_allocator, allocated_mem]() {
      rclcpp::allocator::retyped_deallocate<int, std::allocator<int>>(
        allocated_mem, untyped_allocator);
    };
  EXPECT_NO_THROW(code());

  allocated_mem = allocator.allocate(1);
  // The more natural check here is ASSERT_NE(nullptr, ptr), but clang static
  // analysis throws a false-positive memory leak warning.  Use ASSERT_TRUE instead.
  ASSERT_TRUE(nullptr != allocated_mem);
  void * reallocated_mem =
    rclcpp::allocator::retyped_reallocate<int, std::allocator<int>>(
    allocated_mem, 2u, untyped_allocator);
  // The more natural check here is ASSERT_NE(nullptr, ptr), but clang static
  // analysis throws a false-positive memory leak warning.  Use ASSERT_TRUE instead.
  ASSERT_TRUE(nullptr != reallocated_mem);

  auto code2 = [&untyped_allocator, reallocated_mem]() {
      rclcpp::allocator::retyped_deallocate<int, std::allocator<int>>(
        reallocated_mem, untyped_allocator);
    };
  EXPECT_NO_THROW(code2());
}

TEST(TestAllocatorCommon, get_rcl_allocator) {
  std::allocator<int> allocator;
  auto rcl_allocator = rclcpp::allocator::get_rcl_allocator<int>(allocator);
  EXPECT_NE(nullptr, rcl_allocator.allocate);
  EXPECT_NE(nullptr, rcl_allocator.deallocate);
  EXPECT_NE(nullptr, rcl_allocator.reallocate);
  EXPECT_NE(nullptr, rcl_allocator.zero_allocate);
  // Not testing state as that may or may not be null depending on platform
}

TEST(TestAllocatorCommon, get_void_rcl_allocator) {
  std::allocator<void> allocator;
  auto rcl_allocator =
    rclcpp::allocator::get_rcl_allocator<void, std::allocator<void>>(allocator);
  EXPECT_NE(nullptr, rcl_allocator.allocate);
  EXPECT_NE(nullptr, rcl_allocator.deallocate);
  EXPECT_NE(nullptr, rcl_allocator.reallocate);
  EXPECT_NE(nullptr, rcl_allocator.zero_allocate);
  // Not testing state as that may or may not be null depending on platform
}
