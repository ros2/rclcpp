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

TEST(TestAllocatorCommon, get_rcl_allocator) {
  std::allocator<int> allocator;
  auto rcl_allocator = rclcpp::get_rcl_allocator(allocator);
  EXPECT_NE(nullptr, rcl_allocator.allocate);
  EXPECT_NE(nullptr, rcl_allocator.deallocate);
  EXPECT_NE(nullptr, rcl_allocator.reallocate);
  EXPECT_NE(nullptr, rcl_allocator.zero_allocate);
  // Not testing state as that may or may not be null depending on platform
}

TEST(TestAllocatorCommon, get_void_rcl_allocator) {
  std::allocator<void> allocator;
  auto rcl_allocator = rclcpp::get_rcl_allocator(allocator);
  EXPECT_NE(nullptr, rcl_allocator.allocate);
  EXPECT_NE(nullptr, rcl_allocator.deallocate);
  EXPECT_NE(nullptr, rcl_allocator.reallocate);
  EXPECT_NE(nullptr, rcl_allocator.zero_allocate);
  // Not testing state as that may or may not be null depending on platform
}
