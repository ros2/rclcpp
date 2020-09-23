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

#include "rclcpp/allocator/allocator_deleter.hpp"

#include "../../utils/rclcpp_gtest_macros.hpp"

TEST(TestAllocatorDeleter, construct_destruct) {
  std::allocator<int> allocator;

  rclcpp::allocator::AllocatorDeleter<std::allocator<int>> deleter;
  EXPECT_EQ(nullptr, deleter.get_allocator());
  deleter.set_allocator(&allocator);
  EXPECT_EQ(&allocator, deleter.get_allocator());

  rclcpp::allocator::AllocatorDeleter<std::allocator<int>> deleter2(&allocator);
  EXPECT_EQ(&allocator, deleter2.get_allocator());

  rclcpp::allocator::AllocatorDeleter<std::allocator<int>> deleter3(deleter2);
  EXPECT_EQ(&allocator, deleter3.get_allocator());
}

TEST(TestAllocatorDeleter, delete) {
  std::allocator<int> allocator;
  int * some_mem = allocator.allocate(1u);
  ASSERT_NE(nullptr, some_mem);

  rclcpp::allocator::AllocatorDeleter<std::allocator<int>> deleter(&allocator);
  EXPECT_NO_THROW(deleter(some_mem));
}

TEST(TestAllocatorDeleter, set_allocator_for_deleter_AllocatorDeleter) {
  using AllocatorT = std::allocator<int>;
  using DeleterT = rclcpp::allocator::AllocatorDeleter<AllocatorT>;
  AllocatorT allocator;
  DeleterT deleter(&allocator);

  std::allocator<int> allocator2;
  rclcpp::allocator::set_allocator_for_deleter<AllocatorT, AllocatorT>(&deleter, &allocator2);
  EXPECT_EQ(&allocator2, deleter.get_allocator());

  auto throwing_statement = [&allocator]() {
      DeleterT * null_del_ptr = nullptr;
      rclcpp::allocator::set_allocator_for_deleter<AllocatorT, AllocatorT>(
        null_del_ptr, &allocator);
    };
  RCLCPP_EXPECT_THROW_EQ(
    throwing_statement(),
    std::invalid_argument("Argument was NULL to set_allocator_for_deleter"));

  auto throwing_statement2 = [&deleter]() {
      AllocatorT * null_alloc_ptr = nullptr;
      rclcpp::allocator::set_allocator_for_deleter<AllocatorT, AllocatorT>(
        &deleter, null_alloc_ptr);
    };

  RCLCPP_EXPECT_THROW_EQ(
    throwing_statement2(),
    std::invalid_argument("Argument was NULL to set_allocator_for_deleter"));
}

TEST(TestAllocatorDeleter, set_allocator_for_deleter_std_default_delete) {
  using AllocatorT = std::allocator<int>;
  using DeleterT = std::default_delete<int>;
  auto not_throwing_statement = []() {
      AllocatorT allocator;
      DeleterT deleter;
      rclcpp::allocator::set_allocator_for_deleter<int, int>(&deleter, &allocator);
    };
  EXPECT_NO_THROW(not_throwing_statement());
}

TEST(TestAllocatorDeleter, set_allocator_for_deleter_unexpected_template) {
  class SomeAllocatorClass {};
  class SomeDeleterClass {};
  using AllocatorT = SomeAllocatorClass;
  using DeleterT = SomeDeleterClass;
  auto throwing_statement = []() {
      DeleterT deleter;
      AllocatorT allocator;
      rclcpp::allocator::set_allocator_for_deleter<AllocatorT, int, DeleterT>(&deleter, &allocator);
    };
  RCLCPP_EXPECT_THROW_EQ(
    throwing_statement(),
    std::runtime_error("Reached unexpected template specialization"));
}
