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

#include <gtest/gtest.h>

#define RCLCPP_BUILDING_LIBRARY 1  // Prevent including unavailable symbols
#include <rclcpp/mapped_ring_buffer.hpp>

#include <memory>

/*
   Tests get_copy and pop on an empty mrb.
 */
TEST(TestMappedRingBuffer, empty) {
  // Cannot create a buffer of size zero.
  EXPECT_THROW(rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(0), std::invalid_argument);
  // Getting or popping an empty buffer should result in a nullptr.
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(1);

  std::unique_ptr<char> actual;
  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.pop_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);
}

/*
   Tests push_and_replace with a temporary object.
 */
TEST(TestMappedRingBuffer, temporary_l_value) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  // Pass in value with temporary object
  mrb.push_and_replace(1, std::unique_ptr<char>(new char('a')));

  std::unique_ptr<char> actual;
  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.pop_at_key(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);
}

/*
   Tests normal usage of the mrb.
 */
TEST(TestMappedRingBuffer, nominal) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::unique_ptr<char> expected(new char('a'));
  // Store expected value's address for later comparison.
  char * expected_orig = expected.get();

  EXPECT_FALSE(mrb.push_and_replace(1, expected));

  std::unique_ptr<char> actual;
  mrb.get_copy_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_NE(expected_orig, actual.get());

  mrb.pop_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected_orig, actual.get());

  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);

  expected.reset(new char('a'));
  EXPECT_FALSE(mrb.push_and_replace(1, expected));

  expected.reset(new char('b'));
  EXPECT_FALSE(mrb.push_and_replace(2, expected));

  expected.reset(new char('c'));
  EXPECT_TRUE(mrb.push_and_replace(3, expected));

  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.get_copy_at_key(2, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }

  mrb.get_copy_at_key(3, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('c', *actual);
  }
}

/*
   Tests get_ownership on a normal mrb.
 */
TEST(TestMappedRingBuffer, get_ownership) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::unique_ptr<char> expected(new char('a'));
  // Store expected value's address for later comparison.
  char * expected_orig = expected.get();

  EXPECT_FALSE(mrb.push_and_replace(1, expected));

  std::unique_ptr<char> actual;
  mrb.get_copy_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_NE(expected_orig, actual.get());

  mrb.get_ownership_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected_orig, actual.get());

  mrb.pop_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);  // The value should be the same.
  }
  EXPECT_NE(expected_orig, actual.get());  // Even though we pop'ed, we didn't get the original.

  mrb.get_copy_at_key(1, actual);
  EXPECT_EQ(nullptr, actual);
}

/*
   Tests the affect of reusing keys (non-unique keys) in a mrb.
 */
TEST(TestMappedRingBuffer, non_unique_keys) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);

  std::unique_ptr<char> input(new char('a'));
  mrb.push_and_replace(1, input);
  input.reset(new char('b'));

  // Different value, same key.
  mrb.push_and_replace(1, input);

  std::unique_ptr<char> actual;
  mrb.pop_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }

  actual = nullptr;
  mrb.pop_at_key(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }
}
