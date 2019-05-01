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

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#define RCLCPP_BUILDING_LIBRARY 1  // Prevent including unavailable symbols
#include "rclcpp/mapped_ring_buffer.hpp"

/*
   Tests get_copy and pop on an empty mrb.
 */
TEST(TestMappedRingBuffer, empty) {
  // Cannot create a buffer of size zero.
  EXPECT_THROW(rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(0), std::invalid_argument);
  // Getting or popping an empty buffer should result in a nullptr.
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(1);

  std::unique_ptr<char> unique;
  mrb.get(1, unique);
  EXPECT_EQ(nullptr, unique);

  mrb.pop(1, unique);
  EXPECT_EQ(nullptr, unique);

  std::shared_ptr<const char> shared;
  mrb.get(1, shared);
  EXPECT_EQ(nullptr, shared);

  mrb.pop(1, shared);
  EXPECT_EQ(nullptr, shared);
}

/*
   Tests push_and_replace with a temporary object, and using
   get and pop methods with shared_ptr signature.
 */
TEST(TestMappedRingBuffer, temporary_l_value_with_shared_get_pop) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  // Pass in value with temporary object
  mrb.push_and_replace(1, std::shared_ptr<const char>(new char('a')));

  std::shared_ptr<const char> actual;
  mrb.get(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.pop(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);
}

/*
   Tests push_and_replace with a temporary object, and using
   get and pop methods with unique_ptr signature.
 */
TEST(TestMappedRingBuffer, temporary_l_value_with_unique_get_pop) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  // Pass in value with temporary object
  mrb.push_and_replace(1, std::shared_ptr<const char>(new char('a')));

  std::unique_ptr<char> actual;
  mrb.get(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.pop(1, actual);
  EXPECT_EQ('a', *actual);

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);
}

/*
   Tests normal usage of the mrb.
   Using shared push_and_replace, get and pop methods.
 */
TEST(TestMappedRingBuffer, nominal_push_shared_get_pop_shared) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::shared_ptr<const char> expected(new char('a'));

  EXPECT_FALSE(mrb.push_and_replace(1, expected));
  EXPECT_EQ(2, expected.use_count());

  std::shared_ptr<const char> actual;
  mrb.get(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected, actual);
  EXPECT_EQ(3, actual.use_count());

  mrb.pop(1, actual);
  EXPECT_EQ(expected, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  expected.reset();
  EXPECT_TRUE(actual.unique());

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  expected.reset(new char('a'));
  EXPECT_FALSE(mrb.push_and_replace(1, expected));

  expected.reset(new char('b'));
  EXPECT_FALSE(mrb.push_and_replace(2, expected));

  expected.reset(new char('c'));
  EXPECT_TRUE(mrb.push_and_replace(3, expected));

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.get(2, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }

  mrb.get(3, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('c', *actual);
  }
}

/*
   Tests normal usage of the mrb.
   Using shared push_and_replace, unique get and pop methods.
 */
TEST(TestMappedRingBuffer, nominal_push_shared_get_pop_unique) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::shared_ptr<const char> expected(new char('a'));
  const char * expected_orig = expected.get();

  EXPECT_FALSE(mrb.push_and_replace(1, expected));
  EXPECT_EQ(2, expected.use_count());

  std::unique_ptr<char> actual;
  mrb.get(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_NE(expected_orig, actual.get());
  mrb.pop(1, actual);
  EXPECT_NE(expected_orig, actual.get());
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  EXPECT_FALSE(mrb.push_and_replace(1, expected));
  expected.reset();
  mrb.pop(1, actual);
  EXPECT_NE(expected_orig, actual.get());
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  expected.reset(new char('a'));
  EXPECT_FALSE(mrb.push_and_replace(1, expected));

  expected.reset(new char('b'));
  EXPECT_FALSE(mrb.push_and_replace(2, expected));

  expected.reset(new char('c'));
  EXPECT_TRUE(mrb.push_and_replace(3, expected));

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.get(2, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }

  mrb.get(3, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('c', *actual);
  }
}

/*
   Tests normal usage of the mrb.
   Using unique push_and_replace, get and pop methods.
 */
TEST(TestMappedRingBuffer, nominal_push_unique_get_pop_unique) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::unique_ptr<char> expected(new char('a'));
  const char * expected_orig = expected.get();

  EXPECT_FALSE(mrb.push_and_replace(1, std::move(expected)));

  std::unique_ptr<char> actual;
  mrb.get(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_NE(expected_orig, actual.get());
  mrb.pop(1, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected_orig, actual.get());
  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  expected.reset(new char('a'));
  EXPECT_FALSE(mrb.push_and_replace(1, std::move(expected)));

  expected.reset(new char('b'));
  EXPECT_FALSE(mrb.push_and_replace(2, std::move(expected)));

  expected.reset(new char('c'));
  EXPECT_TRUE(mrb.push_and_replace(3, std::move(expected)));

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.get(2, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }

  mrb.get(3, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('c', *actual);
  }
}

/*
   Tests normal usage of the mrb.
   Using unique push_and_replace, shared get and pop methods.
 */
TEST(TestMappedRingBuffer, nominal_push_unique_get_pop_shared) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);
  std::unique_ptr<char> expected(new char('a'));
  const char * expected_orig = expected.get();

  EXPECT_FALSE(mrb.push_and_replace(1, std::move(expected)));

  std::shared_ptr<const char> actual;
  mrb.get(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected_orig, actual.get());
  mrb.pop(1, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }
  EXPECT_EQ(expected_orig, actual.get());
  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  expected.reset(new char('a'));
  EXPECT_FALSE(mrb.push_and_replace(1, std::move(expected)));

  expected.reset(new char('b'));
  EXPECT_FALSE(mrb.push_and_replace(2, std::move(expected)));

  expected.reset(new char('c'));
  EXPECT_TRUE(mrb.push_and_replace(3, std::move(expected)));

  mrb.get(1, actual);
  EXPECT_EQ(nullptr, actual);

  mrb.get(2, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }

  mrb.get(3, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('c', *actual);
  }
}

/*
   Tests the affect of reusing keys (non-unique keys) in a mrb.
 */
TEST(TestMappedRingBuffer, non_unique_keys) {
  rclcpp::mapped_ring_buffer::MappedRingBuffer<char> mrb(2);

  std::shared_ptr<const char> input(new char('a'));
  mrb.push_and_replace(1, input);
  input.reset(new char('b'));

  // Different value, same key.
  mrb.push_and_replace(1, input);
  input.reset();

  std::unique_ptr<char> actual;
  mrb.pop(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('a', *actual);
  }

  actual = nullptr;
  mrb.pop(1, actual);
  EXPECT_NE(nullptr, actual);
  if (actual) {
    EXPECT_EQ('b', *actual);
  }
}
