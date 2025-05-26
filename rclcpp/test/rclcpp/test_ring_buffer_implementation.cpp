// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/experimental/buffers/ring_buffer_implementation.hpp"

/*
 * Construtctor
 */
TEST(TestRingBufferImplementation, constructor) {
  // Cannot create a buffer of size zero.
  EXPECT_THROW(
    rclcpp::experimental::buffers::RingBufferImplementation<char> rb(0),
    std::invalid_argument);

  rclcpp::experimental::buffers::RingBufferImplementation<char> rb(1);

  EXPECT_EQ(false, rb.has_data());
  EXPECT_EQ(false, rb.is_full());
}

/*
 * Basic usage
 * - insert data and check that it has data
 * - get all data
 * - extract data
 * - overwrite old data writing over the buffer capacity
 */
TEST(TestRingBufferImplementation, basic_usage) {
  rclcpp::experimental::buffers::RingBufferImplementation<char> rb(2);

  rb.enqueue('a');

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(false, rb.is_full());

  char v = rb.dequeue();

  EXPECT_EQ('a', v);
  EXPECT_EQ(false, rb.has_data());
  EXPECT_EQ(false, rb.is_full());

  rb.enqueue('b');
  rb.enqueue('c');

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(true, rb.is_full());

  rb.enqueue('d');

  const auto all_data_vec = rb.get_all_data();

  EXPECT_EQ(2u, all_data_vec.size());
  EXPECT_EQ('c', all_data_vec[0]);
  EXPECT_EQ('d', all_data_vec[1]);

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(true, rb.is_full());

  v = rb.dequeue();

  EXPECT_EQ('c', v);
  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(false, rb.is_full());

  v = rb.dequeue();

  EXPECT_EQ('d', v);
  EXPECT_EQ(false, rb.has_data());
  EXPECT_EQ(false, rb.is_full());
}

/*
 * Basic usage with unique_ptr
 * - insert unique_ptr data and check that it has data
 * - get all data
 * - extract data
 * - overwrite old data writing over the buffer capacity
 */
TEST(TestRingBufferImplementation, basic_usage_unique_ptr) {
  rclcpp::experimental::buffers::RingBufferImplementation<std::unique_ptr<char>> rb(2);

  auto a = std::make_unique<char>('a');
  auto b = std::make_unique<char>('b');
  auto original_b_pointer = reinterpret_cast<std::uintptr_t>(b.get());
  auto c = std::make_unique<char>('c');
  auto original_c_pointer = reinterpret_cast<std::uintptr_t>(c.get());

  rb.enqueue(std::move(a));

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(false, rb.is_full());

  rb.enqueue(std::move(b));
  rb.enqueue(std::move(c));

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(true, rb.is_full());

  const auto all_data_vec = rb.get_all_data();

  EXPECT_EQ(2u, all_data_vec.size());
  EXPECT_EQ('b', *all_data_vec[0]);
  EXPECT_EQ('c', *all_data_vec[1]);
  EXPECT_NE(original_b_pointer, reinterpret_cast<std::uintptr_t>(all_data_vec[0].get()));
  EXPECT_NE(original_c_pointer, reinterpret_cast<std::uintptr_t>(all_data_vec[1].get()));

  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(true, rb.is_full());

  auto uni_ptr = rb.dequeue();

  EXPECT_EQ('b', *uni_ptr);
  EXPECT_EQ(original_b_pointer, reinterpret_cast<std::uintptr_t>(uni_ptr.get()));
  EXPECT_EQ(true, rb.has_data());
  EXPECT_EQ(false, rb.is_full());

  uni_ptr = rb.dequeue();

  EXPECT_EQ('c', *uni_ptr);
  EXPECT_EQ(original_c_pointer, reinterpret_cast<std::uintptr_t>(uni_ptr.get()));
  EXPECT_EQ(false, rb.has_data());
  EXPECT_EQ(false, rb.is_full());
}

TEST(TestRingBufferImplementation, handle_nullptr_deletion) {
  rclcpp::experimental::buffers::RingBufferImplementation<std::unique_ptr<int>> rb(3);
  rb.enqueue(std::make_unique<int>(42));
  rb.enqueue(nullptr);  // intentionally enqueuing nullptr
  rb.enqueue(std::make_unique<int>(84));
  auto all_data = rb.get_all_data();
  EXPECT_EQ(3u, all_data.size());
  EXPECT_EQ(42, *(all_data[0]));
  EXPECT_EQ(nullptr, all_data[1]);
  EXPECT_EQ(84, *(all_data[2]));
}
