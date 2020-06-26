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
   Construtctor
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
   Basic usage
   - insert data and check that it has data
   - extract data
   - overwrite old data writing over the buffer capacity
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
