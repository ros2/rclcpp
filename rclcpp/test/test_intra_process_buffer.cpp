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

#include "rclcpp/rclcpp.hpp"

/*
   Construtctor
 */
TEST(TestIntraProcessBuffer, constructor) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedMessageT = std::shared_ptr<const MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using SharedIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, SharedMessageT>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, UniqueMessageT>;

  auto shared_buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<SharedMessageT>>(2);

  SharedIntraProcessBufferT shared_intra_process_buffer(std::move(shared_buffer_impl));

  EXPECT_EQ(true, shared_intra_process_buffer.use_take_shared_method());

  auto unique_buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<UniqueMessageT>>(2);

  UniqueIntraProcessBufferT unique_intra_process_buffer(std::move(unique_buffer_impl));

  EXPECT_EQ(false, unique_intra_process_buffer.use_take_shared_method());
}

/*
  Add data to an intra-process buffer with an implementations that stores shared_ptr
  Messages are extracted using the same data as the implementation, i.e. shared_ptr
  - Add shared_ptr no copies are expected
  - Add unique_ptr no copies are expected
 */
TEST(TestIntraProcessBuffer, shared_buffer_add) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedMessageT = std::shared_ptr<const MessageT>;
  using SharedIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, SharedMessageT>;

  auto buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<SharedMessageT>>(2);

  SharedIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());

  intra_process_buffer.add_shared(original_shared_msg);

  EXPECT_EQ(2L, original_shared_msg.use_count());

  SharedMessageT popped_shared_msg;
  popped_shared_msg = intra_process_buffer.consume_shared();
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(original_shared_msg.use_count(), popped_shared_msg.use_count());
  EXPECT_EQ(*original_shared_msg, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);

  auto original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;

  intra_process_buffer.add_unique(std::move(original_unique_msg));

  popped_shared_msg = intra_process_buffer.consume_shared();
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(1L, popped_shared_msg.use_count());
  EXPECT_EQ(original_value, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
}

/*
  Add data to an intra-process buffer with an implementations that stores unique_ptr
  Messages are extracted using the same data as the implementation, i.e. unique_ptr
  - Add shared_ptr a copy is expected
  - Add unique_ptr no copies are expected
 */
TEST(TestIntraProcessBuffer, unique_buffer_add) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, UniqueMessageT>;

  auto buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<UniqueMessageT>>(2);

  UniqueIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());

  intra_process_buffer.add_shared(original_shared_msg);

  EXPECT_EQ(1L, original_shared_msg.use_count());

  UniqueMessageT popped_unique_msg;
  popped_unique_msg = intra_process_buffer.consume_unique();
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(*original_shared_msg, *popped_unique_msg);
  EXPECT_NE(original_message_pointer, popped_message_pointer);

  auto original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;

  intra_process_buffer.add_unique(std::move(original_unique_msg));

  popped_unique_msg = intra_process_buffer.consume_unique();
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(original_value, *popped_unique_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
}

/*
  Consume data from an intra-process buffer with an implementations that stores shared_ptr
  Messages are inserted using the same data as the implementation, i.e. shared_ptr
  - Request shared_ptr no copies are expected
  - Request unique_ptr a copy is expected
 */
TEST(TestIntraProcessBuffer, shared_buffer_consume) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedMessageT = std::shared_ptr<const MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using SharedIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, SharedMessageT>;

  auto buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<SharedMessageT>>(2);

  SharedIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());

  intra_process_buffer.add_shared(original_shared_msg);

  EXPECT_EQ(2L, original_shared_msg.use_count());

  SharedMessageT popped_shared_msg;
  popped_shared_msg = intra_process_buffer.consume_shared();
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(original_shared_msg.use_count(), popped_shared_msg.use_count());
  EXPECT_EQ(*original_shared_msg, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);

  original_shared_msg = std::make_shared<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());

  intra_process_buffer.add_shared(original_shared_msg);

  UniqueMessageT popped_unique_msg;
  popped_unique_msg = intra_process_buffer.consume_unique();
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(1L, original_shared_msg.use_count());
  EXPECT_EQ(*original_shared_msg, *popped_unique_msg);
  EXPECT_NE(original_message_pointer, popped_message_pointer);
}

/*
  Consume data from an intra-process buffer with an implementations that stores unique_ptr
  Messages are inserted using the same data as the implementation, i.e. unique_ptr
  - Request shared_ptr no copies are expected
  - Request unique_ptr no copies are expected
 */
TEST(TestIntraProcessBuffer, unique_buffer_consume) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedMessageT = std::shared_ptr<const MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, UniqueMessageT>;

  auto buffer_impl =
    std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<UniqueMessageT>>(2);

  UniqueIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_unique_msg = std::make_unique<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;

  intra_process_buffer.add_unique(std::move(original_unique_msg));

  SharedMessageT popped_shared_msg;
  popped_shared_msg = intra_process_buffer.consume_shared();
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(original_value, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);

  original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  original_value = *original_unique_msg;

  intra_process_buffer.add_unique(std::move(original_unique_msg));

  UniqueMessageT popped_unique_msg;
  popped_unique_msg = intra_process_buffer.consume_unique();
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(original_value, *popped_unique_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
}
