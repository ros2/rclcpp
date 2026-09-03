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

#include "rclcpp/experimental/buffers/intra_process_buffer_data.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/rclcpp.hpp"

template<size_t idx>
static constexpr rmw_message_info_t generate_message_info()
{
  rmw_message_info_t message_info{};
  message_info.source_timestamp = static_cast<rmw_time_point_value_t>(10 * idx + 1);
  message_info.received_timestamp = static_cast<rmw_time_point_value_t>(10 * idx + 2);
  message_info.publication_sequence_number = static_cast<uint64_t>(10 * idx + 3);
  message_info.reception_sequence_number = static_cast<uint64_t>(10 * idx + 4);
  message_info.publisher_gid = {"test", {idx, idx, idx, idx, idx, idx, idx,
      idx, idx, idx, idx, idx, idx, idx, idx, idx}};
  message_info.from_intra_process = true;
  return message_info;
}

/*
   Constructor
 */
TEST(TestIntraProcessBuffer, constructor) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::SharedPtr>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::UniquePtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  auto shared_buffer_impl = std::make_unique<BufferImplT>(2);

  SharedIntraProcessBufferT shared_intra_process_buffer(std::move(shared_buffer_impl));

  EXPECT_EQ(rclcpp::IntraProcessBufferType::SharedPtr, shared_intra_process_buffer.buffer_type());

  auto unique_buffer_impl = std::make_unique<BufferImplT>(2);

  UniqueIntraProcessBufferT unique_intra_process_buffer(std::move(unique_buffer_impl));

  EXPECT_EQ(rclcpp::IntraProcessBufferType::UniquePtr, unique_intra_process_buffer.buffer_type());
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
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::SharedPtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  auto buffer_impl = std::make_unique<BufferImplT>(2);

  SharedIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());
  rmw_message_info_t original_message_info = generate_message_info<0>();

  intra_process_buffer.add({original_shared_msg, original_message_info});

  EXPECT_EQ(2L, original_shared_msg.use_count());

  SharedMessageT popped_shared_msg;
  rmw_message_info_t popped_message_info;
  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<SharedMessageT>(popped_data.message));
    popped_shared_msg = std::get<SharedMessageT>(popped_data.message);
    popped_message_info = popped_data.message_info;
  }
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(original_shared_msg.use_count(), popped_shared_msg.use_count());
  EXPECT_EQ(*original_shared_msg, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);

  auto original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;
  original_message_info = generate_message_info<1>();

  intra_process_buffer.add({std::move(original_unique_msg), original_message_info});

  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<SharedMessageT>(popped_data.message));
    popped_shared_msg = std::get<SharedMessageT>(popped_data.message);
    popped_message_info = popped_data.message_info;
  }
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_shared_msg.get());

  EXPECT_EQ(1L, popped_shared_msg.use_count());
  EXPECT_EQ(original_value, *popped_shared_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);
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
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::UniquePtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  auto buffer_impl = std::make_unique<BufferImplT>(2);

  UniqueIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());
  rmw_message_info_t original_message_info = generate_message_info<0>();

  intra_process_buffer.add({original_shared_msg, original_message_info});

  EXPECT_EQ(1L, original_shared_msg.use_count());

  UniqueMessageT popped_unique_msg;
  rmw_message_info_t popped_message_info;
  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(popped_data.message));
    popped_unique_msg = std::move(std::get<UniqueMessageT>(popped_data.message));
    popped_message_info = popped_data.message_info;
  }
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(*original_shared_msg, *popped_unique_msg);
  EXPECT_NE(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);

  auto original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;
  original_message_info = generate_message_info<1>();

  intra_process_buffer.add({std::move(original_unique_msg), original_message_info});

  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(popped_data.message));
    popped_unique_msg = std::move(std::get<UniqueMessageT>(popped_data.message));
    popped_message_info = popped_data.message_info;
  }
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(original_value, *popped_unique_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);
}

/*
  Get all data from an intra-process buffer with an implementations that stores shared_ptr
  Messages are inserted as shared_ptr and then as unique_ptr
  - Add shared_ptr no copies are expected
  - Add unique_ptr no copies are expected
 */
TEST(TestIntraProcessBuffer, shared_buffer_get_all_data) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using SharedMessageT = std::shared_ptr<const MessageT>;
  using SharedIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::SharedPtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  auto buffer_impl = std::make_unique<BufferImplT>(2);

  SharedIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());
  auto original_value = *original_shared_msg;
  rmw_message_info_t original_message_info = generate_message_info<0>();
  intra_process_buffer.add({original_shared_msg, original_message_info});

  auto original_unique_msg_2 = std::make_unique<char>('b');
  auto original_message_pointer_2 = reinterpret_cast<std::uintptr_t>(original_unique_msg_2.get());
  auto original_value_2 = *original_unique_msg_2;
  rmw_message_info_t original_message_info_2 = generate_message_info<1>();
  intra_process_buffer.add({std::move(original_unique_msg_2), original_message_info_2});

  auto all_data = intra_process_buffer.get_all_data();
  ASSERT_EQ(2u, all_data.size());

  ASSERT_TRUE(std::holds_alternative<SharedMessageT>(all_data[0].message));
  {
    const auto & shared_msg = std::get<SharedMessageT>(all_data[0].message);
    // 3 owners: original_shared_msg, the buffer's own stored copy, and this
    // get_all_data() snapshot's copy.
    EXPECT_EQ(3L, original_shared_msg.use_count());
    EXPECT_EQ(original_shared_msg.use_count(), shared_msg.use_count());
    EXPECT_EQ(original_value, *shared_msg);
    EXPECT_EQ(original_message_pointer, reinterpret_cast<std::uintptr_t>(shared_msg.get()));
    EXPECT_EQ(
      std::memcmp(&original_message_info, &all_data[0].message_info, sizeof(rmw_message_info_t)),
      0);
  }

  ASSERT_TRUE(std::holds_alternative<SharedMessageT>(all_data[1].message));
  {
    const auto & shared_msg = std::get<SharedMessageT>(all_data[1].message);
    EXPECT_EQ(original_value_2, *shared_msg);
    EXPECT_EQ(original_message_pointer_2, reinterpret_cast<std::uintptr_t>(shared_msg.get()));
    EXPECT_EQ(
      std::memcmp(
        &original_message_info_2, &all_data[1].message_info, sizeof(rmw_message_info_t)),
      0);
  }
}

/*
  Get all data from an intra-process buffer with an implementations that stores unique_ptr
  Messages are inserted as shared_ptr and then as unique_ptr
  - Add shared_ptr a copy is expected
  - Add unique_ptr no copies are expected
 */
TEST(TestIntraProcessBuffer, unique_buffer_get_all_data) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::UniquePtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  auto buffer_impl = std::make_unique<BufferImplT>(2);

  UniqueIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  auto original_shared_msg = std::make_shared<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_shared_msg.get());
  auto original_value = *original_shared_msg;
  rmw_message_info_t original_message_info = generate_message_info<0>();
  intra_process_buffer.add({original_shared_msg, original_message_info});

  auto original_unique_msg_2 = std::make_unique<char>('b');
  auto original_message_pointer_2 = reinterpret_cast<std::uintptr_t>(original_unique_msg_2.get());
  auto original_value_2 = *original_unique_msg_2;
  rmw_message_info_t original_message_info_2 = generate_message_info<1>();
  intra_process_buffer.add({std::move(original_unique_msg_2), original_message_info_2});

  auto all_data = intra_process_buffer.get_all_data();
  ASSERT_EQ(2u, all_data.size());

  ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(all_data[0].message));
  {
    const auto & unique_msg = std::get<UniqueMessageT>(all_data[0].message);
    EXPECT_EQ(1L, original_shared_msg.use_count());
    EXPECT_EQ(original_value, *unique_msg);
    EXPECT_NE(original_message_pointer, reinterpret_cast<std::uintptr_t>(unique_msg.get()));
    EXPECT_EQ(
      std::memcmp(&original_message_info, &all_data[0].message_info, sizeof(rmw_message_info_t)),
      0);
  }

  ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(all_data[1].message));
  {
    const auto & unique_msg = std::get<UniqueMessageT>(all_data[1].message);
    EXPECT_EQ(original_value_2, *unique_msg);
    // get_all_data() always returns an independent copy of the buffer's own storage.
    EXPECT_NE(original_message_pointer_2, reinterpret_cast<std::uintptr_t>(unique_msg.get()));
    EXPECT_EQ(
      std::memcmp(
        &original_message_info_2, &all_data[1].message_info, sizeof(rmw_message_info_t)),
      0);
  }
}

/*
  Check the available buffer capacity while storing and consuming data from an intra-process
  buffer.
  The initial available buffer capacity should equal the buffer size.
  Inserting a message should decrease the available buffer capacity by 1.
  Consuming a message should increase the available buffer capacity by 1.
 */
TEST(TestIntraProcessBuffer, available_capacity) {
  using MessageT = char;
  using Alloc = std::allocator<void>;
  using Deleter = std::default_delete<MessageT>;
  using UniqueMessageT = std::unique_ptr<MessageT, Deleter>;
  using UniqueIntraProcessBufferT = rclcpp::experimental::buffers::TypedIntraProcessBuffer<
    MessageT, Alloc, Deleter, rclcpp::IntraProcessBufferType::UniquePtr>;
  using BufferT = rclcpp::experimental::buffers::IntraProcessBufferData<MessageT, Deleter>;
  using BufferImplT = rclcpp::experimental::buffers::RingBufferImplementation<BufferT>;

  constexpr auto history_depth = 5u;

  auto buffer_impl = std::make_unique<BufferImplT>(history_depth);

  UniqueIntraProcessBufferT intra_process_buffer(std::move(buffer_impl));

  EXPECT_EQ(history_depth, intra_process_buffer.available_capacity());

  auto original_unique_msg = std::make_unique<char>('a');
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  auto original_value = *original_unique_msg;
  rmw_message_info_t original_message_info = generate_message_info<0>();

  intra_process_buffer.add({std::move(original_unique_msg), original_message_info});

  EXPECT_EQ(history_depth - 1u, intra_process_buffer.available_capacity());

  UniqueMessageT popped_unique_msg;
  rmw_message_info_t popped_message_info;
  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(popped_data.message));
    popped_unique_msg = std::move(std::get<UniqueMessageT>(popped_data.message));
    popped_message_info = popped_data.message_info;
  }
  auto popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(history_depth, intra_process_buffer.available_capacity());
  EXPECT_EQ(original_value, *popped_unique_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);

  original_unique_msg = std::make_unique<char>('b');
  original_message_pointer = reinterpret_cast<std::uintptr_t>(original_unique_msg.get());
  original_value = *original_unique_msg;
  original_message_info = generate_message_info<1>();

  intra_process_buffer.add({std::move(original_unique_msg), original_message_info});

  auto second_unique_msg = std::make_unique<char>('c');
  auto second_message_pointer = reinterpret_cast<std::uintptr_t>(second_unique_msg.get());
  auto second_value = *second_unique_msg;
  rmw_message_info_t second_message_info = generate_message_info<2>();

  intra_process_buffer.add({std::move(second_unique_msg), second_message_info});

  EXPECT_EQ(history_depth - 2u, intra_process_buffer.available_capacity());

  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(popped_data.message));
    popped_unique_msg = std::move(std::get<UniqueMessageT>(popped_data.message));
    popped_message_info = popped_data.message_info;
  }
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(history_depth - 1u, intra_process_buffer.available_capacity());
  EXPECT_EQ(original_value, *popped_unique_msg);
  EXPECT_EQ(original_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&original_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);

  {
    auto popped_data = intra_process_buffer.consume();
    ASSERT_TRUE(std::holds_alternative<UniqueMessageT>(popped_data.message));
    popped_unique_msg = std::move(std::get<UniqueMessageT>(popped_data.message));
    popped_message_info = popped_data.message_info;
  }
  popped_message_pointer = reinterpret_cast<std::uintptr_t>(popped_unique_msg.get());

  EXPECT_EQ(history_depth, intra_process_buffer.available_capacity());
  EXPECT_EQ(second_value, *popped_unique_msg);
  EXPECT_EQ(second_message_pointer, popped_message_pointer);
  EXPECT_EQ(std::memcmp(&second_message_info, &popped_message_info, sizeof(rmw_message_info_t)),
    0);
}
