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

#ifndef RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/buffers/ring_buffer_implementation.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp
{
namespace experimental
{

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<MessageT>>
typename rclcpp::experimental::buffers::IntraProcessBuffer<MessageT, Deleter>::UniquePtr
create_intra_process_buffer(
  IntraProcessBufferType buffer_type,
  const rclcpp::QoS & qos,
  std::shared_ptr<Alloc> allocator)
{
  size_t buffer_size = qos.depth();

  using rclcpp::experimental::buffers::IntraProcessBuffer;
  using rclcpp::experimental::buffers::IntraProcessBufferNode;
  using rclcpp::experimental::buffers::TypedIntraProcessBuffer;
  using rclcpp::experimental::buffers::RingBufferImplementation;

  using BufferT = IntraProcessBufferNode<MessageT, Deleter>;
  using BufferImplT = RingBufferImplementation<BufferT>;

  auto buffer_implementation = std::make_unique<BufferImplT>(buffer_size);

  typename IntraProcessBuffer<MessageT, Deleter>::UniquePtr buffer;

  switch (buffer_type) {
    case IntraProcessBufferType::SharedPtr:
      {
        using IntraProcessBufferT = TypedIntraProcessBuffer<
          MessageT, Alloc, Deleter, IntraProcessBufferType::SharedPtr>;

        // Construct the intra_process_buffer
        buffer = std::make_unique<IntraProcessBufferT>(
          std::move(buffer_implementation), allocator);

        break;
      }
    case IntraProcessBufferType::UniquePtr:
      {
        using IntraProcessBufferT = TypedIntraProcessBuffer<
          MessageT, Alloc, Deleter, IntraProcessBufferType::UniquePtr>;

        // Construct the intra_process_buffer
        buffer = std::make_unique<IntraProcessBufferT>(
          std::move(buffer_implementation), allocator);

        break;
      }
    case IntraProcessBufferType::CallbackDefault:
      {
        using IntraProcessBufferT = TypedIntraProcessBuffer<
          MessageT, Alloc, Deleter, IntraProcessBufferType::CallbackDefault>;

        // Construct the intra_process_buffer
        buffer = std::make_unique<IntraProcessBufferT>(
          std::move(buffer_implementation), allocator);

        break;
      }
  }

  return buffer;
}

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_
