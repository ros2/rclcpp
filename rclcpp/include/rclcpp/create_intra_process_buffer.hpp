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

#ifndef RCLCPP__CREATE_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__CREATE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <type_traits>
#include <utility>

#include "rcl/subscription.h"

#include "rclcpp/buffers/ring_buffer_implementation.hpp"
#include "rclcpp/buffers/intra_process_buffer.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"

namespace rclcpp
{

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<MessageT>>
typename intra_process_buffer::IntraProcessBuffer<MessageT, Alloc, Deleter>::UniquePtr
create_intra_process_buffer(
  IntraProcessBufferType buffer_type,
  rmw_qos_profile_t qos,
  std::shared_ptr<Alloc> allocator)
{
  using MessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;

  size_t buffer_size = qos.depth;

  typename intra_process_buffer::IntraProcessBuffer<MessageT, Alloc, Deleter>::UniquePtr buffer;

  switch (buffer_type) {
    case IntraProcessBufferType::SharedPtr:
      {
        using BufferT = MessageSharedPtr;

        auto buffer_implementation =
          std::make_unique<rclcpp::intra_process_buffer::RingBufferImplementation<BufferT>>(
          buffer_size);

        // Construct the intra_process_buffer
        buffer =
          std::make_unique<rclcpp::intra_process_buffer::TypedIntraProcessBuffer<MessageT, Alloc,
            Deleter, BufferT>>(
          std::move(buffer_implementation),
          allocator);

        break;
      }
    case IntraProcessBufferType::UniquePtr:
      {
        using BufferT = MessageUniquePtr;

        auto buffer_implementation =
          std::make_unique<rclcpp::intra_process_buffer::RingBufferImplementation<BufferT>>(
          buffer_size);

        // Construct the intra_process_buffer
        buffer =
          std::make_unique<rclcpp::intra_process_buffer::TypedIntraProcessBuffer<MessageT, Alloc,
            Deleter, BufferT>>(
          std::move(buffer_implementation),
          allocator);

        break;
      }
    default:
      {
        throw std::runtime_error("Unrecognized IntraProcessBufferType value");
        break;
      }
  }

  return buffer;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_INTRA_PROCESS_BUFFER_HPP_
