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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer_data.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/macros.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

class IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBufferBase)

  virtual ~IntraProcessBufferBase() {}

  virtual void clear() = 0;

  virtual bool has_data() const = 0;
  virtual IntraProcessBufferType buffer_type() const = 0;
  virtual size_t available_capacity() const = 0;
};

template<
  typename MessageT,
  typename MessageDeleter = std::default_delete<MessageT>>
class IntraProcessBuffer : public IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBuffer)

  virtual ~IntraProcessBuffer() {}

  using Data = IntraProcessBufferData<MessageT, MessageDeleter>;

  virtual void add(Data data) = 0;

  virtual Data consume() = 0;

  virtual std::vector<Data> get_all_data() = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename MessageDeleter = std::default_delete<MessageT>,
  IntraProcessBufferType BufferType = IntraProcessBufferType::CallbackDefault>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT, MessageDeleter>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  using Buffer = IntraProcessBuffer<MessageT, MessageDeleter>;
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using Data = typename Buffer::Data;
  using MessageSharedPtr = typename Data::MessageSharedPtr;
  using MessageUniquePtr = typename Data::MessageUniquePtr;

  explicit
  TypedIntraProcessBuffer(
    std::unique_ptr<BufferImplementationBase<Data>> buffer_impl,
    std::shared_ptr<Alloc> allocator = nullptr)
  : buffer_(std::move(buffer_impl))
  {
    TRACETOOLS_TRACEPOINT(
      rclcpp_buffer_to_ipb,
      static_cast<const void *>(buffer_.get()),
      static_cast<const void *>(this));
    if (!allocator) {
      message_allocator_ = std::make_shared<MessageAlloc>();
    } else {
      message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    }
  }

  virtual ~TypedIntraProcessBuffer() {}

  void add(Data data) override
  {
    add_impl<BufferType>(std::move(data));
  }

  Data consume() override
  {
    return buffer_->dequeue();
  }

  std::vector<Data> get_all_data() override
  {
    return buffer_->get_all_data();
  }

  bool has_data() const override
  {
    return buffer_->has_data();
  }

  void clear() override
  {
    buffer_->clear();
  }

  IntraProcessBufferType buffer_type() const override
  {
    return BufferType;
  }

  size_t available_capacity() const override
  {
    return buffer_->available_capacity();
  }

private:
  std::unique_ptr<BufferImplementationBase<Data>> buffer_;

  std::shared_ptr<MessageAlloc> message_allocator_;

  template<IntraProcessBufferType BufferT>
  typename std::enable_if_t<
    BufferT == IntraProcessBufferType::CallbackDefault>
  add_impl(Data data)
  {
    buffer_->enqueue(std::move(data));
  }

  template<IntraProcessBufferType BufferT>
  typename std::enable_if_t<
    BufferT == IntraProcessBufferType::SharedPtr>
  add_impl(Data data)
  {
    if (std::holds_alternative<MessageSharedPtr>(data.message)) {
      buffer_->enqueue(std::move(data));
    } else {
      // Promote to a shared pointer
      auto unique_msg = std::move(std::get<MessageUniquePtr>(data.message));
      data.message = MessageSharedPtr(unique_msg.release());
      buffer_->enqueue(std::move(data));
    }
  }

  template<IntraProcessBufferType BufferT>
  typename std::enable_if_t<
    BufferT == IntraProcessBufferType::UniquePtr>
  add_impl(Data data)
  {
    if (std::holds_alternative<MessageUniquePtr>(data.message)) {
      buffer_->enqueue(std::move(data));
    } else {
      auto shared_msg = std::move(std::get<MessageSharedPtr>(data.message));
      MessageDeleter * deleter = std::get_deleter<MessageDeleter, const MessageT>(shared_msg);
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr, *shared_msg);
      if (deleter) {
        data.message = MessageUniquePtr(ptr, *deleter);
      } else {
        data.message = MessageUniquePtr(ptr);
      }
      buffer_->enqueue(std::move(data));
    }
  }
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
