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
#include <type_traits>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/macros.hpp"

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
  virtual bool use_take_shared_method() const = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename MessageDeleter = std::default_delete<MessageT>>
class IntraProcessBuffer : public IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBuffer)

  virtual ~IntraProcessBuffer() {}

  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  virtual void add_shared(MessageSharedPtr msg) = 0;
  virtual void add_unique(MessageUniquePtr msg) = 0;

  virtual MessageSharedPtr consume_shared() = 0;
  virtual MessageUniquePtr consume_unique() = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename MessageDeleter = std::default_delete<MessageT>,
  typename BufferT = std::unique_ptr<MessageT>>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT, Alloc, MessageDeleter>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  explicit
  TypedIntraProcessBuffer(
    std::unique_ptr<BufferImplementationBase<BufferT>> buffer_impl,
    std::shared_ptr<Alloc> allocator = nullptr)
  {
    bool valid_type = (std::is_same<BufferT, MessageSharedPtr>::value ||
      std::is_same<BufferT, MessageUniquePtr>::value);
    if (!valid_type) {
      throw std::runtime_error("Creating TypedIntraProcessBuffer with not valid BufferT");
    }

    buffer_ = std::move(buffer_impl);

    if (!allocator) {
      message_allocator_ = std::make_shared<MessageAlloc>();
    } else {
      message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    }
  }

  virtual ~TypedIntraProcessBuffer() {}

  void add_shared(MessageSharedPtr msg) override
  {
    add_shared_impl<BufferT>(std::move(msg));
  }

  void add_unique(MessageUniquePtr msg) override
  {
    buffer_->enqueue(std::move(msg));
  }

  MessageSharedPtr consume_shared() override
  {
    return consume_shared_impl<BufferT>();
  }

  MessageUniquePtr consume_unique() override
  {
    return consume_unique_impl<BufferT>();
  }

  bool has_data() const override
  {
    return buffer_->has_data();
  }

  void clear() override
  {
    buffer_->clear();
  }

  bool use_take_shared_method() const override
  {
    return std::is_same<BufferT, MessageSharedPtr>::value;
  }

private:
  std::unique_ptr<BufferImplementationBase<BufferT>> buffer_;

  std::shared_ptr<MessageAlloc> message_allocator_;

  // MessageSharedPtr to MessageSharedPtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageSharedPtr>::value
  >::type
  add_shared_impl(MessageSharedPtr shared_msg)
  {
    buffer_->enqueue(std::move(shared_msg));
  }

  // MessageSharedPtr to MessageUniquePtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageUniquePtr>::value
  >::type
  add_shared_impl(MessageSharedPtr shared_msg)
  {
    // This should not happen: here a copy is unconditionally made, while the intra-process manager
    // can decide whether a copy is needed depending on the number and the type of buffers

    MessageUniquePtr unique_msg;
    MessageDeleter * deleter = std::get_deleter<MessageDeleter, const MessageT>(shared_msg);
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *shared_msg);
    if (deleter) {
      unique_msg = MessageUniquePtr(ptr, *deleter);
    } else {
      unique_msg = MessageUniquePtr(ptr);
    }

    buffer_->enqueue(std::move(unique_msg));
  }

  // MessageSharedPtr to MessageSharedPtr
  template<typename OriginT>
  typename std::enable_if<
    std::is_same<OriginT, MessageSharedPtr>::value,
    MessageSharedPtr
  >::type
  consume_shared_impl()
  {
    return buffer_->dequeue();
  }

  // MessageUniquePtr to MessageSharedPtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageUniquePtr>::value),
    MessageSharedPtr
  >::type
  consume_shared_impl()
  {
    // automatic cast from unique ptr to shared ptr
    return buffer_->dequeue();
  }

  // MessageSharedPtr to MessageUniquePtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageSharedPtr>::value),
    MessageUniquePtr
  >::type
  consume_unique_impl()
  {
    MessageSharedPtr buffer_msg = buffer_->dequeue();

    MessageUniquePtr unique_msg;
    MessageDeleter * deleter = std::get_deleter<MessageDeleter, const MessageT>(buffer_msg);
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *buffer_msg);
    if (deleter) {
      unique_msg = MessageUniquePtr(ptr, *deleter);
    } else {
      unique_msg = MessageUniquePtr(ptr);
    }

    return unique_msg;
  }

  // MessageUniquePtr to MessageUniquePtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageUniquePtr>::value),
    MessageUniquePtr
  >::type
  consume_unique_impl()
  {
    return buffer_->dequeue();
  }
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
