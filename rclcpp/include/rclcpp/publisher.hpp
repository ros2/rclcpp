// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PUBLISHER_HPP_
#define RCLCPP__PUBLISHER_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/publisher.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// A publisher publishes messages of any type to a topic.
template<typename MessageT, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, Alloc>)

  Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rcl_publisher_options_t & publisher_options,
    const PublisherEventCallbacks & event_callbacks,
    const std::shared_ptr<MessageAlloc> & allocator)
  : PublisherBase(
      node_base,
      topic,
      *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
      publisher_options),
    message_allocator_(allocator)
  {
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_.get());

    if (event_callbacks.deadline_callback) {
      this->add_event_handler(event_callbacks.deadline_callback,
        RCL_PUBLISHER_OFFERED_DEADLINE_MISSED);
    }
    if (event_callbacks.liveliness_callback) {
      this->add_event_handler(event_callbacks.liveliness_callback,
        RCL_PUBLISHER_LIVELINESS_LOST);
    }
  }

  virtual ~Publisher()
  {}

  mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  make_mapped_ring_buffer(size_t size) const override
  {
    return mapped_ring_buffer::MappedRingBuffer<
      MessageT,
      typename Publisher<MessageT, Alloc>::MessageAlloc
    >::make_shared(size, this->get_allocator());
  }

  /// Send a message to the topic for this publisher.
  /**
   * This function is templated on the input message type, MessageT.
   * \param[in] msg A shared pointer to the message to send.
   */
  virtual void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    if (!intra_process_is_enabled_) {
      this->do_inter_process_publish(msg.get());
      return;
    }
    // If an interprocess subscription exist, then the unique_ptr is promoted
    // to a shared_ptr and published.
    // This allows doing the intraprocess publish first and then doing the
    // interprocess publish, resulting in lower publish-to-subscribe latency.
    // It's not possible to do that with an unique_ptr,
    // as do_intra_process_publish takes the ownership of the message.
    uint64_t message_seq;
    bool inter_process_publish_needed =
      get_subscription_count() > get_intra_process_subscription_count();
    MessageSharedPtr shared_msg;
    if (inter_process_publish_needed) {
      shared_msg = std::move(msg);
      message_seq =
        store_intra_process_message(intra_process_publisher_id_, shared_msg);
    } else {
      message_seq =
        store_intra_process_message(intra_process_publisher_id_, std::move(msg));
    }
    this->do_intra_process_publish(message_seq);
    if (inter_process_publish_needed) {
      this->do_inter_process_publish(shared_msg.get());
    }
  }

// Skip deprecated attribute in windows, as it raise a warning in template specialization.
#if !defined(_WIN32)
  [[deprecated(
    "publishing an unique_ptr is prefered when using intra process communication."
    " If using a shared_ptr, use publish(*msg).")]]
#endif
  virtual void
  publish(const std::shared_ptr<const MessageT> & msg)
  {
    publish(*msg);
  }

  virtual void
  publish(const MessageT & msg)
  {
    // Avoid allocating when not using intra process.
    if (!intra_process_is_enabled_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(&msg);
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // As the message is not const, a copy should be made.
    // A shared_ptr<const MessageT> could also be constructed here.
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, msg);
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    this->publish(std::move(unique_msg));
  }

// Skip deprecated attribute in windows, as it raise a warning in template specialization.
#if !defined(_WIN32)
  [[deprecated(
    "Use publish(*msg). Check against nullptr before calling if necessary.")]]
#endif
  virtual void
  publish(const MessageT * msg)
  {
    if (!msg) {
      throw std::runtime_error("msg argument is nullptr");
    }
    return this->publish(*msg);
  }

  void
  publish(const rcl_serialized_message_t & serialized_msg)
  {
    return this->do_serialized_publish(&serialized_msg);
  }

// Skip deprecated attribute in windows, as it raise a warning in template specialization.
#if !defined(_WIN32)
  [[deprecated(
    "Use publish(*serialized_msg). Check against nullptr before calling if necessary.")]]
#endif
  void
  publish(const rcl_serialized_message_t * serialized_msg)
  {
    return this->do_serialized_publish(serialized_msg);
  }

// Skip deprecated attribute in windows, as it raise a warning in template specialization.
#if !defined(_WIN32)
  [[deprecated(
    "Use publish(*serialized_msg). Check against nullptr before calling if necessary.")]]
#endif
  void
  publish(std::shared_ptr<const rcl_serialized_message_t> serialized_msg)
  {
    return this->do_serialized_publish(serialized_msg.get());
  }

  std::shared_ptr<MessageAlloc> get_allocator() const
  {
    return message_allocator_;
  }

protected:
  void
  do_inter_process_publish(const MessageT * msg)
  {
    auto status = rcl_publish(&publisher_handle_, msg, nullptr);
    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // next call will reset error message if not context
      if (rcl_publisher_is_valid_except_context(&publisher_handle_)) {
        rcl_context_t * context = rcl_publisher_get_context(&publisher_handle_);
        if (nullptr != context && !rcl_context_is_valid(context)) {
          // publisher is invalid due to context being shutdown
          return;
        }
      }
    }
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish message");
    }
  }

  void
  do_serialized_publish(const rcl_serialized_message_t * serialized_msg)
  {
    if (intra_process_is_enabled_) {
      // TODO(Karsten1987): support serialized message passed by intraprocess
      throw std::runtime_error("storing serialized messages in intra process is not supported yet");
    }
    auto status = rcl_publish_serialized_message(&publisher_handle_, serialized_msg, nullptr);
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
    }
  }

  void
  do_intra_process_publish(uint64_t message_seq)
  {
    rcl_interfaces::msg::IntraProcessMessage ipm;
    ipm.publisher_id = intra_process_publisher_id_;
    ipm.message_sequence = message_seq;
    auto status = rcl_publish(&intra_process_publisher_handle_, &ipm, nullptr);
    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // next call will reset error message if not context
      if (rcl_publisher_is_valid_except_context(&intra_process_publisher_handle_)) {
        rcl_context_t * context = rcl_publisher_get_context(&intra_process_publisher_handle_);
        if (nullptr != context && !rcl_context_is_valid(context)) {
          // publisher is invalid due to context being shutdown
          return;
        }
      }
    }
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish intra process message");
    }
  }

  uint64_t
  store_intra_process_message(
    uint64_t publisher_id,
    std::shared_ptr<const MessageT> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publisher msg which is a null pointer");
    }
    uint64_t message_seq =
      ipm->template store_intra_process_message<MessageT, Alloc>(publisher_id, msg);
    return message_seq;
  }

  uint64_t
  store_intra_process_message(
    uint64_t publisher_id,
    std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publisher msg which is a null pointer");
    }
    uint64_t message_seq =
      ipm->template store_intra_process_message<MessageT, Alloc>(publisher_id, std::move(msg));
    return message_seq;
  }

  std::shared_ptr<MessageAlloc> message_allocator_;

  MessageDeleter message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_
