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

#include "rcl/error_handling.h"
#include "rcl/publisher.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

// Forward declaration is used for friend statement.
namespace node_interfaces
{
class NodeTopicsInterface;
}

class PublisherBase
{
  friend ::rclcpp::node_interfaces::NodeTopicsInterface;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)

  /// Default constructor.
  /**
   * Typically, a publisher is not created through this method, but instead is created through a
   * call to `Node::create_publisher`.
   * \param[in] node_base A pointer to the NodeBaseInterface for the parent node.
   * \param[in] topic The topic that this publisher publishes on.
   * \param[in] type_support The type support structure for the type to be published.
   * \param[in] publisher_options QoS settings for this publisher.
   */
  RCLCPP_PUBLIC
  PublisherBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rosidl_message_type_support_t & type_support,
    const rcl_publisher_options_t & publisher_options);

  RCLCPP_PUBLIC
  virtual ~PublisherBase();

  /// Get the topic that this publisher publishes on.
  /** \return The topic name. */
  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  /// Get the queue size for this publisher.
  /** \return The queue size. */
  RCLCPP_PUBLIC
  size_t
  get_queue_size() const;

  /// Get the global identifier for this publisher (used in rmw and by DDS).
  /** \return The gid. */
  RCLCPP_PUBLIC
  const rmw_gid_t &
  get_gid() const;

  /// Get the global identifier for this publisher used by intra-process communication.
  /** \return The intra-process gid. */
  RCLCPP_PUBLIC
  const rmw_gid_t &
  get_intra_process_gid() const;

  /// Get the rcl publisher handle.
  /** \return The rcl publisher handle. */
  RCLCPP_PUBLIC
  rcl_publisher_t *
  get_publisher_handle();

  /// Get the rcl publisher handle.
  /** \return The rcl publisher handle. */
  RCLCPP_PUBLIC
  const rcl_publisher_t *
  get_publisher_handle() const;

  /// Compare this publisher to a gid.
  /**
   * Note that this function calls the next function.
   * \param[in] gid Reference to a gid.
   * \return True if the publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool
  operator==(const rmw_gid_t & gid) const;

  /// Compare this publisher to a pointer gid.
  /**
   * A wrapper for comparing this publisher's gid to the input using rmw_compare_gids_equal.
   * \param[in] gid A pointer to a gid.
   * \return True if this publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool
  operator==(const rmw_gid_t * gid) const;

  using StoreMessageCallbackT = std::function<uint64_t(uint64_t, void *, const std::type_info &)>;

  /// Implementation utility function used to setup intra process publishing after creation.
  RCLCPP_PUBLIC
  void
  setup_intra_process(
    uint64_t intra_process_publisher_id,
    StoreMessageCallbackT callback,
    const rcl_publisher_options_t & intra_process_options);

protected:
  std::shared_ptr<rcl_node_t> rcl_node_handle_;

  rcl_publisher_t publisher_handle_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t intra_process_publisher_handle_ = rcl_get_zero_initialized_publisher();

  uint64_t intra_process_publisher_id_;
  StoreMessageCallbackT store_intra_process_message_;

  rmw_gid_t rmw_gid_;
  rmw_gid_t intra_process_rmw_gid_;
};

/// A publisher publishes messages of any type to a topic.
template<typename MessageT, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, Alloc>)

  Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rcl_publisher_options_t & publisher_options,
    const std::shared_ptr<MessageAlloc> & allocator)
  : PublisherBase(
      node_base,
      topic,
      *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
      publisher_options),
    message_allocator_(allocator)
  {
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_.get());
  }

  virtual ~Publisher()
  {}

  /// Send a message to the topic for this publisher.
  /**
   * This function is templated on the input message type, MessageT.
   * \param[in] msg A shared pointer to the message to send.
   */
  virtual void
  publish(std::unique_ptr<MessageT, MessageDeleter> & msg)
  {
    this->do_inter_process_publish(msg.get());
    if (store_intra_process_message_) {
      // Take the pointer from the unique_msg, release it and pass as a void *
      // to the ipm. The ipm should then capture it again as a unique_ptr of
      // the correct type.
      // TODO(wjwwood):
      //   investigate how to transfer the custom deleter (if there is one)
      //   from the incoming unique_ptr through to the ipm's unique_ptr.
      //   See: http://stackoverflow.com/questions/11002641/dynamic-casting-for-unique-ptr
      MessageT * msg_ptr = msg.get();
      msg.release();
      uint64_t message_seq =
        store_intra_process_message_(intra_process_publisher_id_, msg_ptr, typeid(MessageT));
      rcl_interfaces::msg::IntraProcessMessage ipm;
      ipm.publisher_id = intra_process_publisher_id_;
      ipm.message_sequence = message_seq;
      auto status = rcl_publish(&intra_process_publisher_handle_, &ipm);
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
    } else {
      // Always destroy the message, even if we don't consume it, for consistency.
      msg.reset();
    }
  }

  virtual void
  publish(const std::shared_ptr<MessageT> & msg)
  {
    // Avoid allocating when not using intra process.
    if (!store_intra_process_message_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(msg.get());
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // TODO(wjwwood):
    //   The intra process manager should probably also be able to store
    //   shared_ptr's and do the "smart" thing based on other intra process
    //   subscriptions. For now call the other publish().
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *msg.get());
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    return this->publish(unique_msg);
  }

  virtual void
  publish(std::shared_ptr<const MessageT> msg)
  {
    // Avoid allocating when not using intra process.
    if (!store_intra_process_message_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(msg.get());
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // TODO(wjwwood):
    //   The intra process manager should probably also be able to store
    //   shared_ptr's and do the "smart" thing based on other intra process
    //   subscriptions. For now call the other publish().
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *msg.get());
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    return this->publish(unique_msg);
  }

  virtual void
  publish(const MessageT & msg)
  {
    // Avoid allocating when not using intra process.
    if (!store_intra_process_message_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(&msg);
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, msg);
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    return this->publish(unique_msg);
  }

  virtual void
  publish(const MessageT * msg)
  {
    if (!msg) {
      throw std::runtime_error("msg argument is nullptr");
    }
    return this->publish(*msg);
  }

  void
  publish(const rcl_serialized_message_t * serialized_msg)
  {
    if (store_intra_process_message_) {
      // TODO(Karsten1987): support serialized message passed by intraprocess
      throw std::runtime_error("storing serialized messages in intra process is not supported yet");
    }
    auto status = rcl_publish_serialized_message(&publisher_handle_, serialized_msg);
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
    }
  }

  void
  publish(std::shared_ptr<const rcl_serialized_message_t> serialized_msg)
  {
    return this->publish(serialized_msg.get());
  }

  std::shared_ptr<MessageAlloc> get_allocator() const
  {
    return message_allocator_;
  }

protected:
  void
  do_inter_process_publish(const MessageT * msg)
  {
    auto status = rcl_publish(&publisher_handle_, msg);
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

  std::shared_ptr<MessageAlloc> message_allocator_;

  MessageDeleter message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_
