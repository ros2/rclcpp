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

#include <rcl/error_handling.h>
#include <rcl/publisher.h>

#include "rcl_interfaces/msg/intra_process_message.hpp"
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

// Forward declaration for friend statement
namespace node
{
class Node;
}  // namespace node

namespace publisher
{

class PublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase);
  /// Default constructor.
  /**
   * Typically, a publisher is not created through this method, but instead is created through a
   * call to `Node::create_publisher`.
   * \param[in] node_handle The corresponding rmw representation of the owner node.
   * \param[in] topic The topic that this publisher publishes on.
   * \param[in] queue_size The maximum number of unpublished messages to queue.
   */
  RCLCPP_PUBLIC
  PublisherBase(
    std::shared_ptr<rcl_node_t> node_handle,
    std::string topic,
    size_t queue_size);

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~PublisherBase();

  /// Get the topic that this publisher publishes on.
  // \return The topic name.
  RCLCPP_PUBLIC
  const std::string &
  get_topic_name() const;

  /// Get the queue size for this publisher.
  // \return The queue size.
  RCLCPP_PUBLIC
  size_t
  get_queue_size() const;

  /// Get the global identifier for this publisher (used in rmw and by DDS).
  // \return The gid.
  RCLCPP_PUBLIC
  const rmw_gid_t &
  get_gid() const;

  /// Get the global identifier for this publisher used by intra-process communication.
  // \return The intra-process gid.
  RCLCPP_PUBLIC
  const rmw_gid_t &
  get_intra_process_gid() const;

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

  typedef std::function<uint64_t(uint64_t, void *, const std::type_info &)> StoreMessageCallbackT;

protected:
  RCLCPP_PUBLIC
  void
  setup_intra_process(
    uint64_t intra_process_publisher_id,
    StoreMessageCallbackT callback,
    rcl_publisher_options_t & intra_process_options);

  std::shared_ptr<rcl_node_t> node_handle_;

  rcl_publisher_t publisher_handle_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t intra_process_publisher_handle_ = rcl_get_zero_initialized_publisher();
  rcl_allocator_t rcl_allocator_;

  std::string topic_;
  size_t queue_size_;

  uint64_t intra_process_publisher_id_;
  StoreMessageCallbackT store_intra_process_message_;

  rmw_gid_t rmw_gid_;
  rmw_gid_t intra_process_rmw_gid_;
};

/// A publisher publishes messages of any type to a topic.
template<typename MessageT, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
  friend rclcpp::node::Node;

public:
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, Alloc>);

  Publisher(
    std::shared_ptr<rcl_node_t> node_handle,
    std::string topic,
    rcl_publisher_options_t & publisher_options,
    std::shared_ptr<MessageAlloc> allocator)
  : PublisherBase(node_handle, topic, publisher_options.qos.depth), message_allocator_(allocator)
  {
    using rosidl_generator_cpp::get_message_type_support_handle;
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_.get());

    rcl_allocator_ = publisher_options.allocator;
    auto type_support_handle = get_message_type_support_handle<MessageT>();
    if (rcl_publisher_init(
          &publisher_handle_, node_handle_.get(), type_support_handle,
          topic.c_str(), &publisher_options) != RCL_RET_OK)
    {
      throw std::runtime_error(
        std::string("could not create publisher: ") +
        rcl_get_error_string_safe());
    }
    // Life time of this object is tied to the publisher handle.
    rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(&publisher_handle_);
    if (!publisher_rmw_handle) {
      throw std::runtime_error(
        std::string("failed to get rmw handle: ") + rcl_get_error_string_safe());
    }
    if (rmw_get_gid_for_publisher(publisher_rmw_handle, &rmw_gid_) != RMW_RET_OK)
    {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to get publisher gid: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }
  }


  /// Send a message to the topic for this publisher.
  /**
   * This function is templated on the input message type, MessageT.
   * \param[in] msg A shared pointer to the message to send.
   */
  void
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
      if (status != RCL_RET_OK) {
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          std::string("failed to publish intra process message: ") + rcl_get_error_string_safe());
        // *INDENT-ON*
      }
    } else {
      // Always destroy the message, even if we don't consume it, for consistency.
      msg.reset();
    }
  }

  void
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

  void
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

  void
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

  std::shared_ptr<MessageAlloc> get_allocator() const
  {
    return message_allocator_;
  }

protected:
  void
  do_inter_process_publish(const MessageT * msg)
  {
    auto status = rcl_publish(&publisher_handle_, msg);
    if (status != RCL_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to publish message: ") + rcl_get_error_string_safe());
      // *INDENT-ON*
    }
  }

  std::shared_ptr<MessageAlloc> message_allocator_;

  MessageDeleter message_deleter_;
};

}  // namespace publisher
}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_
