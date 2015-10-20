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

#ifndef RCLCPP_RCLCPP_PUBLISHER_HPP_
#define RCLCPP_RCLCPP_PUBLISHER_HPP_

#include <rclcpp/macros.hpp>

#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rcl_interfaces/msg/intra_process_message.hpp>
#include <rmw/impl/cpp/demangle.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/allocator/allocator_deleter.hpp>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node
{
class Node;
} // namespace node

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
   * \param[in] publisher_handle The rmw publisher handle corresponding to this publisher.
   * \param[in] topic The topic that this publisher publishes on.
   * \param[in] queue_size The maximum number of unpublished messages to queue.
   */
  PublisherBase(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_publisher_t * publisher_handle,
    std::string topic,
    size_t queue_size)
  : node_handle_(node_handle), publisher_handle_(publisher_handle),
    intra_process_publisher_handle_(nullptr),
    topic_(topic), queue_size_(queue_size),
    intra_process_publisher_id_(0), store_intra_process_message_(nullptr)
  {
    // Life time of this object is tied to the publisher handle.
    if (rmw_get_gid_for_publisher(publisher_handle_, &rmw_gid_) != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to get publisher gid: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }
  }

  /// Default destructor.
  virtual ~PublisherBase()
  {
    if (intra_process_publisher_handle_) {
      if (rmw_destroy_publisher(node_handle_.get(), intra_process_publisher_handle_)) {
        fprintf(
          stderr,
          "Error in destruction of intra process rmw publisher handle: %s\n",
          rmw_get_error_string_safe());
      }
    }
    if (publisher_handle_) {
      if (rmw_destroy_publisher(node_handle_.get(), publisher_handle_) != RMW_RET_OK) {
        fprintf(
          stderr,
          "Error in destruction of rmw publisher handle: %s\n",
          rmw_get_error_string_safe());
      }
    }
  }

  /// Get the topic that this publisher publishes on.
  // \return The topic name.
  const std::string &
  get_topic_name() const
  {
    return topic_;
  }

  /// Get the queue size for this publisher.
  // \return The queue size.
  size_t
  get_queue_size() const
  {
    return queue_size_;
  }

  /// Get the global identifier for this publisher (used in rmw and by DDS).
  // \return The gid.
  const rmw_gid_t &
  get_gid() const
  {
    return rmw_gid_;
  }

  /// Get the global identifier for this publisher used by intra-process communication.
  // \return The intra-process gid.
  const rmw_gid_t &
  get_intra_process_gid() const
  {
    return intra_process_rmw_gid_;
  }

  /// Compare this publisher to a gid.
  /**
   * Note that this function calls the next function.
   * \param[in] gid Reference to a gid.
   * \return True if the publisher's gid matches the input.
   */
  bool
  operator==(const rmw_gid_t & gid) const
  {
    return *this == &gid;
  }

  /// Compare this publisher to a pointer gid.
  /**
   * A wrapper for comparing this publisher's gid to the input using rmw_compare_gids_equal.
   * \param[in] gid A pointer to a gid.
   * \return True if this publisher's gid matches the input.
   */
  bool
  operator==(const rmw_gid_t * gid) const
  {
    bool result = false;
    auto ret = rmw_compare_gids_equal(gid, &this->get_gid(), &result);
    if (ret != RMW_RET_OK) {
      throw std::runtime_error(
              std::string("failed to compare gids: ") + rmw_get_error_string_safe());
    }
    if (!result) {
      ret = rmw_compare_gids_equal(gid, &this->get_intra_process_gid(), &result);
      if (ret != RMW_RET_OK) {
        throw std::runtime_error(
                std::string("failed to compare gids: ") + rmw_get_error_string_safe());
      }
    }
    return result;
  }

  typedef std::function<uint64_t(uint64_t, void *, const std::type_info &)> StoreMessageCallbackT;

protected:
  void
  setup_intra_process(
    uint64_t intra_process_publisher_id,
    StoreMessageCallbackT callback,
    rmw_publisher_t * intra_process_publisher_handle)
  {
    intra_process_publisher_id_ = intra_process_publisher_id;
    store_intra_process_message_ = callback;
    intra_process_publisher_handle_ = intra_process_publisher_handle;
    // Life time of this object is tied to the publisher handle.
    auto ret = rmw_get_gid_for_publisher(intra_process_publisher_handle_, &intra_process_rmw_gid_);
    if (ret != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to create intra process publisher gid: ") +
        rmw_get_error_string_safe());
      // *INDENT-ON*
    }
  }

  std::shared_ptr<rmw_node_t> node_handle_;

  rmw_publisher_t * publisher_handle_;
  rmw_publisher_t * intra_process_publisher_handle_;

  std::string topic_;
  size_t queue_size_;

  uint64_t intra_process_publisher_id_;
  StoreMessageCallbackT store_intra_process_message_;

  rmw_gid_t rmw_gid_;
  rmw_gid_t intra_process_rmw_gid_;

  std::mutex intra_process_publish_mutex_;
};

/// A publisher publishes messages of any type to a topic.
template<typename MessageT, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
  friend rclcpp::node::Node;

public:
  using MessageAlloc = allocator::AllocRebind<MessageT, Alloc>;
  using MessageDeleter = allocator::Deleter<typename MessageAlloc::allocator_type, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, Alloc>);

  Publisher(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_publisher_t * publisher_handle,
    std::string topic,
    size_t queue_size,
    std::shared_ptr<Alloc> allocator)
  : PublisherBase(node_handle, publisher_handle, topic, queue_size)
  {
    message_allocator_ = new typename MessageAlloc::allocator_type(*allocator.get());
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_);
  }


  /// Send a message to the topic for this publisher.
  /**
   * This function is templated on the input message type, MessageT.
   * \param[in] msg A shared pointer to the message to send.
   */
  void
  publish(MessageUniquePtr & msg)
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
      uint64_t message_seq;
      {
        std::lock_guard<std::mutex> lock(intra_process_publish_mutex_);
        message_seq =
          store_intra_process_message_(intra_process_publisher_id_, msg_ptr, typeid(MessageT));
      }
      rcl_interfaces::msg::IntraProcessMessage ipm;
      ipm.publisher_id = intra_process_publisher_id_;
      ipm.message_sequence = message_seq;
      auto status = rmw_publish(intra_process_publisher_handle_, &ipm);
      if (status != RMW_RET_OK) {
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          std::string("failed to publish intra process message: ") + rmw_get_error_string_safe());
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
    auto ptr = MessageAlloc::allocate(*message_allocator_, 1);
    MessageAlloc::construct(*message_allocator_, ptr, *msg.get());
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
    auto ptr = MessageAlloc::allocate(*message_allocator_, 1);
    MessageAlloc::construct(*message_allocator_, ptr, *msg.get());
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
    auto ptr = MessageAlloc::allocate(*message_allocator_, 1);
    MessageAlloc::construct(*message_allocator_, ptr, msg);
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    return this->publish(unique_msg);
  }

protected:
  void
  do_inter_process_publish(const MessageT * msg)
  {
    auto status = rmw_publish(publisher_handle_, msg);
    if (status != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to publish message: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }
  }

  typename MessageAlloc::allocator_type * message_allocator_;

  MessageDeleter message_deleter_;

};

} /* namespace publisher */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PUBLISHER_HPP_ */
