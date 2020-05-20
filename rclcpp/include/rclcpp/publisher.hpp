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

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

template<typename MessageT, typename AllocatorT>
class LoanedMessage;

/// A publisher publishes messages of any type to a topic.
template<typename MessageT, typename AllocatorT = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  using MessageAllocatorTraits = allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAllocator, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, AllocatorT>)

  /// Default constructor.
  /**
   * The constructor for a Publisher is almost never called directly.
   * Instead, subscriptions should be instantiated through the function
   * rclcpp::create_publisher().
   *
   * \param[in] node_base NodeBaseInterface pointer that is used in part of the setup.
   * \param[in] topic Name of the topic to publish to.
   * \param[in] qos QoS profile for Subcription.
   * \param[in] options options for the subscription.
   */
  Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : PublisherBase(
      node_base,
      topic,
      *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
      options.template to_rcl_publisher_options<MessageT>(qos)),
    options_(options),
    message_allocator_(new MessageAllocator(*options.get_allocator().get()))
  {
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_.get());

    if (options_.event_callbacks.deadline_callback) {
      this->add_event_handler(
        options_.event_callbacks.deadline_callback,
        RCL_PUBLISHER_OFFERED_DEADLINE_MISSED);
    }
    if (options_.event_callbacks.liveliness_callback) {
      this->add_event_handler(
        options_.event_callbacks.liveliness_callback,
        RCL_PUBLISHER_LIVELINESS_LOST);
    }
    if (options_.event_callbacks.incompatible_qos_callback) {
      this->add_event_handler(
        options_.event_callbacks.incompatible_qos_callback,
        RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
    } else if (options_.use_default_callbacks) {
      // Register default callback when not specified
      try {
        this->add_event_handler(
          [this](QOSOfferedIncompatibleQoSInfo & info) {
            this->default_incompatible_qos_callback(info);
          },
          RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
      } catch (UnsupportedEventTypeException & /*exc*/) {
        // pass
      }
    }
    // Setup continues in the post construction method, post_init_setup().
  }

  /// Called post construction, so that construction may continue after shared_from_this() works.
  virtual
  void
  post_init_setup(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  {
    // Topic is unused for now.
    (void)topic;
    (void)options;

    // If needed, setup intra process communication.
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      auto context = node_base->get_context();
      // Get the intra process manager instance for this context.
      auto ipm = context->get_sub_context<rclcpp::experimental::IntraProcessManager>();
      // Register the publisher with the intra process manager.
      if (qos.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with keep all history qos policy");
      }
      if (qos.get_rmw_qos_profile().depth == 0) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with a zero qos history depth value");
      }
      if (qos.get_rmw_qos_profile().durability != RMW_QOS_POLICY_DURABILITY_VOLATILE) {
        throw std::invalid_argument(
                "intraprocess communication allowed only with volatile durability");
      }
      uint64_t intra_process_publisher_id = ipm->add_publisher(this->shared_from_this());
      this->setup_intra_process(
        intra_process_publisher_id,
        ipm);
    }
  }

  virtual ~Publisher()
  {}

  /// Borrow a loaned ROS message from the middleware.
  /**
   * If the middleware is capable of loaning memory for a ROS message instance,
   * the loaned message will be directly allocated in the middleware.
   * If not, the message allocator of this rclcpp::Publisher instance is being used.
   *
   * With a call to \sa `publish` the LoanedMessage instance is being returned to the middleware
   * or free'd accordingly to the allocator.
   * If the message is not being published but processed differently, the destructor of this
   * class will either return the message to the middleware or deallocate it via the internal
   * allocator.
   * \sa rclcpp::LoanedMessage for details of the LoanedMessage class.
   *
   * \return LoanedMessage containing memory for a ROS message of type MessageT
   */
  rclcpp::LoanedMessage<MessageT, AllocatorT>
  borrow_loaned_message()
  {
    return rclcpp::LoanedMessage<MessageT, AllocatorT>(this, this->get_allocator());
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
      this->do_inter_process_publish(*msg);
      return;
    }
    // If an interprocess subscription exist, then the unique_ptr is promoted
    // to a shared_ptr and published.
    // This allows doing the intraprocess publish first and then doing the
    // interprocess publish, resulting in lower publish-to-subscribe latency.
    // It's not possible to do that with an unique_ptr,
    // as do_intra_process_publish takes the ownership of the message.
    bool inter_process_publish_needed =
      get_subscription_count() > get_intra_process_subscription_count();

    if (inter_process_publish_needed) {
      auto shared_msg = this->do_intra_process_publish_and_return_shared(std::move(msg));
      this->do_inter_process_publish(*shared_msg);
    } else {
      this->do_intra_process_publish(std::move(msg));
    }
  }

  virtual void
  publish(const MessageT & msg)
  {
    // Avoid allocating when not using intra process.
    if (!intra_process_is_enabled_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(msg);
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // As the message is not const, a copy should be made.
    // A shared_ptr<const MessageT> could also be constructed here.
    auto ptr = MessageAllocatorTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocatorTraits::construct(*message_allocator_.get(), ptr, msg);
    MessageUniquePtr unique_msg(ptr, message_deleter_);
    this->publish(std::move(unique_msg));
  }

  void
  publish(const rcl_serialized_message_t & serialized_msg)
  {
    return this->do_serialized_publish(&serialized_msg);
  }

  void
  publish(const SerializedMessage & serialized_msg)
  {
    return this->do_serialized_publish(&serialized_msg.get_rcl_serialized_message());
  }

  /// Publish an instance of a LoanedMessage.
  /**
   * When publishing a loaned message, the memory for this ROS message will be deallocated
   * after being published.
   * The instance of the loaned message is no longer valid after this call.
   *
   * \param loaned_msg The LoanedMessage instance to be published.
   */
  void
  publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    if (!loaned_msg.is_valid()) {
      throw std::runtime_error("loaned message is not valid");
    }
    if (intra_process_is_enabled_) {
      // TODO(Karsten1987): support loaned message passed by intraprocess
      throw std::runtime_error("storing loaned messages in intra process is not supported yet");
    }

    // verify that publisher supports loaned messages
    // TODO(Karsten1987): This case separation has to be done in rclcpp
    // otherwise we have to ensure that every middleware implements
    // `rmw_publish_loaned_message` explicitly the same way as `rmw_publish`
    // by taking a copy of the ros message.
    if (this->can_loan_messages()) {
      // we release the ownership from the rclpp::LoanedMessage instance
      // and let the middleware clean up the memory.
      this->do_loaned_message_publish(loaned_msg.release());
    } else {
      // we don't release the ownership, let the middleware copy the ros message
      // and thus the destructor of rclcpp::LoanedMessage cleans up the memory.
      this->do_inter_process_publish(loaned_msg.get());
    }
  }

  std::shared_ptr<MessageAllocator>
  get_allocator() const
  {
    return message_allocator_;
  }

protected:
  void
  do_inter_process_publish(const MessageT & msg)
  {
    auto status = rcl_publish(publisher_handle_.get(), &msg, nullptr);

    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // next call will reset error message if not context
      if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
        rcl_context_t * context = rcl_publisher_get_context(publisher_handle_.get());
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
    auto status = rcl_publish_serialized_message(publisher_handle_.get(), serialized_msg, nullptr);
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
    }
  }

  void
  do_loaned_message_publish(MessageT * msg)
  {
    auto status = rcl_publish_loaned_message(publisher_handle_.get(), msg, nullptr);

    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // next call will reset error message if not context
      if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
        rcl_context_t * context = rcl_publisher_get_context(publisher_handle_.get());
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
  do_intra_process_publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    ipm->template do_intra_process_publish<MessageT, AllocatorT>(
      intra_process_publisher_id_,
      std::move(msg),
      message_allocator_);
  }

  std::shared_ptr<const MessageT>
  do_intra_process_publish_and_return_shared(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    return ipm->template do_intra_process_publish_and_return_shared<MessageT, AllocatorT>(
      intra_process_publisher_id_,
      std::move(msg),
      message_allocator_);
  }

  /// Copy of original options passed during construction.
  /**
   * It is important to save a copy of this so that the rmw payload which it
   * may contain is kept alive for the duration of the publisher.
   */
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> options_;

  std::shared_ptr<MessageAllocator> message_allocator_;

  MessageDeleter message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_
