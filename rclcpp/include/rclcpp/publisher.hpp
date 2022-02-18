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

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/publisher.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rosidl_runtime_cpp/traits.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/is_ros_compatible_type.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#include "tracetools/tracetools.h"

namespace rclcpp
{

template<typename MessageT, typename AllocatorT>
class LoanedMessage;

/// A publisher publishes messages of any type to a topic.
/**
 * MessageT must be a:
 *
 * - ROS message type with its own type support (e.g. std_msgs::msgs::String), or a
 * - rclcpp::TypeAdapter<CustomType, ROSMessageType>
 *   (e.g. rclcpp::TypeAdapter<std::string, std_msgs::msg::String), or a
 * - custom type that has been setup as the implicit type for a ROS type using
 *   RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(custom_type, ros_message_type)
 *
 * In the case that MessageT is ROS message type (e.g. std_msgs::msg::String),
 * both PublishedType and ROSMessageType will be that type.
 * In the case that MessageT is a TypeAdapter<CustomType, ROSMessageType> type
 * (e.g. TypeAdapter<std::string, std_msgs::msg::String>), PublishedType will
 * be the custom type, and ROSMessageType will be the ros message type.
 *
 * This is achieved because of the "identity specialization" for TypeAdapter,
 * which returns itself if it is already a TypeAdapter, and the default
 * specialization which allows ROSMessageType to be void.
 * \sa rclcpp::TypeAdapter for more details.
 */
template<typename MessageT, typename AllocatorT = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  static_assert(
    rclcpp::is_ros_compatible_type<MessageT>::value,
    "given message type is not compatible with ROS and cannot be used with a Publisher");

  /// MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, AllocatorT>;
  using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
  using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  using MessageAllocatorTraits
  [[deprecated("use PublishedTypeAllocatorTraits")]] =
    PublishedTypeAllocatorTraits;
  using MessageAllocator
  [[deprecated("use PublishedTypeAllocator")]] =
    PublishedTypeAllocator;
  using MessageDeleter
  [[deprecated("use PublishedTypeDeleter")]] =
    PublishedTypeDeleter;
  using MessageUniquePtr
  [[deprecated("use std::unique_ptr<PublishedType, PublishedTypeDeleter>")]] =
    std::unique_ptr<PublishedType, PublishedTypeDeleter>;
  using MessageSharedPtr
  [[deprecated("use std::shared_ptr<const PublishedType>")]] =
    std::shared_ptr<const PublishedType>;

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
   * \param[in] options Options for the subscription.
   */
  Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : PublisherBase(
      node_base,
      topic,
      rclcpp::get_message_type_support_handle<MessageT>(),
      options.template to_rcl_publisher_options<MessageT>(qos)),
    options_(options),
    published_type_allocator_(*options.get_allocator()),
    ros_message_type_allocator_(*options.get_allocator())
  {
    allocator::set_allocator_for_deleter(&published_type_deleter_, &published_type_allocator_);
    allocator::set_allocator_for_deleter(&ros_message_type_deleter_, &ros_message_type_allocator_);

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
      if (qos.history() != rclcpp::HistoryPolicy::KeepLast) {
        throw std::invalid_argument(
                "intraprocess communication allowed only with keep last history qos policy");
      }
      if (qos.depth() == 0) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with a zero qos history depth value");
      }
      if (qos.durability() != rclcpp::DurabilityPolicy::Volatile) {
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
   * \return LoanedMessage containing memory for a ROS message of type ROSMessageType
   */
  rclcpp::LoanedMessage<ROSMessageType, AllocatorT>
  borrow_loaned_message()
  {
    return rclcpp::LoanedMessage<ROSMessageType, AllocatorT>(
      *this,
      this->get_ros_message_type_allocator());
  }

  /// Publish a message on the topic.
  /**
   * This signature is enabled if the element_type of the std::unique_ptr is
   * a ROS message type, as opposed to the custom_type of a TypeAdapter, and
   * that type matches the type given when creating the publisher.
   *
   * This signature allows the user to give ownership of the message to rclcpp,
   * allowing for more efficient intra-process communication optimizations.
   *
   * \param[in] msg A unique pointer to the message to send.
   */
  template<typename T>
  typename std::enable_if_t<
    rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ROSMessageType>::value
  >
  publish(std::unique_ptr<T, ROSMessageTypeDeleter> msg)
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
      auto shared_msg =
        this->do_intra_process_ros_message_publish_and_return_shared(std::move(msg));
      this->do_inter_process_publish(*shared_msg);
    } else {
      this->do_intra_process_ros_message_publish(std::move(msg));
    }
  }

  /// Publish a message on the topic.
  /**
   * This signature is enabled if the object being published is
   * a ROS message type, as opposed to the custom_type of a TypeAdapter, and
   * that type matches the type given when creating the publisher.
   *
   * This signature allows the user to give a reference to a message, which is
   * copied onto the heap without modification so that a copy can be owned by
   * rclcpp and ownership of the copy can be moved later if needed.
   *
   * \param[in] msg A const reference to the message to send.
   */
  template<typename T>
  typename std::enable_if_t<
    rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ROSMessageType>::value
  >
  publish(const T & msg)
  {
    // Avoid allocating when not using intra process.
    if (!intra_process_is_enabled_) {
      // In this case we're not using intra process.
      return this->do_inter_process_publish(msg);
    }
    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // As the message is not const, a copy should be made.
    // A shared_ptr<const MessageT> could also be constructed here.
    auto unique_msg = this->duplicate_ros_message_as_unique_ptr(msg);
    this->publish(std::move(unique_msg));
  }

  /// Publish a message on the topic.
  /**
   * This signature is enabled if this class was created with a TypeAdapter and
   * the element_type of the std::unique_ptr matches the custom_type for the
   * TypeAdapter used with this class.
   *
   * This signature allows the user to give ownership of the message to rclcpp,
   * allowing for more efficient intra-process communication optimizations.
   *
   * \param[in] msg A unique pointer to the message to send.
   */
  template<typename T>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<MessageT>::is_specialized::value &&
    std::is_same<T, PublishedType>::value
  >
  publish(std::unique_ptr<T, PublishedTypeDeleter> msg)
  {
    // Avoid allocating when not using intra process.
    if (!intra_process_is_enabled_) {
      // In this case we're not using intra process.
      ROSMessageType ros_msg;
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*msg, ros_msg);
      return this->do_inter_process_publish(ros_msg);
    }

    bool inter_process_publish_needed =
      get_subscription_count() > get_intra_process_subscription_count();

    if (inter_process_publish_needed) {
      ROSMessageType ros_msg;
      // TODO(clalancette): This is unnecessarily doing an additional conversion
      // that may have already been done in do_intra_process_publish_and_return_shared().
      // We should just reuse that effort.
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*msg, ros_msg);
      this->do_intra_process_publish(std::move(msg));
      this->do_inter_process_publish(ros_msg);
    } else {
      this->do_intra_process_publish(std::move(msg));
    }
  }

  /// Publish a message on the topic.
  /**
   * This signature is enabled if this class was created with a TypeAdapter and
   * the given type matches the custom_type of the TypeAdapter.
   *
   * This signature allows the user to give a reference to a message, which is
   * copied onto the heap without modification so that a copy can be owned by
   * rclcpp and ownership of the copy can be moved later if needed.
   *
   * \param[in] msg A const reference to the message to send.
   */
  template<typename T>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<MessageT>::is_specialized::value &&
    std::is_same<T, PublishedType>::value
  >
  publish(const T & msg)
  {
    // Avoid double allocating when not using intra process.
    if (!intra_process_is_enabled_) {
      // Convert to the ROS message equivalent and publish it.
      ROSMessageType ros_msg;
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(msg, ros_msg);
      // In this case we're not using intra process.
      return this->do_inter_process_publish(ros_msg);
    }

    // Otherwise we have to allocate memory in a unique_ptr and pass it along.
    // As the message is not const, a copy should be made.
    // A shared_ptr<const MessageT> could also be constructed here.
    auto unique_msg = this->duplicate_type_adapt_message_as_unique_ptr(msg);
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
  publish(rclcpp::LoanedMessage<ROSMessageType, AllocatorT> && loaned_msg)
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
      this->do_loaned_message_publish(std::move(loaned_msg.release()));
    } else {
      // we don't release the ownership, let the middleware copy the ros message
      // and thus the destructor of rclcpp::LoanedMessage cleans up the memory.
      this->do_inter_process_publish(loaned_msg.get());
    }
  }

  [[deprecated("use get_published_type_allocator() or get_ros_message_type_allocator() instead")]]
  std::shared_ptr<PublishedTypeAllocator>
  get_allocator() const
  {
    return std::make_shared<PublishedTypeAllocator>(published_type_allocator_);
  }

  PublishedTypeAllocator
  get_published_type_allocator() const
  {
    return published_type_allocator_;
  }

  ROSMessageTypeAllocator
  get_ros_message_type_allocator() const
  {
    return ros_message_type_allocator_;
  }

protected:
  void
  do_inter_process_publish(const ROSMessageType & msg)
  {
    TRACEPOINT(rclcpp_publish, nullptr, static_cast<const void *>(&msg));
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
  do_loaned_message_publish(
    std::unique_ptr<ROSMessageType, std::function<void(ROSMessageType *)>> msg)
  {
    auto status = rcl_publish_loaned_message(publisher_handle_.get(), msg.get(), nullptr);

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
  do_intra_process_publish(std::unique_ptr<PublishedType, PublishedTypeDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    ipm->template do_intra_process_publish<PublishedType, ROSMessageType, AllocatorT>(
      intra_process_publisher_id_,
      std::move(msg),
      published_type_allocator_);
  }

  void
  do_intra_process_ros_message_publish(std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    ipm->template do_intra_process_publish<ROSMessageType, ROSMessageType, AllocatorT>(
      intra_process_publisher_id_,
      std::move(msg),
      ros_message_type_allocator_);
  }

  std::shared_ptr<const ROSMessageType>
  do_intra_process_ros_message_publish_and_return_shared(
    std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publish called after destruction of intra process manager");
    }
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    return ipm->template do_intra_process_publish_and_return_shared<ROSMessageType, ROSMessageType,
             AllocatorT>(
      intra_process_publisher_id_,
      std::move(msg),
      ros_message_type_allocator_);
  }


  /// Return a new unique_ptr using the ROSMessageType of the publisher.
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  create_ros_message_unique_ptr()
  {
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr);
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  /// Duplicate a given ros message as a unique_ptr.
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  duplicate_ros_message_as_unique_ptr(const ROSMessageType & msg)
  {
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr, msg);
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  /// Duplicate a given type adapted message as a unique_ptr.
  std::unique_ptr<PublishedType, PublishedTypeDeleter>
  duplicate_type_adapt_message_as_unique_ptr(const PublishedType & msg)
  {
    auto ptr = PublishedTypeAllocatorTraits::allocate(published_type_allocator_, 1);
    PublishedTypeAllocatorTraits::construct(published_type_allocator_, ptr, msg);
    return std::unique_ptr<PublishedType, PublishedTypeDeleter>(ptr, published_type_deleter_);
  }

  /// Copy of original options passed during construction.
  /**
   * It is important to save a copy of this so that the rmw payload which it
   * may contain is kept alive for the duration of the publisher.
   */
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> options_;

  PublishedTypeAllocator published_type_allocator_;
  PublishedTypeDeleter published_type_deleter_;
  ROSMessageTypeAllocator ros_message_type_allocator_;
  ROSMessageTypeDeleter ros_message_type_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_
