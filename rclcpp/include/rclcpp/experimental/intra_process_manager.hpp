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

#ifndef RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_
#define RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_

#include <rmw/types.h>

#include <shared_mutex>

#include <iterator>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include <typeinfo>

#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/ros_message_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcpputils/asserts.hpp"

namespace rclcpp
{

namespace experimental
{

/// This class performs intra process communication between nodes.
/**
 * This class is used in the creation of publishers and subscriptions.
 * A singleton instance of this class is owned by a rclcpp::Context and a
 * rclcpp::Node can use an associated Context to get an instance of this class.
 * Nodes which do not have a common Context will not exchange intra process
 * messages because they do not share access to the same instance of this class.
 *
 * When a Node creates a subscription, it can also create a helper class,
 * called SubscriptionIntraProcess, meant to receive intra process messages.
 * It can be registered with this class.
 * It is also allocated an id which is unique among all publishers
 * and subscriptions in this process and that is associated to the subscription.
 *
 * When a Node creates a publisher, as with subscriptions, a helper class can
 * be registered with this class.
 * This is required in order to publish intra-process messages.
 * It is also allocated an id which is unique among all publishers
 * and subscriptions in this process and that is associated to the publisher.
 *
 * When a publisher or a subscription are registered, this class checks to see
 * which other subscriptions or publishers it will communicate with,
 * i.e. they have the same topic and compatible QoS.
 *
 * When the user publishes a message, if intra-process communication is enabled
 * on the publisher, the message is given to this class.
 * Using the publisher id, a list of recipients for the message is selected.
 * For each subscription in the list, this class stores the message, whether
 * sharing ownership or making a copy, in a buffer associated with the
 * subscription helper class.
 *
 * The subscription helper class contains a buffer where published
 * intra-process messages are stored until they are taken from the subscription.
 * Depending on the data type stored in the buffer, the subscription helper
 * class can request either shared or exclusive ownership on the message.
 *
 * Thus, when an intra-process message is published, this class knows how many
 * intra-process subscriptions needs it and how many require ownership.
 * This information allows this class to operate efficiently by performing the
 * fewest number of copies of the message required.
 *
 * This class is neither CopyConstructable nor CopyAssignable.
 */
class IntraProcessManager
{
private:
  RCLCPP_DISABLE_COPY(IntraProcessManager)

public:
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessManager)

  RCLCPP_PUBLIC
  IntraProcessManager();

  RCLCPP_PUBLIC
  virtual ~IntraProcessManager();

  /// Register a subscription with the manager, returns subscriptions unique id.
  /**
   * This method stores the subscription intra process object, together with
   * the information of its wrapped subscription (i.e. topic name and QoS).
   *
   * In addition this generates a unique intra process id for the subscription.
   *
   * \param subscription the SubscriptionIntraProcess to register.
   * \return an unsigned 64-bit integer which is the subscription's unique id.
   */
  RCLCPP_PUBLIC
  uint64_t
  add_subscription(rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr subscription);

  /// Unregister a subscription using the subscription's unique id.
  /**
   * This method does not allocate memory.
   *
   * \param intra_process_subscription_id id of the subscription to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_subscription(uint64_t intra_process_subscription_id);

  /// Register a publisher with the manager, returns the publisher unique id.
  /**
   * This method stores the publisher intra process object, together with
   * the information of its wrapped publisher (i.e. topic name and QoS).
   *
   * In addition this generates a unique intra process id for the publisher.
   *
   * \param publisher publisher to be registered with the manager.
   * \return an unsigned 64-bit integer which is the publisher's unique id.
   */
  RCLCPP_PUBLIC
  uint64_t
  add_publisher(rclcpp::PublisherBase::SharedPtr publisher);

  /// Unregister a publisher using the publisher's unique id.
  /**
   * This method does not allocate memory.
   *
   * \param intra_process_publisher_id id of the publisher to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_publisher(uint64_t intra_process_publisher_id);

  /// Publishes an intra-process message, passed as a unique pointer.
  /**
   * This is one of the two methods for publishing intra-process.
   *
   * Using the intra-process publisher id, a list of recipients is obtained.
   * This list is split in half, depending whether they require ownership or not.
   *
   * This particular method takes a unique pointer as input.
   * The pointer can be promoted to a shared pointer and passed to all the subscriptions
   * that do not require ownership.
   * In case of subscriptions requiring ownership, the message will be copied for all of
   * them except the last one, when ownership can be transferred.
   *
   * This method can save an additional copy compared to the shared pointer one.
   *
   * This method can throw an exception if the publisher id is not found or
   * if the publisher shared_ptr given to add_publisher has gone out of scope.
   *
   * This method does allocate memory.
   *
   * \param intra_process_publisher_id the id of the publisher of this message.
   * \param message the message that is being stored.
   * \param allocator for allocations when buffering messages.
   */
  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>
  >
  void
  do_intra_process_publish(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator)
  {
    constexpr bool is_serialized_publisher =
      serialization_traits::is_serialized_message_class<MessageT>::value;
    using Indices = SplitSubscriptionsIndices<is_serialized_publisher>;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return;
    }
    const auto & sub_ids = publisher_it->second;

    // check if (de)serialization is needed
    if (sub_ids.take_subscriptions[Indices::ownership_other].size() +
      sub_ids.take_subscriptions[Indices::shared_other].size() > 0)
    {
      do_intra_process_publish_other_type<MessageT, ROSMessageType, Alloc, Deleter>(
        intra_process_publisher_id,
        message.get(), allocator,
        sub_ids.take_subscriptions[Indices::ownership_other],
        sub_ids.take_subscriptions[Indices::shared_other]
      );
    }

    do_intra_process_publish_same_type<MessageT, ROSMessageType, Alloc, Deleter>(
      intra_process_publisher_id,
      std::move(message), allocator,
      sub_ids.take_subscriptions[Indices::ownership_same],
      sub_ids.take_subscriptions[Indices::shared_same]
    );
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>
  >
  void
  do_intra_process_publish_same_type(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator,
    const std::vector<uint64_t> & take_ownership_subscriptions,
    const std::vector<uint64_t> & take_shared_subscriptions)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return;
    }

    if (take_ownership_subscriptions.empty()) {
      // None of the buffers require ownership, so we promote the pointer
      std::shared_ptr<MessageT> msg = std::move(message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        msg, take_shared_subscriptions);
    } else if (!take_ownership_subscriptions.empty() && // NOLINT
      take_shared_subscriptions.size() <= 1)
    {
      // There is at maximum 1 buffer that does not require ownership.
      // So this case is equivalent to all the buffers requiring ownership

      // Merge the two vector of ids into a unique one
      std::vector<uint64_t> concatenated_vector(take_shared_subscriptions);
      concatenated_vector.insert(
        concatenated_vector.end(),
        take_ownership_subscriptions.begin(),
        take_ownership_subscriptions.end());
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        std::move(message),
        concatenated_vector,
        allocator);
    } else if (!take_ownership_subscriptions.empty() && // NOLINT
      take_shared_subscriptions.size() > 1)
    {
      // Construct a new shared pointer from the message
      // for the buffers that do not require ownership
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        shared_msg, take_shared_subscriptions);
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        std::move(message), take_ownership_subscriptions, allocator);
    }
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>
  >
  std::shared_ptr<const MessageT>
  do_intra_process_publish_and_return_shared(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator)
  {
    constexpr bool is_serialized_publisher =
      serialization_traits::is_serialized_message_class<MessageT>::value;
    using Indices = SplitSubscriptionsIndices<is_serialized_publisher>;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return nullptr;
    }
    const auto & sub_ids = publisher_it->second;

    // check if (de)serialization is needed
    if (sub_ids.take_subscriptions[Indices::ownership_other].size() +
      sub_ids.take_subscriptions[Indices::shared_other].size() > 0)
    {
      do_intra_process_publish_other_type<MessageT, ROSMessageType, Alloc, Deleter>(
        intra_process_publisher_id,
        message.get(), allocator,
        sub_ids.take_subscriptions[Indices::ownership_other],
        sub_ids.take_subscriptions[Indices::shared_other]
      );
    }

    return do_intra_process_publish_and_return_shared_same_type<MessageT, ROSMessageType, Alloc,
             Deleter>(
      intra_process_publisher_id,
      std::move(message), allocator,
      sub_ids.take_subscriptions[Indices::ownership_same],
      sub_ids.take_subscriptions[Indices::shared_same]
             );
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>
  >
  std::shared_ptr<const MessageT>
  do_intra_process_publish_and_return_shared_same_type(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator,
    const std::vector<uint64_t> & take_ownership_subscriptions,
    const std::vector<uint64_t> & take_shared_subscriptions)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return nullptr;
    }

    if (take_ownership_subscriptions.empty()) {
      // If there are no owning, just convert to shared.
      std::shared_ptr<MessageT> shared_msg = std::move(message);
      if (!take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          shared_msg, take_shared_subscriptions);
      }
      return shared_msg;
    } else {
      // Construct a new shared pointer from the message for the buffers that
      // do not require ownership and to return.
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      if (!take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          shared_msg,
          take_shared_subscriptions);
      }
      if (!take_ownership_subscriptions.empty()) {
        this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          std::move(message),
          take_ownership_subscriptions,
          allocator);
      }
      return shared_msg;
    }
  }

  /// Return true if the given rmw_gid_t matches any stored Publishers.
  RCLCPP_PUBLIC
  bool
  matches_any_publishers(const rmw_gid_t * id) const;

  /// Return the number of intraprocess subscriptions that are matched with a given publisher id.
  RCLCPP_PUBLIC
  size_t
  get_subscription_count(uint64_t intra_process_publisher_id) const;

  RCLCPP_PUBLIC
  rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr
  get_subscription_intra_process(uint64_t intra_process_subscription_id);

  /// Return the lowest available capacity for all subscription buffers for a publisher id.
  RCLCPP_PUBLIC
  size_t
  lowest_available_capacity(const uint64_t intra_process_publisher_id) const;

private:
  struct SplittedSubscriptions
  {
    enum
    {
      IndexSharedTyped = 0u, IndexSharedSerialized = 1u,
      IndexOwnershipTyped = 2u, IndexOwnershipSerialized = 3u,
      IndexNum = 4u
    };

    /// get the index for "take_subscriptions" depending on shared/serialized
    constexpr static uint32_t
    get_index(const bool is_shared, const bool is_serialized)
    {
      return (is_serialized ? IndexSharedTyped : IndexSharedSerialized) +
             (is_shared ? IndexSharedTyped : IndexOwnershipTyped);
    }

    std::vector<uint64_t> take_subscriptions[IndexNum];
  };

  template<bool is_serialized>
  struct SplitSubscriptionsIndices
  {
    constexpr static auto ownership_same = SplittedSubscriptions::get_index(
      false,
      is_serialized);
    constexpr static auto shared_same = SplittedSubscriptions::get_index(
      true,
      is_serialized);
    constexpr static auto ownership_other = SplittedSubscriptions::get_index(
      false,
      !is_serialized);
    constexpr static auto shared_other = SplittedSubscriptions::get_index(
      true,
      !is_serialized);
  };

  using SubscriptionMap =
    std::unordered_map<uint64_t, rclcpp::experimental::SubscriptionIntraProcessBase::WeakPtr>;

  using PublisherMap =
    std::unordered_map<uint64_t, rclcpp::PublisherBase::WeakPtr>;

  using PublisherToSubscriptionIdsMap =
    std::unordered_map<uint64_t, SplittedSubscriptions>;

  RCLCPP_PUBLIC
  static
  uint64_t
  get_next_unique_id();

  RCLCPP_PUBLIC
  void
  insert_sub_id_for_pub(
    uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method,
    bool is_serialized_subscriber);

  RCLCPP_PUBLIC
  bool
  can_communicate(
    rclcpp::PublisherBase::SharedPtr pub,
    rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr sub) const;

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>>
  typename std::enable_if_t<
    std::is_same<MessageT, ROSMessageType>::value,
    void
  >
  do_intra_process_publish_other_type(
    uint64_t intra_process_publisher_id,
    const MessageT * unsupported_message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type &,
    const std::vector<uint64_t> & take_ownership_subscriptions,
    const std::vector<uint64_t> & take_shared_subscriptions)
  {
    typedef std::allocator<rclcpp::SerializedMessage> SerializedAlloc;
    using SerializedAllocatorTraits = allocator::AllocRebind<rclcpp::SerializedMessage,
        SerializedAlloc>;

    SerializedAllocatorTraits::allocator_type serialized_allocator;

    auto ptr = SerializedAllocatorTraits::allocate(serialized_allocator, 1);
    SerializedAllocatorTraits::construct(serialized_allocator, ptr);
    std::unique_ptr<rclcpp::SerializedMessage> serialized_message(ptr);
    if constexpr (!std::is_same<ROSMessageType, rclcpp::SerializedMessage>::value) {
      Serialization<ROSMessageType> serializer;
      serializer.serialize_message(unsupported_message, serialized_message.get());
    } else {
      (void)unsupported_message;
      throw std::runtime_error("Serialized message cannot be serialized.");
    }

    this->template do_intra_process_publish_and_return_shared_same_type<rclcpp::SerializedMessage,
      rclcpp::SerializedMessage, SerializedAlloc>(
      intra_process_publisher_id,
      std::move(serialized_message),
      serialized_allocator,
      take_ownership_subscriptions,
      take_shared_subscriptions
      );
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>>
  typename std::enable_if_t<
    !std::is_same<MessageT, ROSMessageType>::value,
    void
  >
  do_intra_process_publish_other_type(
    uint64_t intra_process_publisher_id,
    const MessageT * unsupported_message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type &,
    const std::vector<uint64_t> & take_ownership_subscriptions,
    const std::vector<uint64_t> & take_shared_subscriptions)
  {
    typedef std::allocator<rclcpp::SerializedMessage> SerializedAlloc;
    using SerializedAllocatorTraits = allocator::AllocRebind<rclcpp::SerializedMessage,
        SerializedAlloc>;

    SerializedAllocatorTraits::allocator_type serialized_allocator;

    auto ptr = SerializedAllocatorTraits::allocate(serialized_allocator, 1);
    SerializedAllocatorTraits::construct(serialized_allocator, ptr);
    std::unique_ptr<rclcpp::SerializedMessage> serialized_message(ptr);
    Serialization<ROSMessageType> serializer;
    ROSMessageType ros_msg;
    rclcpp::TypeAdapter<MessageT, ROSMessageType>::convert_to_ros_message(
      *unsupported_message,
      ros_msg);
    serializer.serialize_message(&ros_msg, serialized_message.get());

    this->template do_intra_process_publish_and_return_shared_same_type<rclcpp::SerializedMessage,
      rclcpp::SerializedMessage, SerializedAlloc>(
      intra_process_publisher_id,
      std::move(serialized_message),
      serialized_allocator,
      take_ownership_subscriptions,
      take_shared_subscriptions
      );
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>>
  typename std::enable_if_t<
    !std::is_same<MessageT, rclcpp::SerializedMessage>::value,
    void
  >
  do_intra_process_publish_other_type(
    uint64_t intra_process_publisher_id,
    const rclcpp::SerializedMessage * unsupported_message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator,
    const std::vector<uint64_t> & take_ownership_subscriptions,
    const std::vector<uint64_t> & take_shared_subscriptions)
  {
    for (const auto id : take_ownership_subscriptions) {
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(id);
        continue;
      }

      if (subscription_base->serve_serialized_message(
          unsupported_message,
          this,
          intra_process_publisher_id,
          static_cast<void *>(&allocator),
          take_ownership_subscriptions,
          take_shared_subscriptions
      ))
      {
        // message was successfully converted and forwarded, so stop further forwarding
        return;
      }
    }

    for (const auto id : take_shared_subscriptions) {
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(id);
        continue;
      }

      if (subscription_base->serve_serialized_message(
          unsupported_message,
          this,
          intra_process_publisher_id,
          static_cast<void *>(&allocator),
          take_ownership_subscriptions,
          take_shared_subscriptions
      ))
      {
        // message was successfully converted and forwarded, so stop further forwarding
        return;
      }
    }
  }

  template<
    typename MessageT,
    typename Alloc,
    typename Deleter,
    typename ROSMessageType>
  void
  add_shared_msg_to_buffers(
    std::shared_ptr<const MessageT> message,
    std::vector<uint64_t> subscription_ids)
  {
    using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
    using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
    using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

    using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
    using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, Alloc>;
    using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
    using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

    for (auto id : subscription_ids) {
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(id);
        continue;
      }

      auto subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionIntraProcessBuffer<PublishedType,
        PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>
        >(subscription_base);
      if (subscription != nullptr) {
        subscription->provide_intra_process_data(message);
        continue;
      }

      auto ros_message_subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,
        ROSMessageTypeAllocator, ROSMessageTypeDeleter>
        >(subscription_base);
      if (nullptr == ros_message_subscription) {
        throw std::runtime_error(
                "failed to dynamic cast SubscriptionIntraProcessBase to "
                "SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>, or to "
                "SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,ROSMessageTypeAllocator,"
                "ROSMessageTypeDeleter> which can happen when the publisher and "
                "subscription use different allocator types, which is not supported");
      }

      if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
        ROSMessageType ros_msg;
        rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*message, ros_msg);
        ros_message_subscription->provide_intra_process_message(
          std::make_shared<ROSMessageType>(ros_msg));
      } else {
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          ros_message_subscription->provide_intra_process_message(message);
        } else {
          if constexpr (std::is_same<typename rclcpp::TypeAdapter<MessageT,
            ROSMessageType>::ros_message_type, ROSMessageType>::value)
          {
            ROSMessageType ros_msg;
            rclcpp::TypeAdapter<MessageT, ROSMessageType>::convert_to_ros_message(
              *message, ros_msg);
            ros_message_subscription->provide_intra_process_message(
              std::make_shared<ROSMessageType>(ros_msg));
          }
        }
      }
    }
  }

  template<
    typename MessageT,
    typename Alloc,
    typename Deleter,
    typename ROSMessageType>
  void
  add_owned_msg_to_buffers(
    std::unique_ptr<MessageT, Deleter> message,
    std::vector<uint64_t> subscription_ids,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;

    using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
    using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
    using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

    using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
    using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, Alloc>;
    using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
    using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

    for (auto it = subscription_ids.begin(); it != subscription_ids.end(); it++) {
      auto subscription_it = subscriptions_.find(*it);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(subscription_it);
        continue;
      }

      auto subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionIntraProcessBuffer<PublishedType,
        PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>
        >(subscription_base);
      if (subscription != nullptr) {
        if (std::next(it) == subscription_ids.end()) {
          // If this is the last subscription, give up ownership
          subscription->provide_intra_process_data(std::move(message));
          // Last message delivered, break from for loop
          break;
        } else {
          // Copy the message since we have additional subscriptions to serve
          Deleter deleter = message.get_deleter();
          auto ptr = MessageAllocTraits::allocate(allocator, 1);
          MessageAllocTraits::construct(allocator, ptr, *message);

          subscription->provide_intra_process_data(MessageUniquePtr(ptr, deleter));
        }

        continue;
      }

      auto ros_message_subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,
        ROSMessageTypeAllocator, ROSMessageTypeDeleter>
        >(subscription_base);
      if (nullptr == ros_message_subscription) {
        throw std::runtime_error(
                "failed to dynamic cast SubscriptionIntraProcessBase to "
                "SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>, or to "
                "SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,ROSMessageTypeAllocator,"
                "ROSMessageTypeDeleter> which can happen when the publisher and "
                "subscription use different allocator types, which is not supported");
      }

      if constexpr (rclcpp::TypeAdapter<MessageT, ROSMessageType>::is_specialized::value) {
        ROSMessageTypeAllocator ros_message_alloc(allocator);
        auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_alloc, 1);
        ROSMessageTypeAllocatorTraits::construct(ros_message_alloc, ptr);
        ROSMessageTypeDeleter deleter;
        allocator::set_allocator_for_deleter(&deleter, &allocator);
        rclcpp::TypeAdapter<MessageT, ROSMessageType>::convert_to_ros_message(*message, *ptr);
        auto ros_msg = std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, deleter);
        ros_message_subscription->provide_intra_process_message(std::move(ros_msg));
      } else {
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          if (std::next(it) == subscription_ids.end()) {
            // If this is the last subscription, give up ownership
            ros_message_subscription->provide_intra_process_message(std::move(message));
            // Last message delivered, break from for loop
            break;
          } else {
            // Copy the message since we have additional subscriptions to serve
            Deleter deleter = message.get_deleter();
            allocator::set_allocator_for_deleter(&deleter, &allocator);
            auto ptr = MessageAllocTraits::allocate(allocator, 1);
            MessageAllocTraits::construct(allocator, ptr, *message);

            ros_message_subscription->provide_intra_process_message(
              MessageUniquePtr(ptr, deleter));
          }
        }
      }
    }
  }

  PublisherToSubscriptionIdsMap pub_to_subs_;
  SubscriptionMap subscriptions_;
  PublisherMap publishers_;

  mutable std::shared_timed_mutex mutex_;
};

namespace detail
{
/**
 * Helper function to expose method of IntraProcessManager
 * for publishing messages with same data type.
 *
 * This is needed as the publisher of serialized message is not aware of the subscribed
 * data type. While the subscription has all needed information (MessageT, Allocator) to
 * cast and deserialize the message. The type information is forwarded by the helper function.
 */
template<
  typename MessageT,
  typename ROSMessageType,
  typename Alloc,
  typename Deleter = std::default_delete<MessageT>
>
void do_intra_process_publish_same_type(
  IntraProcessManager * intraprocess_manager,
  uint64_t intra_process_publisher_id,
  std::unique_ptr<MessageT, Deleter> message,
  typename rclcpp::allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator,
  const std::vector<uint64_t> & take_ownership_subscriptions,
  const std::vector<uint64_t> & take_shared_subscriptions)
{
  intraprocess_manager->template do_intra_process_publish_same_type<MessageT, ROSMessageType, Alloc,
    Deleter>(
    intra_process_publisher_id,
    std::move(message),
    allocator,
    take_ownership_subscriptions,
    take_shared_subscriptions);
}
}  // namespace detail

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_
