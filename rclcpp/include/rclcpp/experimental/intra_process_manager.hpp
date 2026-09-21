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

#include <algorithm>
#include <atomic>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include <typeinfo>

#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/ros_message_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/visibility_control.hpp"

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
  template<
    typename ROSMessageType,
    typename Alloc = std::allocator<ROSMessageType>
  >
  uint64_t
  add_subscription(
    const rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr & subscription)
  {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);

    uint64_t sub_id = IntraProcessManager::get_next_unique_id();

    subscriptions_.try_emplace(sub_id, subscription);

    // adds the subscription id to all the matchable publishers
    for (auto & pair : publishers_) {
      auto publisher = pair.second.weak_publisher.lock();
      if (!publisher) {
        continue;
      }
      if (can_communicate(publisher, subscription)) {
        uint64_t pub_id = pair.first;
        insert_sub_id_for_pub(sub_id, pub_id, subscription->use_take_shared_method());
        if (publisher->is_durability_transient_local() &&
          subscription->is_durability_transient_local())
        {
          do_transient_local_publish<ROSMessageType, Alloc>(
            pub_id, sub_id,
            subscription->use_take_shared_method());
        }
      }
    }

    return sub_id;
  }

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
   * If the publisher's durability is transient local, its buffer pointer should
   * be passed and the method will store it as well.
   *
   * In addition this generates a unique intra process id for the publisher.
   *
   * \param publisher publisher to be registered with the manager.
   * \param buffer publisher's buffer to be stored if its durability is transient local.
   * \return an unsigned 64-bit integer which is the publisher's unique id.
   */
  RCLCPP_PUBLIC
  uint64_t
  add_publisher(
    const rclcpp::PublisherBase::SharedPtr & publisher,
    const rclcpp::experimental::buffers::IntraProcessBufferBase::SharedPtr & buffer =
    rclcpp::experimental::buffers::IntraProcessBufferBase::SharedPtr());

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
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator,
    rmw_message_info_t * message_info_out = nullptr)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto pub_to_subs_it = pub_to_subs_.find(intra_process_publisher_id);
    if (pub_to_subs_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return;
    }

    auto publisher_it = publishers_.find(intra_process_publisher_id);
    if (publisher_it == publishers_.end()) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }
    auto publisher = publisher_it->second.weak_publisher.lock();
    if (!publisher) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }

    rmw_message_info_t message_info{};
    message_info.from_intra_process = true;
    message_info.publication_sequence_number = publisher_it->second.publication_sequence_number++;
    message_info.publisher_gid = publisher->get_gid();

    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
      message_info.source_timestamp = now;
      message_info.received_timestamp = now;
    }

    const auto & take_ownership_subscriptions = pub_to_subs_it->second.take_ownership_subscriptions;
    const auto & take_shared_subscriptions = pub_to_subs_it->second.take_shared_subscriptions;

    if (take_ownership_subscriptions.empty()) {
      // None of the buffers require ownership, so we promote the pointer
      std::shared_ptr<MessageT> msg = std::move(message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        msg,
        message_info,
        take_shared_subscriptions);
    } else if (!take_ownership_subscriptions.empty() && // NOLINT
      take_shared_subscriptions.size() <= 1)
    {
      // There is at maximum 1 buffer that does not require ownership.
      // So this case is equivalent to all the buffers requiring ownership

      // Merge the two vector of ids into a unique one
      std::vector<uint64_t> concatenated_vector(
        take_shared_subscriptions.begin(), take_shared_subscriptions.end());
      concatenated_vector.insert(
        concatenated_vector.end(),
        take_ownership_subscriptions.begin(),
        take_ownership_subscriptions.end());
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        std::move(message),
        message_info,
        concatenated_vector,
        allocator);
    } else if (!take_ownership_subscriptions.empty() && // NOLINT
      take_shared_subscriptions.size() > 1)
    {
      // Construct a new shared pointer from the message
      // for the buffers that do not require ownership
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        shared_msg,
        message_info,
        take_shared_subscriptions);
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
        std::move(message),
        message_info,
        take_ownership_subscriptions,
        allocator);
    }

    if (message_info_out) {
      *message_info_out = message_info;
    }
  }

  template<
    typename MessageT,
    typename ROSMessageType,
    typename Alloc,
    typename Deleter = std::default_delete<MessageT>
  >
  std::pair<std::shared_ptr<const MessageT>, rmw_message_info_t>
  do_intra_process_publish_and_return_shared(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto pub_to_subs_it = pub_to_subs_.find(intra_process_publisher_id);
    if (pub_to_subs_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return {};
    }

    auto publisher_it = publishers_.find(intra_process_publisher_id);
    if (publisher_it == publishers_.end()) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }
    auto publisher = publisher_it->second.weak_publisher.lock();
    if (!publisher) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }

    rmw_message_info_t message_info{};
    message_info.from_intra_process = true;
    message_info.publication_sequence_number = publisher_it->second.publication_sequence_number++;
    message_info.publisher_gid = publisher->get_gid();

    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
      message_info.source_timestamp = now;
      message_info.received_timestamp = now;
    }

    const auto & take_ownership_subscriptions = pub_to_subs_it->second.take_ownership_subscriptions;
    const auto & take_shared_subscriptions = pub_to_subs_it->second.take_shared_subscriptions;

    if (take_ownership_subscriptions.empty()) {
      // If there are no owning, just convert to shared.
      std::shared_ptr<MessageT> shared_msg = std::move(message);
      if (!take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          shared_msg,
          message_info,
          take_shared_subscriptions);
      }
      return {shared_msg, message_info};
    } else {
      // Construct a new shared pointer from the message for the buffers that
      // do not require ownership and to return.
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      if (!take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          shared_msg,
          message_info,
          take_shared_subscriptions);
      }
      if (!take_ownership_subscriptions.empty()) {
        this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          std::move(message),
          message_info,
          take_ownership_subscriptions,
          allocator);
      }
      return {shared_msg, message_info};
    }
  }

  template<
    typename MessageT,
    typename Alloc,
    typename Deleter,
    typename ROSMessageType>
  void
  add_shared_msg_to_buffer(
    std::shared_ptr<const MessageT> message,
    const rmw_message_info_t & message_info,
    uint64_t subscription_id)
  {
    add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(message, message_info,
      {subscription_id});
  }

  template<
    typename MessageT,
    typename Alloc,
    typename Deleter,
    typename ROSMessageType>
  void
  add_owned_msg_to_buffer(
    std::unique_ptr<MessageT, Deleter> message,
    const rmw_message_info_t & message_info,
    uint64_t subscription_id,
    typename allocator::AllocRebind<MessageT, Alloc>::allocator_type & allocator)
  {
    add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
      std::move(message), message_info, {subscription_id}, allocator);
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
  struct SubscriptionData
  {
    explicit SubscriptionData(
      rclcpp::experimental::SubscriptionIntraProcessBase::WeakPtr subscription)
    : weak_subscription(std::move(subscription))
    {}

    rclcpp::experimental::SubscriptionIntraProcessBase::WeakPtr weak_subscription;
    // Incremented from do_intra_process_publish(), which only holds a shared (reader) lock on
    // mutex_, so concurrent publishers delivering to this subscription need this to be atomic.
    std::atomic<uint64_t> reception_sequence_number{0};
  };

  struct PublisherData
  {
    explicit PublisherData(rclcpp::PublisherBase::WeakPtr publisher)
    : weak_publisher(std::move(publisher))
    {}

    rclcpp::PublisherBase::WeakPtr weak_publisher;
    // Incremented from do_intra_process_publish(), which only holds a shared (reader) lock on
    // mutex_, so concurrent publishes from this same publisher need this to be atomic.
    std::atomic<uint64_t> publication_sequence_number{0};
  };

  struct SplittedSubscriptions
  {
    std::vector<uint64_t> take_shared_subscriptions;
    std::vector<uint64_t> take_ownership_subscriptions;
  };

  /// Hash function for rmw_gid_t to enable use in unordered_map
  struct rmw_gid_hash
  {
    std::size_t operator()(const rmw_gid_t & gid) const noexcept
    {
      // Using the FNV-1a hash algorithm on the gid data
      constexpr std::size_t FNV_prime = 1099511628211u;
      std::size_t result = 14695981039346656037u;

      for (std::size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
        result ^= gid.data[i];
        result *= FNV_prime;
      }
      return result;
    }
  };

  /// Equality comparison for rmw_gid_t to enable use in unordered_map
  struct rmw_gid_equal
  {
    bool operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const noexcept
    {
      // Compare the data bytes only.
      // implementation_identifier pointer comparison is not used here because
      // intra-process communication is always within the same process and RMW,
      // and pointer comparison is fragile across dynamically loaded components.
      return std::equal(
        std::begin(lhs.data),
        std::end(lhs.data),
        std::begin(rhs.data));
    }
  };

  using SubscriptionMap =
    std::unordered_map<uint64_t, SubscriptionData>;

  using PublisherMap =
    std::unordered_map<uint64_t, PublisherData>;

  using PublisherBufferMap =
    std::unordered_map<uint64_t, rclcpp::experimental::buffers::IntraProcessBufferBase::WeakPtr>;

  using PublisherToSubscriptionIdsMap =
    std::unordered_map<uint64_t, SplittedSubscriptions>;

  /// Structure to store publisher information in GID lookup map
  struct PublisherInfo
  {
    uint64_t pub_id;
    rclcpp::PublisherBase::WeakPtr publisher;
  };

  using GidToPublisherInfoMap =
    std::unordered_map<rmw_gid_t, PublisherInfo, rmw_gid_hash, rmw_gid_equal>;

  RCLCPP_PUBLIC
  static
  uint64_t
  get_next_unique_id();

  RCLCPP_PUBLIC
  void
  insert_sub_id_for_pub(uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method);

  RCLCPP_PUBLIC
  bool
  can_communicate(
    const rclcpp::PublisherBase::SharedPtr & pub,
    const rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr & sub) const;

  template<
    typename ROSMessageType,
    typename Alloc = std::allocator<ROSMessageType>
  >
  void do_transient_local_publish(
    const uint64_t pub_id, const uint64_t sub_id,
    const bool use_take_shared_method)
  {
    using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
    using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
    using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;
    using BufferType = rclcpp::experimental::buffers::IntraProcessBuffer<
      ROSMessageType,
      ROSMessageTypeDeleter
    >;
    using ROSMessageSharedPtr = typename BufferType::Data::MessageSharedPtr;
    using ROSMessageUniquePtr = typename BufferType::Data::MessageUniquePtr;

    auto publisher_buffer = publisher_buffers_[pub_id].lock();
    if (!publisher_buffer) {
      throw std::runtime_error("publisher buffer has unexpectedly gone out of scope");
    }
    auto buffer = std::dynamic_pointer_cast<BufferType>(publisher_buffer);
    if (!buffer) {
      throw std::runtime_error(
              "failed to dynamic cast publisher's IntraProcessBufferBase to "
              "IntraProcessBuffer<ROSMessageType,ROSMessageTypeAllocator,"
              "ROSMessageTypeDeleter> which can happen when the publisher and "
              "subscription use different allocator types, which is not supported");
    }
    auto data_vec = buffer->get_all_data();
    for (auto & data : data_vec) {
      // The buffer's own storage (shared vs unique) is independent of what this
      // particular new subscription requests, so the variant's actual alternative may
      // not match use_take_shared_method: convert as needed.
      if (use_take_shared_method) {
        ROSMessageSharedPtr shared_ptr;

        std::visit(
          [&shared_ptr](auto && message) {
            using T = std::decay_t<decltype(message)>;
            if constexpr (std::is_same_v<T, ROSMessageSharedPtr>) {
              shared_ptr = message;
            } else if constexpr (std::is_same_v<T, ROSMessageUniquePtr>) {
              auto allocator = ROSMessageTypeAllocator();
              shared_ptr = std::allocate_shared<ROSMessageType, ROSMessageTypeAllocator>(
                allocator, *message);
            }
          }, data.message);

        this->template add_shared_msg_to_buffer<
          ROSMessageType, ROSMessageTypeAllocator, ROSMessageTypeDeleter, ROSMessageType>(
          shared_ptr, data.message_info, sub_id);
      } else {
        ROSMessageUniquePtr unique_ptr;
        auto allocator = ROSMessageTypeAllocator();

        std::visit(
          [&unique_ptr, &allocator](auto && message) {
            ROSMessageTypeDeleter deleter;
            auto ptr = ROSMessageTypeAllocatorTraits::allocate(allocator, 1);
            ROSMessageTypeAllocatorTraits::construct(allocator, ptr, *message);

            using T = std::decay_t<decltype(message)>;
            if constexpr (std::is_same_v<T, ROSMessageSharedPtr>) {
              allocator::set_allocator_for_deleter(&deleter, &allocator);
            } else if constexpr (std::is_same_v<T, ROSMessageUniquePtr>) {
              deleter = message.get_deleter();
            }

            unique_ptr = ROSMessageUniquePtr(ptr, deleter);
          }, data.message);

        this->template add_owned_msg_to_buffer<
          ROSMessageType, ROSMessageTypeAllocator, ROSMessageTypeDeleter, ROSMessageType>(
          std::move(unique_ptr), data.message_info, sub_id, allocator);
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
    rmw_message_info_t message_info,
    const std::vector<uint64_t> & subscription_ids)
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
      auto subscription_base = subscription_it->second.weak_subscription.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(id);
        continue;
      }

      message_info.reception_sequence_number = subscription_it->second.reception_sequence_number++;

      auto subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionIntraProcessBuffer<PublishedType,
        PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>
        >(subscription_base);
      if (subscription != nullptr) {
        subscription->provide_intra_process_data(message, message_info);
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
          std::make_shared<ROSMessageType>(ros_msg), message_info);
      } else {
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          ros_message_subscription->provide_intra_process_message(message,
            message_info);
        } else {
          if constexpr (std::is_same<typename rclcpp::TypeAdapter<MessageT,
            ROSMessageType>::ros_message_type, ROSMessageType>::value)
          {
            ROSMessageType ros_msg;
            rclcpp::TypeAdapter<MessageT, ROSMessageType>::convert_to_ros_message(
              *message, ros_msg);
            ros_message_subscription->provide_intra_process_message(
              std::make_shared<ROSMessageType>(ros_msg), message_info);
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
    rmw_message_info_t message_info,
    const std::vector<uint64_t> & subscription_ids,
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
      auto subscription_base = subscription_it->second.weak_subscription.lock();
      if (subscription_base == nullptr) {
        subscriptions_.erase(subscription_it);
        continue;
      }

      message_info.reception_sequence_number = subscription_it->second.reception_sequence_number++;

      auto subscription = std::dynamic_pointer_cast<
        rclcpp::experimental::SubscriptionIntraProcessBuffer<PublishedType,
        PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>
        >(subscription_base);
      if (subscription != nullptr) {
        if (std::next(it) == subscription_ids.end()) {
          // If this is the last subscription, give up ownership
          subscription->provide_intra_process_data(std::move(message), message_info);
          // Last message delivered, break from for loop
          break;
        } else {
          // Copy the message since we have additional subscriptions to serve
          Deleter deleter = message.get_deleter();
          auto ptr = MessageAllocTraits::allocate(allocator, 1);
          MessageAllocTraits::construct(allocator, ptr, *message);

          subscription->provide_intra_process_data(MessageUniquePtr(ptr, deleter),
            message_info);
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
        ros_message_subscription->provide_intra_process_message(std::move(ros_msg),
          message_info);
      } else {
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          if (std::next(it) == subscription_ids.end()) {
            // If this is the last subscription, give up ownership
            ros_message_subscription->provide_intra_process_message(std::move(message),
              message_info);
            // Last message delivered, break from for loop
            break;
          } else {
            // Copy the message since we have additional subscriptions to serve
            Deleter deleter = message.get_deleter();
            allocator::set_allocator_for_deleter(&deleter, &allocator);
            auto ptr = MessageAllocTraits::allocate(allocator, 1);
            MessageAllocTraits::construct(allocator, ptr, *message);

            ros_message_subscription->provide_intra_process_message(
              MessageUniquePtr(ptr, deleter), message_info);
          }
        }
      }
    }
  }

  PublisherToSubscriptionIdsMap pub_to_subs_;
  SubscriptionMap subscriptions_;
  PublisherMap publishers_;
  PublisherBufferMap publisher_buffers_;

  mutable std::shared_timed_mutex mutex_;

  GidToPublisherInfoMap gid_to_publisher_info_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_
