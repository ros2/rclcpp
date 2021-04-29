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
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
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
   */
  template<
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
  void
  do_intra_process_publish(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    std::shared_ptr<typename allocator::AllocRebind<MessageT, Alloc>::allocator_type> allocator)
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
    const auto & sub_ids = publisher_it->second;

    if (sub_ids.take_ownership_subscriptions.empty()) {
      // None of the buffers require ownership, so we promote the pointer
      std::shared_ptr<MessageT> msg = std::move(message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter>(
        msg, sub_ids.take_shared_subscriptions);
    } else if (!sub_ids.take_ownership_subscriptions.empty() && // NOLINT
      sub_ids.take_shared_subscriptions.size() <= 1)
    {
      // There is at maximum 1 buffer that does not require ownership.
      // So this case is equivalent to all the buffers requiring ownership

      // Merge the two vector of ids into a unique one
      std::vector<uint64_t> concatenated_vector(sub_ids.take_shared_subscriptions);
      concatenated_vector.insert(
        concatenated_vector.end(),
        sub_ids.take_ownership_subscriptions.begin(),
        sub_ids.take_ownership_subscriptions.end());

      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter>(
        std::move(message),
        concatenated_vector,
        allocator);
    } else if (!sub_ids.take_ownership_subscriptions.empty() && // NOLINT
      sub_ids.take_shared_subscriptions.size() > 1)
    {
      // Construct a new shared pointer from the message
      // for the buffers that do not require ownership
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(*allocator, *message);

      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter>(
        shared_msg, sub_ids.take_shared_subscriptions);
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter>(
        std::move(message), sub_ids.take_ownership_subscriptions, allocator);
    }
  }

  template<
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
  std::shared_ptr<const MessageT>
  do_intra_process_publish_and_return_shared(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message,
    std::shared_ptr<typename allocator::AllocRebind<MessageT, Alloc>::allocator_type> allocator)
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
    const auto & sub_ids = publisher_it->second;

    if (sub_ids.take_ownership_subscriptions.empty()) {
      // If there are no owning, just convert to shared.
      std::shared_ptr<MessageT> shared_msg = std::move(message);
      if (!sub_ids.take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter>(
          shared_msg, sub_ids.take_shared_subscriptions);
      }
      return shared_msg;
    } else {
      // Construct a new shared pointer from the message for the buffers that
      // do not require ownership and to return.
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(*allocator, *message);

      if (!sub_ids.take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter>(
          shared_msg,
          sub_ids.take_shared_subscriptions);
      }
      if (!sub_ids.take_ownership_subscriptions.empty()) {
        this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter>(
          std::move(message),
          sub_ids.take_ownership_subscriptions,
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

private:
  struct SubscriptionInfo
  {
    SubscriptionInfo() = default;

    rclcpp::experimental::SubscriptionIntraProcessBase::WeakPtr subscription;
    rmw_qos_profile_t qos;
    const char * topic_name;
    bool use_take_shared_method;
  };

  struct PublisherInfo
  {
    PublisherInfo() = default;

    rclcpp::PublisherBase::WeakPtr publisher;
    rmw_qos_profile_t qos;
    const char * topic_name;
  };

  struct SplittedSubscriptions
  {
    std::vector<uint64_t> take_shared_subscriptions;
    std::vector<uint64_t> take_ownership_subscriptions;
  };

  using SubscriptionMap =
    std::unordered_map<uint64_t, SubscriptionInfo>;

  using PublisherMap =
    std::unordered_map<uint64_t, PublisherInfo>;

  using PublisherToSubscriptionIdsMap =
    std::unordered_map<uint64_t, SplittedSubscriptions>;

  RCLCPP_PUBLIC
  static
  uint64_t
  get_next_unique_id();

  RCLCPP_PUBLIC
  void
  insert_sub_id_for_pub(uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method);

  RCLCPP_PUBLIC
  bool
  can_communicate(PublisherInfo pub_info, SubscriptionInfo sub_info) const;

  template<
    typename MessageT,
    typename Alloc,
    typename Deleter>
  void
  add_shared_msg_to_buffers(
    std::shared_ptr<const MessageT> message,
    std::vector<uint64_t> subscription_ids)
  {
    for (auto id : subscription_ids) {
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.subscription.lock();
      if (subscription_base) {
        auto subscription = std::dynamic_pointer_cast<
          rclcpp::experimental::SubscriptionIntraProcess<MessageT, Alloc, Deleter>
          >(subscription_base);
        if (nullptr == subscription) {
          throw std::runtime_error(
                  "failed to dynamic cast SubscriptionIntraProcessBase to "
                  "SubscriptionIntraProcess<MessageT, Alloc, Deleter>, which "
                  "can happen when the publisher and subscription use different "
                  "allocator types, which is not supported");
        }

        subscription->provide_intra_process_message(message);
      } else {
        subscriptions_.erase(id);
      }
    }
  }

  template<
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
  void
  add_owned_msg_to_buffers(
    std::unique_ptr<MessageT, Deleter> message,
    std::vector<uint64_t> subscription_ids,
    std::shared_ptr<typename allocator::AllocRebind<MessageT, Alloc>::allocator_type> allocator)
  {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;

    for (auto it = subscription_ids.begin(); it != subscription_ids.end(); it++) {
      auto subscription_it = subscriptions_.find(*it);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.subscription.lock();
      if (subscription_base) {
        auto subscription = std::dynamic_pointer_cast<
          rclcpp::experimental::SubscriptionIntraProcess<MessageT, Alloc, Deleter>
          >(subscription_base);
        if (nullptr == subscription) {
          throw std::runtime_error(
                  "failed to dynamic cast SubscriptionIntraProcessBase to "
                  "SubscriptionIntraProcess<MessageT, Alloc, Deleter>, which "
                  "can happen when the publisher and subscription use different "
                  "allocator types, which is not supported");
        }

        if (std::next(it) == subscription_ids.end()) {
          // If this is the last subscription, give up ownership
          subscription->provide_intra_process_message(std::move(message));
        } else {
          // Copy the message since we have additional subscriptions to serve
          MessageUniquePtr copy_message;
          Deleter deleter = message.get_deleter();
          auto ptr = MessageAllocTraits::allocate(*allocator.get(), 1);
          MessageAllocTraits::construct(*allocator.get(), ptr, *message);
          copy_message = MessageUniquePtr(ptr, deleter);

          subscription->provide_intra_process_message(std::move(copy_message));
        }
      } else {
        subscriptions_.erase(subscription_it);
      }
    }
  }

  PublisherToSubscriptionIdsMap pub_to_subs_;
  SubscriptionMap subscriptions_;
  PublisherMap publishers_;

  mutable std::shared_timed_mutex mutex_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_
