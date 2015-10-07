// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_INTRA_PROCESS_MANAGER_HPP_
#define RCLCPP_RCLCPP_INTRA_PROCESS_MANAGER_HPP_

#include <rclcpp/mapped_ring_buffer.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <unordered_map>
#include <set>

#include <rmw/types.h>

namespace rclcpp
{
namespace intra_process_manager
{

/// This class facilitates intra process communication between nodes.
/* This class is used in the creation of publishers and subscriptions.
 * A singleton instance of this class is owned by a rclcpp::Context and a
 * rclcpp::Node can use an associated Context to get an instance of this class.
 * Nodes which do not have a common Context will not exchange intra process
 * messages because they will not share access to an instance of this class.
 *
 * When a Node creates a publisher or subscription, it will register them
 * with this class.
 * The node will also hook into the publisher's publish call
 * in order to do intra process related work.
 *
 * When a publisher is created, it advertises on the topic the user provided,
 * as well as a "shadowing" topic of type rcl_interfaces/IntraProcessMessage.
 * For instance, if the user specified the topic '/namespace/chatter', then the
 * corresponding intra process topic might be '/namespace/chatter__intra'.
 * The publisher is also allocated an id which is unique among all publishers
 * and subscriptions in this process.
 * Additionally, when registered with this class a ring buffer is created and
 * owned by this class as a temporary place to hold messages destined for intra
 * process subscriptions.
 *
 * When a subscription is created, it subscribes to the topic provided by the
 * user as well as to the corresponding intra process topic.
 * It is also gets a unique id from the singleton instance of this class which
 * is unique among publishers and subscriptions.
 *
 * When the user publishes a message, the message is stored by calling
 * store_intra_process_message on this class.
 * The instance of that message is uniquely identified by a publisher id and a
 * message sequence number.
 * The publisher id, message sequence pair is unique with in the process.
 * At that point a list of the id's of intra process subscriptions which have
 * been registered with the singleton instance of this class are stored with
 * the message instance so that delivery is only made to those subscriptions.
 * Then an instance of rcl_interfaces/IntraProcessMessage is published to the
 * intra process topic which is specific to the topic specified by the user.
 *
 * When an instance of rcl_interfaces/IntraProcessMessage is received by a
 * subscription, then it is handled by calling take_intra_process_message
 * on a singleton of this class.
 * The subscription passes a publisher id, message sequence pair which
 * uniquely identifies the message instance it was suppose to receive as well
 * as the subscriptions unique id.
 * If the message is still being held by this class and the subscription's id
 * is in the list of intended subscriptions then the message is returned.
 * If either of those predicates are not satisfied then the message is not
 * returned and the subscription does not call the users callback.
 *
 * Since the publisher builds a list of destined subscriptions on publish, and
 * other requests are ignored, this class knows how many times a message
 * instance should be requested.
 * The final time a message is requested, the ownership is passed out of this
 * class and passed to the final subscription, effectively freeing space in
 * this class's internal storage.
 *
 * Since a topic is being used to ferry notifications about new intra process
 * messages between publishers and subscriptions, it is possible for that
 * notification to be lost.
 * It is also possible that a subscription which was available when publish was
 * called will no longer exist once the notification gets posted.
 * In both cases this might result in a message instance getting requested
 * fewer times than expected.
 * This is why the internal storage of this class is a ring buffer.
 * That way if a message is orphaned it will eventually be dropped from storage
 * when a new message instance is stored and will not result in a memory leak.
 *
 * However, since the storage system is finite, this also means that a message
 * instance might get displaced by an incoming message instance before all
 * interested parties have called take_intra_process_message.
 * Because of this the size of the internal storage should be carefully
 * considered.
 *
 * /TODO(wjwwood): update to include information about handling latching.
 * /TODO(wjwwood): consider thread safety of the class.
 *
 * This class is neither CopyConstructable nor CopyAssignable.
 */
class IntraProcessManager
{
private:
  RCLCPP_DISABLE_COPY(IntraProcessManager);

public:
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessManager);

  IntraProcessManager() = default;

  /// Register a subscription with the manager, returns subscriptions unique id.
  /* In addition to generating a unique intra process id for the subscription,
   * this method also stores the topic name of the subscription.
   *
   * This method is normally called during the creation of a subscription,
   * but after it creates the internal intra process rmw_subscription_t.
   *
   * This method will allocate memory.
   *
   * \param subscription the Subscription to register.
   * \return an unsigned 64-bit integer which is the subscription's unique id.
   */
  uint64_t
  add_subscription(subscription::SubscriptionBase::SharedPtr subscription)
  {
    auto id = IntraProcessManager::get_next_unique_id();
    subscriptions_[id] = subscription;
    subscription_ids_by_topic_[subscription->get_topic_name()].insert(id);
    return id;
  }

  /// Unregister a subscription using the subscription's unique id.
  /* This method does not allocate memory.
   *
   * \param intra_process_subscription_id id of the subscription to remove.
   */
  void
  remove_subscription(uint64_t intra_process_subscription_id)
  {
    subscriptions_.erase(intra_process_subscription_id);
    for (auto & pair : subscription_ids_by_topic_) {
      pair.second.erase(intra_process_subscription_id);
    }
    // Iterate over all publisher infos and all stored subscription id's and
    // remove references to this subscription's id.
    for (auto & publisher_pair : publishers_) {
      for (auto & sub_pair : publisher_pair.second.target_subscriptions_by_message_sequence) {
        sub_pair.second.erase(intra_process_subscription_id);
      }
    }
  }

  /// Register a publisher with the manager, returns the publisher unique id.
  /* In addition to generating and returning a unique id for the publisher,
   * this method creates internal ring buffer storage for "in-flight" intra
   * process messages which are stored when store_intra_process_message is
   * called with this publisher's unique id.
   *
   * The buffer_size must be less than or equal to the max uint64_t value.
   * If the buffer_size is 0 then a buffer size is calculated using the
   * publisher's QoS settings.
   * The default is to use the depth field of the publisher's QoS.
   * TODO(wjwwood): Consider doing depth *= 1.2, round up, or similar.
   * TODO(wjwwood): Consider what to do for keep all.
   *
   * This method is templated on the publisher's message type so that internal
   * storage of the same type can be allocated.
   *
   * This method will allocate memory.
   *
   * \param publisher publisher to be registered with the manager.
   * \param buffer_size if 0 (default) a size is calculated based on the QoS.
   * \return an unsigned 64-bit integer which is the publisher's unique id.
   */
  template<typename MessageT, typename Allocator = std::allocator<MessageT>>
  uint64_t
  add_publisher(publisher::Publisher::SharedPtr publisher, size_t buffer_size = 0)
  {
    auto id = IntraProcessManager::get_next_unique_id();
    publishers_[id].publisher = publisher;
    size_t size = buffer_size > 0 ? buffer_size : publisher->get_queue_size();
    // As long as the size of the ring buffer is less than the max sequence number, we're safe.
    if (size > std::numeric_limits<uint64_t>::max()) {
      throw std::invalid_argument("the calculated buffer size is too large");
    }
    publishers_[id].sequence_number.store(0);
    publishers_[id].buffer = mapped_ring_buffer::MappedRingBuffer<MessageT, Allocator>::make_shared(size);
    publishers_[id].target_subscriptions_by_message_sequence.reserve(size);
    return id;
  }

  /// Unregister a publisher using the publisher's unique id.
  /* This method does not allocate memory.
   *
   * \param intra_process_publisher_id id of the publisher to remove.
   */
  void
  remove_publisher(uint64_t intra_process_publisher_id)
  {
    publishers_.erase(intra_process_publisher_id);
  }

  /// Store a message in the manager, and return the message sequence number.
  /* The given message is stored in internal storage using the given publisher
   * id and the newly generated message sequence, which is also returned.
   * The combination of publisher id and message sequence number can later
   * be used with a subscription id to retrieve the message by calling
   * take_intra_process_message.
   * The number of times take_intra_process_message can be called with this
   * unique pair of id's is determined by the number of subscriptions currently
   * subscribed to the same topic and which share the same Context, i.e. once
   * for each subscription which should receive the intra process message.
   *
   * The ownership of the incoming message is transfered to the internal
   * storage in order to avoid copying the message data.
   * Therefore, the message parameter will no longer contain the original
   * message after calling this method.
   * Instead it will either be a nullptr or it will contain the ownership of
   * the message instance which was displaced.
   * If the message parameter is not equal to nullptr after calling this method
   * then a message was prematurely displaced, i.e. take_intra_process_message
   * had not been called on it as many times as was expected.
   *
   * This method can throw an exception if the publisher id is not found or
   * if the publisher shared_ptr given to add_publisher has gone out of scope.
   *
   * This method does allocate memory.
   *
   * \param intra_process_publisher_id the id of the publisher of this message.
   * \param message the message that is being stored.
   * \return the message sequence number.
   */
  template<typename MessageT, typename Allocator = std::allocator<MessageT>>
  uint64_t
  store_intra_process_message(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT> & message)
  {
    auto it = publishers_.find(intra_process_publisher_id);
    if (it == publishers_.end()) {
      throw std::runtime_error("store_intra_process_message called with invalid publisher id");
    }
    PublisherInfo & info = it->second;
    // Calculate the next message sequence number.
    uint64_t message_seq = info.sequence_number.fetch_add(1, std::memory_order_relaxed);
    // Insert the message into the ring buffer using the message_seq to identify it.
    typedef typename mapped_ring_buffer::MappedRingBuffer<MessageT, Allocator> TypedMRB;
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(info.buffer);
    bool did_replace = typed_buffer->push_and_replace(message_seq, message);
    // TODO(wjwwood): do something when a message was displaced. log debug?
    (void)did_replace;  // Avoid unused variable warning.
    // Figure out what subscriptions should receive the message.
    auto publisher = info.publisher.lock();
    if (!publisher) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }
    auto & destined_subscriptions = subscription_ids_by_topic_[publisher->get_topic_name()];
    // Store the list for later comparison.
    info.target_subscriptions_by_message_sequence[message_seq].clear();
    std::copy(
      destined_subscriptions.begin(), destined_subscriptions.end(),
      // Memory allocation occurs in info.target_subscriptions_by_message_sequence[message_seq]
      std::inserter(
        info.target_subscriptions_by_message_sequence[message_seq],
        // This ends up only being a hint to std::set, could also be .begin().
        info.target_subscriptions_by_message_sequence[message_seq].end()
      )
    );
    // Return the message sequence which is sent to the subscription.
    return message_seq;
  }

  /// Take an intra process message.
  /* The intra_process_publisher_id and message_sequence_number parameters
   * uniquely identify a message instance, which should be taken.
   *
   * The requesting_subscriptions_intra_process_id parameter is used to make
   * sure the requesting subscription was intended to receive this message
   * instance.
   * This check is made because it could happen that the requester
   * comes up after the publish event, so it still receives the notification of
   * a new intra process message, but it wasn't registered with the manager at
   * the time of publishing, causing it to take when it wasn't intended.
   * This should be avioded unless latching-like behavior is involved.
   *
   * The message parameter is used to store the taken message.
   * On the last expected call to this method, the ownership is transfered out
   * of internal storage and into the message parameter.
   * On all previous calls a copy of the internally stored message is made and
   * the ownership of the copy is transfered to the message parameter.
   * TODO(wjwwood): update this documentation when latching is supported.
   *
   * The message parameter can be set to nullptr if:
   *
   * - The publisher id is not found.
   * - The message sequence is not found for the given publisher id.
   * - The requesting subscription's id is not in the list of intended takers.
   * - The requesting subscription's id has been used before with this message.
   *
   * This method may allocate memory to copy the stored message.
   *
   * \param intra_process_publisher_id the id of the message's publisher.
   * \param message_sequence_number the sequence number of the message.
   * \param requesting_subscriptions_intra_process_id the subscription's id.
   * \param message the message typed unique_ptr used to return the message.
   */
  template<typename MessageT, typename Allocator = std::allocator<MessageT>>
  void
  take_intra_process_message(
    uint64_t intra_process_publisher_id,
    uint64_t message_sequence_number,
    uint64_t requesting_subscriptions_intra_process_id,
    std::unique_ptr<MessageT> & message)
  {
    message = nullptr;
    PublisherInfo * info;
    {
      auto it = publishers_.find(intra_process_publisher_id);
      if (it == publishers_.end()) {
        // Publisher is either invalid or no longer exists.
        return;
      }
      info = &it->second;
    }
    // Figure out how many subscriptions are left.
    std::set<uint64_t> * target_subs;
    {
      auto it = info->target_subscriptions_by_message_sequence.find(message_sequence_number);
      if (it == info->target_subscriptions_by_message_sequence.end()) {
        // Message is no longer being stored by this publisher.
        return;
      }
      target_subs = &it->second;
    }
    {
      auto it = std::find(
        target_subs->begin(), target_subs->end(),
        requesting_subscriptions_intra_process_id);
      if (it == target_subs->end()) {
        // This publisher id/message seq pair was not intended for this subscription.
        return;
      }
      target_subs->erase(it);
    }
    typedef typename mapped_ring_buffer::MappedRingBuffer<MessageT, Allocator> TypedMRB;
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(info->buffer);
    // Return a copy or the unique_ptr (ownership) depending on how many subscriptions are left.
    if (target_subs->size()) {
      // There are more subscriptions to serve, return a copy.
      typed_buffer->get_copy_at_key(message_sequence_number, message);
    } else {
      // This is the last one to be returned, transfer ownership.
      typed_buffer->pop_at_key(message_sequence_number, message);
    }
  }

  /// Return true if the given rmw_gid_t matches any stored Publishers.
  bool
  matches_any_publishers(const rmw_gid_t * id) const
  {
    for (auto & publisher_pair : publishers_) {
      auto publisher = publisher_pair.second.publisher.lock();
      if (!publisher) {
        continue;
      }
      if (*publisher.get() == id) {
        return true;
      }
    }
    return false;
  }

private:
  static uint64_t get_next_unique_id()
  {
    auto next_id = next_unique_id_.fetch_add(1, std::memory_order_relaxed);
    // Check for rollover (we started at 1).
    if (0 == next_id) {
      // This puts a technical limit on the number of times you can add a publisher or subscriber.
      // But even if you could add (and remove) them at 1 kHz (very optimistic rate)
      // it would still be a very long time before you could exhaust the pool of id's:
      //   2^64 / 1000 times per sec / 60 sec / 60 min / 24 hours / 365 days = 584,942,417 years
      // So around 585 million years. Even at 1 GHz, it would take 585 years.
      // I think it's safe to avoid trying to handle overflow.
      // If we roll over then it's most likely a bug.
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::overflow_error(
        "exhausted the unique id's for publishers and subscribers in this process "
        "(congratulations your computer is either extremely fast or extremely old)");
      // *INDENT-ON*
    }
    return next_id;
  }

  static std::atomic<uint64_t> next_unique_id_;

  std::unordered_map<uint64_t, subscription::SubscriptionBase::WeakPtr> subscriptions_;
  std::map<std::string, std::set<uint64_t>> subscription_ids_by_topic_;

  struct PublisherInfo
  {
    RCLCPP_DISABLE_COPY(PublisherInfo);

    PublisherInfo() = default;

    publisher::Publisher::WeakPtr publisher;
    std::atomic<uint64_t> sequence_number;
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer;
    std::unordered_map<uint64_t, std::set<uint64_t>> target_subscriptions_by_message_sequence;
  };

  std::unordered_map<uint64_t, PublisherInfo> publishers_;

};

std::atomic<uint64_t> IntraProcessManager::next_unique_id_ {1};

} /* namespace intra_process_manager */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_INTRA_PROCESS_MANAGER_HPP_ */
