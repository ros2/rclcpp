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

#ifndef RCLCPP__INTRA_PROCESS_MANAGER_HPP_
#define RCLCPP__INTRA_PROCESS_MANAGER_HPP_

#include <rmw/types.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <set>

#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/intra_process_manager_impl.hpp"
#include "rclcpp/mapped_ring_buffer.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace intra_process_manager
{

/// This class facilitates intra process communication between nodes.
/**
 * This class is used in the creation of publishers and subscriptions.
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
 * corresponding intra process topic might be '/namespace/chatter/_intra'.
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
  RCLCPP_DISABLE_COPY(IntraProcessManager)

public:
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessManager)

  RCLCPP_PUBLIC
  explicit IntraProcessManager(
    IntraProcessManagerImplBase::SharedPtr state = create_default_impl());

  RCLCPP_PUBLIC
  virtual ~IntraProcessManager();

  /// Register a subscription with the manager, returns subscriptions unique id.
  /**
   * In addition to generating a unique intra process id for the subscription,
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
  RCLCPP_PUBLIC
  uint64_t
  add_subscription(SubscriptionBase::SharedPtr subscription);

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
   * In addition to generating and returning a unique id for the publisher,
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
  RCLCPP_PUBLIC
  uint64_t
  add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher,
    size_t buffer_size = 0);

  /// Unregister a publisher using the publisher's unique id.
  /**
   * This method does not allocate memory.
   *
   * \param intra_process_publisher_id id of the publisher to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_publisher(uint64_t intra_process_publisher_id);

  /// Store a message in the manager, and return the message sequence number.
  /**
   * The given message is stored in internal storage using the given publisher
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
  template<
    typename MessageT, typename Alloc = std::allocator<void>>
  uint64_t
  store_intra_process_message(
    uint64_t intra_process_publisher_id,
    std::shared_ptr<const MessageT> message)
  {
    using MRBMessageAlloc = typename std::allocator_traits<Alloc>::template rebind_alloc<MessageT>;
    using TypedMRB = typename mapped_ring_buffer::MappedRingBuffer<MessageT, MRBMessageAlloc>;
    uint64_t message_seq = 0;
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer = impl_->get_publisher_info_for_id(
      intra_process_publisher_id, message_seq);
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(buffer);
    if (!typed_buffer) {
      throw std::runtime_error("Typecast failed due to incorrect message type");
    }

    // Insert the message into the ring buffer using the message_seq to identify it.
    bool did_replace = typed_buffer->push_and_replace(message_seq, message);
    // TODO(wjwwood): do something when a message was displaced. log debug?
    (void)did_replace;  // Avoid unused variable warning.

    impl_->store_intra_process_message(intra_process_publisher_id, message_seq);

    // Return the message sequence which is sent to the subscription.
    return message_seq;
  }

  template<
    typename MessageT, typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
  uint64_t
  store_intra_process_message(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> message)
  {
    using MRBMessageAlloc = typename std::allocator_traits<Alloc>::template rebind_alloc<MessageT>;
    using TypedMRB = typename mapped_ring_buffer::MappedRingBuffer<MessageT, MRBMessageAlloc>;
    uint64_t message_seq = 0;
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer = impl_->get_publisher_info_for_id(
      intra_process_publisher_id, message_seq);
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(buffer);
    if (!typed_buffer) {
      throw std::runtime_error("Typecast failed due to incorrect message type");
    }

    // Insert the message into the ring buffer using the message_seq to identify it.
    bool did_replace = typed_buffer->push_and_replace(message_seq, std::move(message));
    // TODO(wjwwood): do something when a message was displaced. log debug?
    (void)did_replace;  // Avoid unused variable warning.

    impl_->store_intra_process_message(intra_process_publisher_id, message_seq);

    // Return the message sequence which is sent to the subscription.
    return message_seq;
  }

  /// Take an intra process message.
  /**
   * The intra_process_publisher_id and message_sequence_number parameters
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
  template<
    typename MessageT, typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
  void
  take_intra_process_message(
    uint64_t intra_process_publisher_id,
    uint64_t message_sequence_number,
    uint64_t requesting_subscriptions_intra_process_id,
    std::unique_ptr<MessageT, Deleter> & message)
  {
    using MRBMessageAlloc = typename std::allocator_traits<Alloc>::template rebind_alloc<MessageT>;
    using TypedMRB = mapped_ring_buffer::MappedRingBuffer<MessageT, MRBMessageAlloc>;
    message = nullptr;

    size_t target_subs_size = 0;
    std::lock_guard<std::mutex> lock(take_mutex_);
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer = impl_->take_intra_process_message(
      intra_process_publisher_id,
      message_sequence_number,
      requesting_subscriptions_intra_process_id,
      target_subs_size
    );
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(buffer);
    if (!typed_buffer) {
      return;
    }
    // Return a copy or the unique_ptr (ownership) depending on how many subscriptions are left.
    if (target_subs_size) {
      // There are more subscriptions to serve, return a copy.
      typed_buffer->get(message_sequence_number, message);
    } else {
      // This is the last one to be returned, transfer ownership.
      typed_buffer->pop(message_sequence_number, message);
    }
  }

  template<
    typename MessageT, typename Alloc = std::allocator<void>>
  void
  take_intra_process_message(
    uint64_t intra_process_publisher_id,
    uint64_t message_sequence_number,
    uint64_t requesting_subscriptions_intra_process_id,
    std::shared_ptr<const MessageT> & message)
  {
    using MRBMessageAlloc = typename std::allocator_traits<Alloc>::template rebind_alloc<MessageT>;
    using TypedMRB = mapped_ring_buffer::MappedRingBuffer<MessageT, MRBMessageAlloc>;
    message = nullptr;

    size_t target_subs_size = 0;
    std::lock_guard<std::mutex> lock(take_mutex_);
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer = impl_->take_intra_process_message(
      intra_process_publisher_id,
      message_sequence_number,
      requesting_subscriptions_intra_process_id,
      target_subs_size
    );
    typename TypedMRB::SharedPtr typed_buffer = std::static_pointer_cast<TypedMRB>(buffer);
    if (!typed_buffer) {
      return;
    }
    // Return a copy or the unique_ptr (ownership) depending on how many subscriptions are left.
    if (target_subs_size) {
      // There are more subscriptions to serve, return a copy.
      typed_buffer->get(message_sequence_number, message);
    } else {
      // This is the last one to be returned, transfer ownership.
      typed_buffer->pop(message_sequence_number, message);
    }
  }

  /// Return true if the given rmw_gid_t matches any stored Publishers.
  RCLCPP_PUBLIC
  bool
  matches_any_publishers(const rmw_gid_t * id) const;

  /// Return the number of intraprocess subscriptions to a topic, given the publisher id.
  RCLCPP_PUBLIC
  size_t
  get_subscription_count(uint64_t intra_process_publisher_id) const;

private:
  RCLCPP_PUBLIC
  static uint64_t
  get_next_unique_id();

  IntraProcessManagerImplBase::SharedPtr impl_;
  std::mutex take_mutex_;
};

}  // namespace intra_process_manager
}  // namespace rclcpp

#endif  // RCLCPP__INTRA_PROCESS_MANAGER_HPP_
