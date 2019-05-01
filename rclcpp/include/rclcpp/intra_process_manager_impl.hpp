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

#ifndef RCLCPP__INTRA_PROCESS_MANAGER_IMPL_HPP_
#define RCLCPP__INTRA_PROCESS_MANAGER_IMPL_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "rmw/validate_full_topic_name.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/mapped_ring_buffer.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace intra_process_manager
{

class IntraProcessManagerImplBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(IntraProcessManagerImplBase)

  IntraProcessManagerImplBase() = default;
  virtual ~IntraProcessManagerImplBase() = default;

  virtual void
  add_subscription(uint64_t id, SubscriptionBase::SharedPtr subscription) = 0;

  virtual void
  remove_subscription(uint64_t intra_process_subscription_id) = 0;

  virtual void add_publisher(
    uint64_t id,
    PublisherBase::WeakPtr publisher,
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr mrb,
    size_t size) = 0;

  virtual void
  remove_publisher(uint64_t intra_process_publisher_id) = 0;

  virtual mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  get_publisher_info_for_id(
    uint64_t intra_process_publisher_id,
    uint64_t & message_seq) = 0;

  virtual void
  store_intra_process_message(uint64_t intra_process_publisher_id, uint64_t message_seq) = 0;

  virtual mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  take_intra_process_message(
    uint64_t intra_process_publisher_id,
    uint64_t message_sequence_number,
    uint64_t requesting_subscriptions_intra_process_id,
    size_t & size) = 0;

  virtual bool
  matches_any_publishers(const rmw_gid_t * id) const = 0;

  virtual size_t
  get_subscription_count(uint64_t intra_process_publisher_id) const = 0;

private:
  RCLCPP_DISABLE_COPY(IntraProcessManagerImplBase)
};

template<typename Allocator = std::allocator<void>>
class IntraProcessManagerImpl : public IntraProcessManagerImplBase
{
public:
  IntraProcessManagerImpl() = default;
  ~IntraProcessManagerImpl() = default;

  void
  add_subscription(uint64_t id, SubscriptionBase::SharedPtr subscription)
  {
    subscriptions_[id] = subscription;
    subscription_ids_by_topic_[fixed_size_string(subscription->get_topic_name())].insert(id);
  }

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

  void add_publisher(
    uint64_t id,
    PublisherBase::WeakPtr publisher,
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr mrb,
    size_t size)
  {
    publishers_[id].publisher = publisher;
    // As long as the size of the ring buffer is less than the max sequence number, we're safe.
    if (size > std::numeric_limits<uint64_t>::max()) {
      throw std::invalid_argument("the calculated buffer size is too large");
    }
    publishers_[id].sequence_number.store(0);

    publishers_[id].buffer = mrb;
    publishers_[id].target_subscriptions_by_message_sequence.reserve(size);
  }

  void
  remove_publisher(uint64_t intra_process_publisher_id)
  {
    publishers_.erase(intra_process_publisher_id);
  }

  // return message_seq and mrb
  mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  get_publisher_info_for_id(
    uint64_t intra_process_publisher_id,
    uint64_t & message_seq)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    auto it = publishers_.find(intra_process_publisher_id);
    if (it == publishers_.end()) {
      throw std::runtime_error("get_publisher_info_for_id called with invalid publisher id");
    }
    PublisherInfo & info = it->second;
    // Calculate the next message sequence number.
    message_seq = info.sequence_number.fetch_add(1);

    return info.buffer;
  }

  void
  store_intra_process_message(uint64_t intra_process_publisher_id, uint64_t message_seq)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    auto it = publishers_.find(intra_process_publisher_id);
    if (it == publishers_.end()) {
      throw std::runtime_error("store_intra_process_message called with invalid publisher id");
    }
    PublisherInfo & info = it->second;
    auto publisher = info.publisher.lock();
    if (!publisher) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }

    // Figure out what subscriptions should receive the message.
    auto & destined_subscriptions =
      subscription_ids_by_topic_[fixed_size_string(publisher->get_topic_name())];
    // Store the list for later comparison.
    if (info.target_subscriptions_by_message_sequence.count(message_seq) == 0) {
      info.target_subscriptions_by_message_sequence.emplace(
        message_seq, AllocSet(std::less<uint64_t>(), uint64_allocator));
    } else {
      info.target_subscriptions_by_message_sequence[message_seq].clear();
    }
    std::copy(
      destined_subscriptions.begin(), destined_subscriptions.end(),
      // Memory allocation occurs in info.target_subscriptions_by_message_sequence[message_seq]
      std::inserter(
        info.target_subscriptions_by_message_sequence[message_seq],
        // This ends up only being a hint to std::set, could also be .begin().
        info.target_subscriptions_by_message_sequence[message_seq].end()
      )
    );
  }

  mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  take_intra_process_message(
    uint64_t intra_process_publisher_id,
    uint64_t message_sequence_number,
    uint64_t requesting_subscriptions_intra_process_id,
    size_t & size
  )
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    PublisherInfo * info;
    {
      auto it = publishers_.find(intra_process_publisher_id);
      if (it == publishers_.end()) {
        // Publisher is either invalid or no longer exists.
        return 0;
      }
      info = &it->second;
    }
    // Figure out how many subscriptions are left.
    AllocSet * target_subs;
    {
      auto it = info->target_subscriptions_by_message_sequence.find(message_sequence_number);
      if (it == info->target_subscriptions_by_message_sequence.end()) {
        // Message is no longer being stored by this publisher.
        return 0;
      }
      target_subs = &it->second;
    }
    {
      auto it = std::find(
        target_subs->begin(), target_subs->end(),
        requesting_subscriptions_intra_process_id);
      if (it == target_subs->end()) {
        // This publisher id/message seq pair was not intended for this subscription.
        return 0;
      }
      target_subs->erase(it);
    }
    size = target_subs->size();
    return info->buffer;
  }

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

  size_t
  get_subscription_count(uint64_t intra_process_publisher_id) const
  {
    auto publisher_it = publishers_.find(intra_process_publisher_id);
    if (publisher_it == publishers_.end()) {
      // Publisher is either invalid or no longer exists.
      return 0;
    }
    auto publisher = publisher_it->second.publisher.lock();
    if (!publisher) {
      throw std::runtime_error("publisher has unexpectedly gone out of scope");
    }
    auto sub_map_it =
      subscription_ids_by_topic_.find(fixed_size_string(publisher->get_topic_name()));
    if (sub_map_it == subscription_ids_by_topic_.end()) {
      // No intraprocess subscribers
      return 0;
    }
    return sub_map_it->second.size();
  }

private:
  RCLCPP_DISABLE_COPY(IntraProcessManagerImpl)

  using FixedSizeString = std::array<char, RMW_TOPIC_MAX_NAME_LENGTH + 1>;

  FixedSizeString
  fixed_size_string(const char * str) const
  {
    FixedSizeString ret;
    size_t size = std::strlen(str) + 1;
    if (size > ret.size()) {
      throw std::runtime_error("failed to copy topic name");
    }
    std::memcpy(ret.data(), str, size);
    return ret;
  }
  struct strcmp_wrapper
  {
    bool
    operator()(const FixedSizeString lhs, const FixedSizeString rhs) const
    {
      return std::strcmp(lhs.data(), rhs.data()) < 0;
    }
  };

  template<typename T>
  using RebindAlloc = typename std::allocator_traits<Allocator>::template rebind_alloc<T>;

  RebindAlloc<uint64_t> uint64_allocator;

  using AllocSet = std::set<uint64_t, std::less<uint64_t>, RebindAlloc<uint64_t>>;
  using SubscriptionMap = std::unordered_map<
    uint64_t, SubscriptionBase::WeakPtr,
    std::hash<uint64_t>, std::equal_to<uint64_t>,
    RebindAlloc<std::pair<const uint64_t, SubscriptionBase::WeakPtr>>>;

  using IDTopicMap = std::map<
    FixedSizeString,
    AllocSet,
    strcmp_wrapper,
    RebindAlloc<std::pair<const FixedSizeString, AllocSet>>>;

  SubscriptionMap subscriptions_;

  IDTopicMap subscription_ids_by_topic_;

  struct PublisherInfo
  {
    RCLCPP_DISABLE_COPY(PublisherInfo)

    PublisherInfo() = default;

    PublisherBase::WeakPtr publisher;
    std::atomic<uint64_t> sequence_number;
    mapped_ring_buffer::MappedRingBufferBase::SharedPtr buffer;

    using TargetSubscriptionsMap = std::unordered_map<
      uint64_t, AllocSet,
      std::hash<uint64_t>, std::equal_to<uint64_t>,
      RebindAlloc<std::pair<const uint64_t, AllocSet>>>;
    TargetSubscriptionsMap target_subscriptions_by_message_sequence;
  };

  using PublisherMap = std::unordered_map<
    uint64_t, PublisherInfo,
    std::hash<uint64_t>, std::equal_to<uint64_t>,
    RebindAlloc<std::pair<const uint64_t, PublisherInfo>>>;

  PublisherMap publishers_;

  std::mutex runtime_mutex_;
};

RCLCPP_PUBLIC
IntraProcessManagerImplBase::SharedPtr
create_default_impl();

}  // namespace intra_process_manager
}  // namespace rclcpp

#endif  // RCLCPP__INTRA_PROCESS_MANAGER_IMPL_HPP_
