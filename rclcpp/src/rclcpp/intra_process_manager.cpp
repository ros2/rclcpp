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

#include "rclcpp/intra_process_manager.hpp"

namespace rclcpp
{
namespace intra_process_manager
{

static std::atomic<uint64_t> _next_unique_id {1};

IntraProcessManager::IntraProcessManager(
  rclcpp::intra_process_manager::IntraProcessManagerImplBase::SharedPtr impl)
: impl_(impl)
{}

IntraProcessManager::~IntraProcessManager()
{}

uint64_t
IntraProcessManager::add_publisher(
  rclcpp::PublisherBase::SharedPtr publisher,
  size_t buffer_size)
{
  auto id = IntraProcessManager::get_next_unique_id();
  size_t size = buffer_size > 0 ? buffer_size : publisher->get_queue_size();
  auto mrb = publisher->make_mapped_ring_buffer(size);
  impl_->add_publisher(id, publisher, mrb, size);
  if (!mrb) {
    throw std::runtime_error("failed to create a mapped ring buffer");
  }
  return id;
}

uint64_t
IntraProcessManager::add_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription)
{
  auto id = IntraProcessManager::get_next_unique_id();
  impl_->add_subscription(id, subscription);
  return id;
}

void
IntraProcessManager::remove_subscription(uint64_t intra_process_subscription_id)
{
  impl_->remove_subscription(intra_process_subscription_id);
}

void
IntraProcessManager::remove_publisher(uint64_t intra_process_publisher_id)
{
  impl_->remove_publisher(intra_process_publisher_id);
}

bool
IntraProcessManager::matches_any_publishers(const rmw_gid_t * id) const
{
  return impl_->matches_any_publishers(id);
}

size_t
IntraProcessManager::get_subscription_count(uint64_t intra_process_publisher_id) const
{
  return impl_->get_subscription_count(intra_process_publisher_id);
}

uint64_t
IntraProcessManager::get_next_unique_id()
{
  auto next_id = _next_unique_id.fetch_add(1, std::memory_order_relaxed);
  // Check for rollover (we started at 1).
  if (0 == next_id) {
    // This puts a technical limit on the number of times you can add a publisher or subscriber.
    // But even if you could add (and remove) them at 1 kHz (very optimistic rate)
    // it would still be a very long time before you could exhaust the pool of id's:
    //   2^64 / 1000 times per sec / 60 sec / 60 min / 24 hours / 365 days = 584,942,417 years
    // So around 585 million years. Even at 1 GHz, it would take 585 years.
    // I think it's safe to avoid trying to handle overflow.
    // If we roll over then it's most likely a bug.
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::overflow_error(
      "exhausted the unique id's for publishers and subscribers in this process "
      "(congratulations your computer is either extremely fast or extremely old)");
    // *INDENT-ON*
  }
  return next_id;
}

}  // namespace intra_process_manager
}  // namespace rclcpp
