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

#include "rclcpp/any_executable.hpp"

#include <cassert>

using rclcpp::executor::AnyExecutable;

AnyExecutable::AnyExecutable() :
 subscription_(nullptr),
 subscription_intra_process_(nullptr),
 timer_(nullptr),
 service_(nullptr),
 client_(nullptr),
 callback_group_(nullptr),
 node_(nullptr)
{}

AnyExecutable::~AnyExecutable()
{
  // Make sure that discarded (taken but not executed) AnyExecutable's have
  // their callback groups reset. This can happen when an executor is canceled
  // between taking an AnyExecutable and executing it.
  if (callback_group_) {
    callback_group_->can_be_taken_from().store(true);
  }
}

bool AnyExecutable::is_one_field_set() const {
  std::atomic<size_t> fields_set(0);
  if (timer_) {
    fields_set.fetch_add(1, std::memory_order_relaxed);
  }/* else {
    assert(timer_.use_count() == 0);
  }*/
  if (subscription_) {
    fields_set.fetch_add(1, std::memory_order_relaxed);
  }/* else {
    assert(subscription_.use_count() == 0);
  }*/
  if (subscription_intra_process_) {
    fields_set.fetch_add(1, std::memory_order_relaxed);
  }
  if (service_) {
    fields_set.fetch_add(1, std::memory_order_relaxed);
  }
  if (client_) {
    fields_set.fetch_add(1, std::memory_order_relaxed);
  }
  return fields_set <= 1;
}

rclcpp::subscription::SubscriptionBase::ConstSharedPtr AnyExecutable::get_subscription() const {
  return subscription_;
}

rclcpp::subscription::SubscriptionBase::ConstSharedPtr
AnyExecutable::get_subscription_intra_process() const {
  return subscription_intra_process_;
}

rclcpp::timer::TimerBase::ConstSharedPtr AnyExecutable::get_timer() const {
  return timer_;
}

rclcpp::service::ServiceBase::SharedPtr AnyExecutable::get_service() const {
  return service_;
}

rclcpp::client::ClientBase::SharedPtr AnyExecutable::get_client() const {
  return client_;
}

rclcpp::callback_group::CallbackGroup::SharedPtr AnyExecutable::get_callback_group() const {
  return callback_group_;
}

rclcpp::node::Node::SharedPtr AnyExecutable::get_node() const {
  return node_;
}

void
AnyExecutable::set_subscription(
  const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription)
{
  subscription_ = subscription;
}

void
AnyExecutable::set_subscription_intra_process(
  const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription)
{
  subscription_intra_process_ = subscription;
}

void AnyExecutable::set_timer(const rclcpp::timer::TimerBase::ConstSharedPtr timer) {
  timer_ = timer;
}

void AnyExecutable::set_service(const rclcpp::service::ServiceBase::SharedPtr service) {
  service_ = service;
}

void AnyExecutable::set_client(const rclcpp::client::ClientBase::SharedPtr client) {
  client_ = client;
}

void
AnyExecutable::set_callback_group(
  const rclcpp::callback_group::CallbackGroup::SharedPtr callback_group)
{
  callback_group_ = callback_group;
}

void AnyExecutable::set_node(const rclcpp::node::Node::SharedPtr node) {
  node_ = node;
}
