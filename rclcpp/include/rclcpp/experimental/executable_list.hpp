// Copyright 2019 Nobleo Technology
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

#ifndef RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_
#define RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_

#include <memory>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace experimental
{

/// This class contains subscriptionbase, timerbase, etc. which can be used to run callbacks.
class ExecutableList final
{
public:
  RCLCPP_PUBLIC
  ExecutableList();

  RCLCPP_PUBLIC
  ~ExecutableList();

  RCLCPP_PUBLIC
  void
  clear();

  RCLCPP_PUBLIC
  void
  add_subscription(rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  void
  add_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  void
  add_service(rclcpp::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  void
  add_client(rclcpp::ClientBase::SharedPtr client);

  RCLCPP_PUBLIC
  void
  add_waitable(rclcpp::Waitable::SharedPtr waitable);

  // Vector containing the SubscriptionBase of all the subscriptions added to the executor.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscription;
  // Contains the count of added subscriptions
  size_t number_of_subscriptions;
  // Vector containing the TimerBase of all the timers added to the executor.
  std::vector<rclcpp::TimerBase::SharedPtr> timer;
  // Contains the count of added timers
  size_t number_of_timers;
  // Vector containing the ServiceBase of all the services added to the executor.
  std::vector<rclcpp::ServiceBase::SharedPtr> service;
  // Contains the count of added services
  size_t number_of_services;
  // Vector containing the ClientBase of all the clients added to the executor.
  std::vector<rclcpp::ClientBase::SharedPtr> client;
  // Contains the count of added clients
  size_t number_of_clients;
  // Vector containing all the waitables added to the executor.
  std::vector<rclcpp::Waitable::SharedPtr> waitable;
  // Contains the count of added waitables
  size_t number_of_waitables;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_
