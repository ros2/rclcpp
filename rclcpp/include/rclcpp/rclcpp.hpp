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

/** \mainpage rclcpp: ROS Client Library for C++
 *
 * `rclcpp` provides the canonical C++ API for interacting with ROS.
 * It consists of these main components:
 *
 * - Nodes
 *   - rclcpp::node::Node
 *   - rclcpp/node.hpp
 * - Publisher
 *   - rclcpp::node::Node::create_publisher()
 *   - rclcpp::publisher::Publisher
 *   - rclcpp::publisher::Publisher::publish()
 *   - rclcpp/publisher.hpp
 * - Subscription
 *   - rclcpp::node::Node::create_subscription()
 *   - rclcpp::subscription::Subscription
 *   - rclcpp/subscription.hpp
 * - Service Client
 *   - rclcpp::node::Node::create_client()
 *   - rclcpp::client::Client
 *   - rclcpp/client.hpp
 * - Service Server
 *   - rclcpp::node::Node::create_service()
 *   - rclcpp::service::Service
 *   - rclcpp/service.hpp
 * - Timer
 *   - rclcpp::node::Node::create_wall_timer()
 *   - rclcpp::timer::WallTimer
 *   - rclcpp::timer::TimerBase
 *   - rclcpp/timer.hpp
 * - Parameters:
 *   - rclcpp::node::Node::set_parameters()
 *   - rclcpp::node::Node::get_parameters()
 *   - rclcpp::node::Node::get_parameter()
 *   - rclcpp::node::Node::describe_parameters()
 *   - rclcpp::node::Node::list_parameters()
 *   - rclcpp::node::Node::register_param_change_callback()
 *   - rclcpp::parameter::ParameterVariant
 *   - rclcpp::parameter_client::AsyncParametersClient
 *   - rclcpp::parameter_client::SyncParametersClient
 *   - rclcpp/parameter.hpp
 *   - rclcpp/parameter_client.hpp
 *   - rclcpp/parameter_service.hpp
 * - Rate:
 *   - rclcpp::rate::Rate
 *   - rclcpp::rate::WallRate
 *   - rclcpp/rate.hpp
 *
 * There are also some components which help control the execution of callbacks:
 *
 * - Executors (responsible for execution of callbacks through a blocking spin):
 *   - rclcpp::spin()
 *   - rclcpp::spin_some()
 *   - rclcpp::spin_until_future_complete()
 *   - rclcpp::executors::single_threaded_executor::SingleThreadedExecutor
 *   - rclcpp::executors::single_threaded_executor::SingleThreadedExecutor::add_node()
 *   - rclcpp::executors::single_threaded_executor::SingleThreadedExecutor::spin()
 *   - rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor
 *   - rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor::add_node()
 *   - rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor::spin()
 *   - rclcpp/executor.hpp
 *   - rclcpp/executors.hpp
 *   - rclcpp/executors/single_threaded_executor.hpp
 *   - rclcpp/executors/multi_threaded_executor.hpp
 * - CallbackGroups (mechanism for enforcing concurrency rules for callbacks):
 *   - rclcpp::node::Node::create_callback_group()
 *   - rclcpp::callback_group::CallbackGroup
 *   - rclcpp/callback_group.hpp
 *
 * Additionally, there are some methods for introspecting the ROS graph:
 *
 * - Graph Events (a waitable event object that wakes up when the graph changes):
 *   - rclcpp::node::Node::get_graph_event()
 *   - rclcpp::node::Node::wait_for_graph_change()
 *   - rclcpp::event::Event
 * - List topic names and types:
 *   - rclcpp::node::Node::get_topic_names_and_types()
 * - Get the number of publishers or subscribers on a topic:
 *   - rclcpp::node::Node::count_publishers()
 *   - rclcpp::node::Node::count_subscribers()
 *
 * Finally, there are many internal API's and utilities:
 *
 * - Exceptions:
 *   - rclcpp/exceptions.hpp
 * - Allocator related items:
 *   - rclcpp/allocator/allocator_common.hpp
 *   - rclcpp/allocator/allocator_deleter.hpp
 * - Memory management tools:
 *   - rclcpp/memory_strategies.hpp
 *   - rclcpp/memory_strategy.hpp
 *   - rclcpp/message_memory_strategy.hpp
 *   - rclcpp/strategies/allocator_memory_strategy.hpp
 *   - rclcpp/strategies/message_pool_memory_strategy.hpp
 * - Context object which is shared amongst multiple Nodes:
 *   - rclcpp::context::Context
 *   - rclcpp/context.hpp
 *   - rclcpp/contexts/default_context.hpp
 * - Various utilities:
 *   - rclcpp/function_traits.hpp
 *   - rclcpp/macros.hpp
 *   - rclcpp/scope_exit.hpp
 *   - rclcpp/time.hpp
 *   - rclcpp/utilities.hpp
 *   - rclcpp/visibility_control.hpp
 */

#ifndef RCLCPP__RCLCPP_HPP_
#define RCLCPP__RCLCPP_HPP_

#include <csignal>
#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

// Namespace escalations.
// For example, this next line escalates type "rclcpp:node::Node" to "rclcpp::Node"
using rclcpp::node::Node;
using rclcpp::publisher::Publisher;
using rclcpp::subscription::SubscriptionBase;
using rclcpp::subscription::Subscription;
using rclcpp::rate::GenericRate;
using rclcpp::rate::WallRate;
using rclcpp::timer::GenericTimer;
using rclcpp::timer::TimerBase;
using rclcpp::timer::WallTimer;
using ContextSharedPtr = rclcpp::context::Context::SharedPtr;
using rclcpp::utilities::ok;
using rclcpp::utilities::shutdown;
using rclcpp::utilities::init;
using rclcpp::utilities::sleep_for;

}  // namespace rclcpp

#endif  // RCLCPP__RCLCPP_HPP_
