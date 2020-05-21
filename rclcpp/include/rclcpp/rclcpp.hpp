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
 * - Node
 *   - rclcpp::Node
 *   - rclcpp/node.hpp
 * - Publisher
 *   - rclcpp::Node::create_publisher()
 *   - rclcpp::Publisher
 *   - rclcpp::Publisher::publish()
 *   - rclcpp/publisher.hpp
 * - Subscription
 *   - rclcpp::Node::create_subscription()
 *   - rclcpp::Subscription
 *   - rclcpp/subscription.hpp
 * - Service Client
 *   - rclcpp::Node::create_client()
 *   - rclcpp::Client
 *   - rclcpp/client.hpp
 * - Service Server
 *   - rclcpp::Node::create_service()
 *   - rclcpp::Service
 *   - rclcpp/service.hpp
 * - Timer
 *   - rclcpp::Node::create_wall_timer()
 *   - rclcpp::WallTimer
 *   - rclcpp::TimerBase
 *   - rclcpp/timer.hpp
 * - Parameters:
 *   - rclcpp::Node::set_parameters()
 *   - rclcpp::Node::get_parameters()
 *   - rclcpp::Node::get_parameter()
 *   - rclcpp::Node::describe_parameters()
 *   - rclcpp::Node::list_parameters()
 *   - rclcpp::Node::add_on_set_parameters_callback()
 *   - rclcpp::Node::remove_on_set_parameters_callback()
 *   - rclcpp::Parameter
 *   - rclcpp::ParameterValue
 *   - rclcpp::AsyncParametersClient
 *   - rclcpp::SyncParametersClient
 *   - rclcpp/parameter.hpp
 *   - rclcpp/parameter_value.hpp
 *   - rclcpp/parameter_client.hpp
 *   - rclcpp/parameter_service.hpp
 * - Rate:
 *   - rclcpp::Rate
 *   - rclcpp::WallRate
 *   - rclcpp/rate.hpp
 *
 * There are also some components which help control the execution of callbacks:
 *
 * - Executors (responsible for execution of callbacks through a blocking spin):
 *   - rclcpp::spin()
 *   - rclcpp::spin_some()
 *   - rclcpp::spin_until_future_complete()
 *   - rclcpp::executors::SingleThreadedExecutor
 *   - rclcpp::executors::SingleThreadedExecutor::add_node()
 *   - rclcpp::executors::SingleThreadedExecutor::spin()
 *   - rclcpp::executors::MultiThreadedExecutor
 *   - rclcpp::executors::MultiThreadedExecutor::add_node()
 *   - rclcpp::executors::MultiThreadedExecutor::spin()
 *   - rclcpp/executor.hpp
 *   - rclcpp/executors.hpp
 *   - rclcpp/executors/single_threaded_executor.hpp
 *   - rclcpp/executors/multi_threaded_executor.hpp
 * - CallbackGroups (mechanism for enforcing concurrency rules for callbacks):
 *   - rclcpp::Node::create_callback_group()
 *   - rclcpp::CallbackGroup
 *   - rclcpp/callback_group.hpp
 *
 * Additionally, there are some methods for introspecting the ROS graph:
 *
 * - Graph Events (a waitable event object that wakes up when the graph changes):
 *   - rclcpp::Node::get_graph_event()
 *   - rclcpp::Node::wait_for_graph_change()
 *   - rclcpp::Event
 * - List topic names and types:
 *   - rclcpp::Node::get_topic_names_and_types()
 * - Get the number of publishers or subscribers on a topic:
 *   - rclcpp::Node::count_publishers()
 *   - rclcpp::Node::count_subscribers()
 *
 * And components related to logging:
 *
 * - Logging macros:
 *   - Some examples (not exhaustive):
 *     - RCLCPP_DEBUG()
 *     - RCLCPP_INFO()
 *     - RCLCPP_WARN_ONCE()
 *     - RCLCPP_ERROR_SKIPFIRST()
 *   - rclcpp/logging.hpp
 * - Logger:
 *   - rclcpp::Logger
 *   - rclcpp/logger.hpp
 *   - rclcpp::Node::get_logger()
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
 *   - rclcpp::Context
 *   - rclcpp/context.hpp
 *   - rclcpp/contexts/default_context.hpp
 * - Various utilities:
 *   - rclcpp/duration.hpp
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
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp/waitable.hpp"

#endif  // RCLCPP__RCLCPP_HPP_
