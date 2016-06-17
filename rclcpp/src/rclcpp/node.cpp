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

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node.hpp"

using rclcpp::node::Node;
using rclcpp::exceptions::throw_from_rcl_error;

Node::Node(const std::string & node_name, bool use_intra_process_comms)
: Node(
    node_name,
    rclcpp::contexts::default_context::get_global_default_context(),
    use_intra_process_comms)
{}

Node::Node(
  const std::string & node_name,
  rclcpp::context::Context::SharedPtr context,
  bool use_intra_process_comms)
: name_(node_name), context_(context),
  number_of_subscriptions_(0), number_of_timers_(0), number_of_services_(0),
  use_intra_process_comms_(use_intra_process_comms),
  graph_listener_(context->get_sub_context<rclcpp::graph_listener::GraphListener>()),
  added_to_graph_listener_(false), graph_users_count_(0)
{
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(&notify_guard_condition_, guard_condition_options);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }

  has_executor.store(false);
  size_t domain_id = 0;
  char * ros_domain_id = nullptr;
  const char * env_var = "ROS_DOMAIN_ID";
#ifndef _WIN32
  ros_domain_id = getenv(env_var);
#else
  size_t ros_domain_id_size;
  _dupenv_s(&ros_domain_id, &ros_domain_id_size, env_var);
#endif
  if (ros_domain_id) {
    uint32_t number = strtoul(ros_domain_id, NULL, 0);
    if (number == (std::numeric_limits<uint32_t>::max)()) {
      // Finalize the interrupt guard condition.
      if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
        fprintf(stderr,
          "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
      }
      throw std::runtime_error("failed to interpret ROS_DOMAIN_ID as integral number");
    }
    domain_id = static_cast<size_t>(number);
#ifdef _WIN32
    free(ros_domain_id);
#endif
  }

  rcl_node_t * rcl_node = new rcl_node_t(rcl_get_zero_initialized_node());
  node_handle_.reset(rcl_node, [](rcl_node_t * node) {
    if (rcl_node_fini(node) != RCL_RET_OK) {
      fprintf(
        stderr, "Error in destruction of rmw node handle: %s\n", rcl_get_error_string_safe());
    }
    delete node;
  });
  rcl_node_options_t options = rcl_node_get_default_options();
  // TODO(jacquelinekay): Allocator options
  options.domain_id = domain_id;
  ret = rcl_node_init(node_handle_.get(), name_.c_str(), &options);
  if (ret != RCL_RET_OK) {
    // Finalize the interrupt guard condition.
    if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
      fprintf(stderr,
        "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
    }

    throw_from_rcl_error(ret, "failed to initialize rcl node");
  }

  using rclcpp::callback_group::CallbackGroupType;
  default_callback_group_ = create_callback_group(
    CallbackGroupType::MutuallyExclusive);
  events_publisher_ = create_publisher<rcl_interfaces::msg::ParameterEvent>(
    "parameter_events", rmw_qos_profile_parameter_events);
}

Node::~Node()
{
  // Finalize the interrupt guard condition.
  if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
    fprintf(stderr,
      "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
  }
  // Remove self from graph listener.
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    if (added_to_graph_listener_) {
      graph_listener_->remove_node(this);
    }
  }
}

const std::string &
Node::get_name() const
{
  return name_;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
Node::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  using rclcpp::callback_group::CallbackGroup;
  using rclcpp::callback_group::CallbackGroupType;
  auto group = CallbackGroup::SharedPtr(new CallbackGroup(group_type));
  callback_groups_.push_back(group);
  return group;
}

bool
Node::group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  bool group_belongs_to_this_node = false;
  for (auto & weak_group : this->callback_groups_) {
    auto cur_group = weak_group.lock();
    if (cur_group && (cur_group == group)) {
      group_belongs_to_this_node = true;
    }
  }
  return group_belongs_to_this_node;
}

// TODO(wjwwood): reenable this once I figure out why the demo doesn't build with it.
// rclcpp::timer::WallTimer::SharedPtr
// Node::create_wall_timer(
//   std::chrono::duration<long double, std::nano> period,
//   rclcpp::timer::CallbackType callback,
//   rclcpp::callback_group::CallbackGroup::SharedPtr group)
// {
//   return create_wall_timer(
//     std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//     callback,
//     group);
// }

std::vector<rcl_interfaces::msg::SetParametersResult>
Node::set_parameters(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  for (auto p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }
  return results;
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameters_atomically(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, rclcpp::parameter::ParameterVariant> tmp_map;
  auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

  for (auto p : parameters) {
    if (parameters_.find(p.get_name()) == parameters_.end()) {
      if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
        parameter_event->new_parameters.push_back(p.to_parameter());
      }
    } else if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
      parameter_event->changed_parameters.push_back(p.to_parameter());
    } else {
      parameter_event->deleted_parameters.push_back(p.to_parameter());
    }
    tmp_map[p.get_name()] = p;
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());
  std::swap(tmp_map, parameters_);

  // TODO(jacquelinekay): handle parameter constraints
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  events_publisher_->publish(parameter_event);

  return result;
}

std::vector<rclcpp::parameter::ParameterVariant>
Node::get_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::parameter::ParameterVariant> results;

  for (auto & name : names) {
    if (std::any_of(parameters_.cbegin(), parameters_.cend(),
      [&name](const std::pair<std::string, rclcpp::parameter::ParameterVariant> & kv) {
      return name == kv.first;
    }))
    {
      results.push_back(parameters_.at(name));
    }
  }
  return results;
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
      parameter_descriptor.name = kv.first;
      parameter_descriptor.type = kv.second.get_type();
      results.push_back(parameter_descriptor);
    }
  }
  return results;
}

std::vector<uint8_t>
Node::get_parameter_types(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      results.push_back(kv.second.get_type());
    } else {
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    }
  }
  return results;
}

rcl_interfaces::msg::ListParametersResult
Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(esteve): define parameter separator, use "." for now
  for (auto & kv : parameters_) {
    if (((prefixes.size() == 0) &&
      ((depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
      (static_cast<uint64_t>(std::count(kv.first.begin(), kv.first.end(), '.')) < depth))) ||
      (std::any_of(prefixes.cbegin(), prefixes.cend(), [&kv, &depth](const std::string & prefix) {
      if (kv.first == prefix) {
        return true;
      } else if (kv.first.find(prefix + ".") == 0) {
        size_t length = prefix.length();
        std::string substr = kv.first.substr(length);
        // Cast as unsigned integer to avoid warning
        return (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
        (static_cast<uint64_t>(std::count(substr.begin(), substr.end(), '.')) < depth);
      }
      return false;
    })))
    {
      result.names.push_back(kv.first);
      size_t last_separator = kv.first.find_last_of('.');
      if (std::string::npos != last_separator) {
        std::string prefix = kv.first.substr(0, last_separator);
        if (std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
          result.prefixes.cend())
        {
          result.prefixes.push_back(prefix);
        }
      }
    }
  }
  return result;
}

std::map<std::string, std::string>
Node::get_topic_names_and_types() const
{
  rmw_topic_names_and_types_t topic_names_and_types;
  topic_names_and_types.topic_count = 0;
  topic_names_and_types.topic_names = nullptr;
  topic_names_and_types.type_names = nullptr;

  auto ret = rmw_get_topic_names_and_types(rcl_node_get_rmw_handle(node_handle_.get()),
      &topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not get topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  std::map<std::string, std::string> topics;
  for (size_t i = 0; i < topic_names_and_types.topic_count; ++i) {
    topics[topic_names_and_types.topic_names[i]] = topic_names_and_types.type_names[i];
  }

  ret = rmw_destroy_topic_names_and_types(&topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  return topics;
}

size_t
Node::count_publishers(const std::string & topic_name) const
{
  size_t count;
  auto ret = rmw_count_publishers(rcl_node_get_rmw_handle(node_handle_.get()),
      topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count publishers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}

size_t
Node::count_subscribers(const std::string & topic_name) const
{
  size_t count;
  auto ret = rmw_count_subscribers(rcl_node_get_rmw_handle(node_handle_.get()),
      topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count subscribers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}


const Node::CallbackGroupWeakPtrList &
Node::get_callback_groups() const
{
  return callback_groups_;
}

const rcl_guard_condition_t *
Node::get_notify_guard_condition() const
{
  return &notify_guard_condition_;
}

const rcl_guard_condition_t *
Node::get_graph_guard_condition() const
{
  return rcl_node_get_graph_guard_condition(node_handle_.get());
}

const rcl_node_t *
Node::get_rcl_node_handle() const
{
  return node_handle_.get();
}

rcl_node_t *
Node::get_rcl_node_handle()
{
  return node_handle_.get();
}

std::shared_ptr<rcl_node_t>
Node::get_shared_node_handle()
{
  return node_handle_;
}

void
Node::notify_graph_change()
{
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    bool bad_ptr_encountered = false;
    for (auto & event_wptr : graph_events_) {
      auto event_ptr = event_wptr.lock();
      if (event_ptr) {
        event_ptr->set();
      } else {
        bad_ptr_encountered = true;
      }
    }
    if (bad_ptr_encountered) {
      // remove invalid pointers with the erase-remove idiom
      graph_events_.erase(
        std::remove_if(
          graph_events_.begin(),
          graph_events_.end(),
          [](const rclcpp::event::Event::WeakPtr & wptr) {
        return wptr.expired();
      }),
        graph_events_.end());
      // update graph_users_count_
      graph_users_count_.store(graph_events_.size());
    }
  }
  graph_cv_.notify_all();
  rcl_ret_t ret = rcl_trigger_guard_condition(&notify_guard_condition_);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to trigger notify guard condition");
  }
}

void
Node::notify_shutdown()
{
  // notify here anything that will not be woken up by ctrl-c or rclcpp::shutdown().
  graph_cv_.notify_all();
}

rclcpp::event::Event::SharedPtr
Node::get_graph_event()
{
  auto event = rclcpp::event::Event::make_shared();
  std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
  // on first call, add node to graph_listener_
  if (!added_to_graph_listener_) {
    graph_listener_->add_node(this);
    added_to_graph_listener_ = true;
    graph_listener_->start_if_not_started();
  }
  graph_events_.push_back(event);
  graph_users_count_++;
  return event;
}

void
Node::wait_for_graph_change(
  rclcpp::event::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  if (!event) {
    throw InvalidEventError();
  }
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    bool event_in_graph_events = false;
    for (const auto & event_wptr : graph_events_) {
      if (event == event_wptr.lock()) {
        event_in_graph_events = true;
        break;
      }
    }
    if (!event_in_graph_events) {
      throw EventNotRegisteredError();
    }
  }
  std::unique_lock<std::mutex> graph_lock(graph_mutex_);
  graph_cv_.wait_for(graph_lock, timeout, [&event]() {
    return event->check() || !rclcpp::utilities::ok();
  });
}

size_t
Node::count_graph_users()
{
  return graph_users_count_.load();
}
