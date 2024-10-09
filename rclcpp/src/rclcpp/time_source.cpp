// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"

namespace rclcpp
{

class ClocksState final
{
public:
  ClocksState()
  : logger_(rclcpp::get_logger("rclcpp")),
    last_time_msg_(std::make_shared<builtin_interfaces::msg::Time>())
  {
  }

  // An internal method to use in the clock callback that iterates and enables all clocks
  void enable_ros_time()
  {
    if (ros_time_active_) {
      // already enabled no-op
      return;
    }

    // Local storage
    ros_time_active_ = true;

    // Update all attached clocks to zero or last recorded time
    set_all_clocks(last_time_msg_, true);
  }

  // An internal method to use in the clock callback that iterates and disables all clocks
  void disable_ros_time()
  {
    if (!ros_time_active_) {
      // already disabled no-op
      return;
    }

    // Local storage
    ros_time_active_ = false;

    // Update all attached clocks
    auto msg = std::make_shared<builtin_interfaces::msg::Time>();
    set_all_clocks(msg, false);
  }

  // Check if ROS time is active
  bool is_ros_time_active() const
  {
    return ros_time_active_;
  }

  // Attach a clock
  void attachClock(rclcpp::Clock::SharedPtr clock)
  {
    {
      std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());
      if (clock->get_clock_type() != RCL_ROS_TIME && ros_time_active_) {
        throw std::invalid_argument(
                "ros_time_active_ can't be true while clock is not of RCL_ROS_TIME type");
      }
    }
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    associated_clocks_.push_back(clock);
    // Set the clock to zero unless there's a recently received message
    set_clock(last_time_msg_, ros_time_active_, clock);
  }

  // Detach a clock
  void detachClock(rclcpp::Clock::SharedPtr clock)
  {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    auto result = std::find(associated_clocks_.begin(), associated_clocks_.end(), clock);
    if (result != associated_clocks_.end()) {
      associated_clocks_.erase(result);
    } else {
      RCLCPP_ERROR(logger_, "failed to remove clock");
    }
  }

  // Internal helper function used inside iterators
  static void set_clock(
    const builtin_interfaces::msg::Time::SharedPtr msg,
    bool set_ros_time_enabled,
    rclcpp::Clock::SharedPtr clock)
  {
    std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());

    if (clock->get_clock_type() == RCL_ROS_TIME) {
      // Do change
      if (!set_ros_time_enabled && clock->ros_time_is_active()) {
        auto ret = rcl_disable_ros_time_override(clock->get_clock_handle());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(
            ret, "Failed to disable ros_time_override_status");
        }
      } else if (set_ros_time_enabled && !clock->ros_time_is_active()) {
        auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(
            ret, "Failed to enable ros_time_override_status");
        }
      }

      auto ret = rcl_set_ros_time_override(
        clock->get_clock_handle(),
        rclcpp::Time(*msg).nanoseconds());
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(
          ret, "Failed to set ros_time_override_status");
      }
    } else if (set_ros_time_enabled) {
      throw std::invalid_argument(
              "set_ros_time_enabled can't be true while clock is not of RCL_ROS_TIME type");
    }
  }

  // Internal helper function
  void set_all_clocks(
    const builtin_interfaces::msg::Time::SharedPtr msg,
    bool set_ros_time_enabled)
  {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
      set_clock(msg, set_ros_time_enabled, *it);
    }
  }

  // Cache the last clock message received
  void cache_last_msg(std::shared_ptr<const rosgraph_msgs::msg::Clock> msg)
  {
    last_time_msg_ = std::make_shared<builtin_interfaces::msg::Time>(msg->clock);
  }

  bool are_all_clocks_rcl_ros_time()
  {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto & clock : associated_clocks_) {
      std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());
      if (clock->get_clock_type() != RCL_ROS_TIME) {
        return false;
      }
    }
    return true;
  }

private:
  // Store (and update on node attach) logger for logging.
  Logger logger_;

  // A lock to protect iterating the associated_clocks_ field.
  std::mutex clock_list_lock_;
  // A vector to store references to associated clocks.
  std::vector<rclcpp::Clock::SharedPtr> associated_clocks_;

  // Local storage of validity of ROS time
  // This is needed when new clocks are added.
  bool ros_time_active_{false};
  // Last set message to be passed to newly registered clocks
  std::shared_ptr<builtin_interfaces::msg::Time> last_time_msg_{nullptr};
};

class TimeSource::NodeState final
{
public:
  NodeState(const rclcpp::QoS & qos, bool use_clock_thread)
  : use_clock_thread_(use_clock_thread),
    logger_(rclcpp::get_logger("rclcpp")),
    qos_(qos)
  {
  }

  ~NodeState()
  {
    if (
      node_base_ || node_topics_ || node_graph_ || node_services_ ||
      node_logging_ || node_clock_ || node_parameters_)
    {
      detachNode();
    }
  }

  // Check if a clock thread will be used
  bool get_use_clock_thread()
  {
    return use_clock_thread_;
  }

  // Set whether a clock thread will be used
  void set_use_clock_thread(bool use_clock_thread)
  {
    use_clock_thread_ = use_clock_thread;
  }

  // Check if the clock thread is joinable
  bool clock_thread_is_joinable()
  {
    return clock_executor_thread_.joinable();
  }

  // Attach a node to this time source
  void attachNode(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
  {
    node_base_ = node_base_interface;
    node_topics_ = node_topics_interface;
    node_graph_ = node_graph_interface;
    node_services_ = node_services_interface;
    node_logging_ = node_logging_interface;
    node_clock_ = node_clock_interface;
    node_parameters_ = node_parameters_interface;
    // TODO(tfoote): Update QOS

    logger_ = node_logging_->get_logger();

    // Though this defaults to false, it can be overridden by initial parameter values for the
    // node, which may be given by the user at the node's construction or even by command-line
    // arguments.
    rclcpp::ParameterValue use_sim_time_param;
    const std::string use_sim_time_name = "use_sim_time";
    if (!node_parameters_->has_parameter(use_sim_time_name)) {
      use_sim_time_param = node_parameters_->declare_parameter(
        use_sim_time_name,
        rclcpp::ParameterValue(false));
    } else {
      use_sim_time_param = node_parameters_->get_parameter(use_sim_time_name).get_parameter_value();
    }
    if (use_sim_time_param.get_type() == rclcpp::PARAMETER_BOOL) {
      if (use_sim_time_param.get<bool>()) {
        parameter_state_ = SET_TRUE;
        clocks_state_.enable_ros_time();
        create_clock_sub();
      }
    } else {
      RCLCPP_ERROR(
        logger_, "Invalid type '%s' for parameter 'use_sim_time', should be 'bool'",
        rclcpp::to_string(use_sim_time_param.get_type()).c_str());
      throw std::invalid_argument("Invalid type for parameter 'use_sim_time', should be 'bool'");
    }

    on_set_parameters_callback_ = node_parameters_->add_on_set_parameters_callback(
      std::bind(&TimeSource::NodeState::on_set_parameters, this, std::placeholders::_1));


    // TODO(tfoote) use parameters interface not subscribe to events via topic ticketed #609
    parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
      node_topics_,
      [this](std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event) {
        if (node_base_ != nullptr) {
          this->on_parameter_event(event);
        }
        // Do nothing if node_base_ is nullptr because it means the TimeSource is now
        // without an attached node
      });
  }

  // Detach the attached node
  void detachNode()
  {
    // destroy_clock_sub() *must* be first here, to ensure that the executor
    // can't possibly call any of the callbacks as we are cleaning up.
    destroy_clock_sub();
    clocks_state_.disable_ros_time();
    if (on_set_parameters_callback_) {
      node_parameters_->remove_on_set_parameters_callback(on_set_parameters_callback_.get());
    }
    on_set_parameters_callback_.reset();
    parameter_subscription_.reset();
    node_base_.reset();
    node_topics_.reset();
    node_graph_.reset();
    node_services_.reset();
    node_logging_.reset();
    node_clock_.reset();
    node_parameters_.reset();
  }

  void attachClock(std::shared_ptr<rclcpp::Clock> clock)
  {
    clocks_state_.attachClock(std::move(clock));
  }

  void detachClock(std::shared_ptr<rclcpp::Clock> clock)
  {
    clocks_state_.detachClock(std::move(clock));
  }

private:
  ClocksState clocks_state_;

  // Dedicated thread for clock subscription.
  bool use_clock_thread_;
  std::thread clock_executor_thread_;

  // Preserve the node reference
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_{nullptr};
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_{nullptr};
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_{nullptr};
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_{nullptr};
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_{nullptr};
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_{nullptr};
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_{nullptr};

  // Store (and update on node attach) logger for logging.
  Logger logger_;

  // QoS of the clock subscription.
  rclcpp::QoS qos_;

  // The subscription for the clock callback
  using SubscriptionT = rclcpp::Subscription<rosgraph_msgs::msg::Clock>;
  std::shared_ptr<SubscriptionT> clock_subscription_{nullptr};
  std::mutex clock_sub_lock_;
  rclcpp::CallbackGroup::SharedPtr clock_callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr clock_executor_;
  std::promise<void> cancel_clock_executor_promise_;

  // The clock callback itself
  void clock_cb(std::shared_ptr<const rosgraph_msgs::msg::Clock> msg)
  {
    if (!clocks_state_.is_ros_time_active() && SET_TRUE == this->parameter_state_) {
      clocks_state_.enable_ros_time();
    }
    // Cache the last message in case a new clock is attached.
    clocks_state_.cache_last_msg(msg);
    auto time_msg = std::make_shared<builtin_interfaces::msg::Time>(msg->clock);

    if (SET_TRUE == this->parameter_state_) {
      clocks_state_.set_all_clocks(time_msg, true);
    }
  }

  // Create the subscription for the clock topic
  void create_clock_sub()
  {
    std::lock_guard<std::mutex> guard(clock_sub_lock_);
    if (clock_subscription_) {
      // Subscription already created.
      return;
    }

    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions(
      {
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability,
      });

    if (use_clock_thread_) {
      clock_callback_group_ = node_base_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false
      );
      options.callback_group = clock_callback_group_;
      rclcpp::ExecutorOptions exec_options;
      exec_options.context = node_base_->get_context();
      clock_executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_options);
      if (!clock_executor_thread_.joinable()) {
        cancel_clock_executor_promise_ = std::promise<void>{};
        clock_executor_thread_ = std::thread(
          [this]() {
            auto future = cancel_clock_executor_promise_.get_future();
            clock_executor_->add_callback_group(clock_callback_group_, node_base_);
            clock_executor_->spin_until_future_complete(future);
          }
        );
      }
    }

    clock_subscription_ = rclcpp::create_subscription<rosgraph_msgs::msg::Clock>(
      node_parameters_,
      node_topics_,
      "/clock",
      qos_,
      [this](std::shared_ptr<const rosgraph_msgs::msg::Clock> msg) {
        // We are using node_base_ as an indication if there is a node attached.
        // Only call the clock_cb if that is the case.
        if (node_base_ != nullptr) {
          clock_cb(msg);
        }
      },
      options
    );
  }

  // Destroy the subscription for the clock topic
  void destroy_clock_sub()
  {
    std::lock_guard<std::mutex> guard(clock_sub_lock_);
    if (clock_executor_thread_.joinable()) {
      cancel_clock_executor_promise_.set_value();
      clock_executor_->cancel();
      clock_executor_thread_.join();
      clock_executor_->remove_callback_group(clock_callback_group_);
    }
    clock_subscription_.reset();
  }

  // On set Parameters callback handle
  node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};

  // Parameter Event subscription
  using ParamSubscriptionT = rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>;
  std::shared_ptr<ParamSubscriptionT> parameter_subscription_;

  // Callback for parameter settings
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "use_sim_time" && param.get_type() == rclcpp::PARAMETER_BOOL) {
        if (param.as_bool() && !(clocks_state_.are_all_clocks_rcl_ros_time())) {
          result.successful = false;
          result.reason =
            "use_sim_time parameter can't be true while clocks are not all of RCL_ROS_TIME type";
          RCLCPP_ERROR(
            logger_,
            "use_sim_time parameter can't be true while clocks are not all of RCL_ROS_TIME type");
        }
      }
    }
    return result;
  }

  // Callback for parameter updates
  void on_parameter_event(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
  {
    // Filter out events on 'use_sim_time' parameter instances in other nodes.
    if (event->node != node_base_->get_fully_qualified_name()) {
      return;
    }
    // Filter for only 'use_sim_time' being added or changed.
    rclcpp::ParameterEventsFilter filter(event, {"use_sim_time"},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    for (auto & it : filter.get_events()) {
      if (it.second->value.type != ParameterType::PARAMETER_BOOL) {
        RCLCPP_ERROR(logger_, "use_sim_time parameter cannot be set to anything but a bool");
        continue;
      }
      if (it.second->value.bool_value) {
        parameter_state_ = SET_TRUE;
        clocks_state_.enable_ros_time();
        create_clock_sub();
      } else {
        parameter_state_ = SET_FALSE;
        destroy_clock_sub();
        clocks_state_.disable_ros_time();
      }
    }
    // Handle the case that use_sim_time was deleted.
    rclcpp::ParameterEventsFilter deleted(event, {"use_sim_time"},
      {rclcpp::ParameterEventsFilter::EventType::DELETED});
    for (auto & it : deleted.get_events()) {
      (void) it;  // if there is a match it's already matched, don't bother reading it.
      // If the parameter is deleted mark it as unset but don't change state.
      parameter_state_ = UNSET;
    }
  }

  // An enum to hold the parameter state
  enum UseSimTimeParameterState {UNSET, SET_TRUE, SET_FALSE};
  UseSimTimeParameterState parameter_state_;
};

TimeSource::TimeSource(
  std::shared_ptr<rclcpp::Node> node,
  const rclcpp::QoS & qos,
  bool use_clock_thread)
: TimeSource(qos, use_clock_thread)
{
  attachNode(node);
}

TimeSource::TimeSource(
  const rclcpp::QoS & qos,
  bool use_clock_thread)
: constructed_use_clock_thread_(use_clock_thread),
  constructed_qos_(qos)
{
  node_state_ = std::make_shared<NodeState>(qos, use_clock_thread);
}

void TimeSource::attachNode(rclcpp::Node::SharedPtr node)
{
  node_state_->set_use_clock_thread(node->get_node_options().use_clock_thread());
  attachNode(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    node->get_node_parameters_interface());
}

void TimeSource::attachNode(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
{
  node_state_->attachNode(
    std::move(node_base_interface),
    std::move(node_topics_interface),
    std::move(node_graph_interface),
    std::move(node_services_interface),
    std::move(node_logging_interface),
    std::move(node_clock_interface),
    std::move(node_parameters_interface));
}

void TimeSource::detachNode()
{
  node_state_.reset();
  node_state_ = std::make_shared<NodeState>(
    constructed_qos_,
    constructed_use_clock_thread_);
}

void TimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  node_state_->attachClock(std::move(clock));
}

void TimeSource::detachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  node_state_->detachClock(std::move(clock));
}

bool TimeSource::get_use_clock_thread()
{
  return node_state_->get_use_clock_thread();
}

void TimeSource::set_use_clock_thread(bool use_clock_thread)
{
  node_state_->set_use_clock_thread(use_clock_thread);
}

bool TimeSource::clock_thread_is_joinable()
{
  return node_state_->clock_thread_is_joinable();
}

TimeSource::~TimeSource()
{
}

}  // namespace rclcpp
