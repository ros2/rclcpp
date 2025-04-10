// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <deque>

#include "rcl_action/action_server.h"
#include "rcl_action/wait.h"

#include "rcpputils/scope_exit.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp_action/server.hpp"

using rclcpp_action::ServerBase;
using rclcpp_action::GoalUUID;

struct ServerBaseData;

namespace rclcpp_action
{

struct ServerBaseData
{
  using GoalRequestData = std::tuple<
    rcl_ret_t,
    const rcl_action_goal_info_t,
    rmw_request_id_t,
    std::shared_ptr<void>
  >;

  using CancelRequestData = std::tuple<
    rcl_ret_t,
    std::shared_ptr<action_msgs::srv::CancelGoal::Request>,
    rmw_request_id_t
  >;

  using ResultRequestData = std::tuple<rcl_ret_t, std::shared_ptr<void>, rmw_request_id_t>;

  using GoalExpiredData = struct Empty {};

  std::variant<GoalRequestData, CancelRequestData, ResultRequestData, GoalExpiredData> data;

  explicit ServerBaseData(GoalRequestData && data_in)
  : data(std::move(data_in)) {}
  explicit ServerBaseData(CancelRequestData && data_in)
  : data(std::move(data_in)) {}
  explicit ServerBaseData(ResultRequestData && data_in)
  : data(std::move(data_in)) {}
  explicit ServerBaseData(GoalExpiredData && data_in)
  : data(std::move(data_in)) {}
};

class ServerBaseImpl
{
public:
  ServerBaseImpl(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger
  )
  : clock_(clock), logger_(logger)
  {
  }

  ~ServerBaseImpl()
  {
    stop_expire_thread();
  }

  std::recursive_mutex expire_thread_reentrant_mutex_;
  std::thread expire_thread;

  // must be a class member, so keep it in scope
  std::function<void(size_t)> expire_reset_callback;

  rclcpp::ClockConditionalVariable::SharedPtr expire_cv;
  std::atomic<bool> expire_time_changed = false;
  std::atomic<bool> terminate_expire_thread = true;

  void
  stop_expire_thread()
  {
    std::lock_guard<std::recursive_mutex> lock(expire_thread_reentrant_mutex_);
    if(expire_cv) {
      {
        // Note, even though terminate_expire_thread is an atomic, we
        // need to acquire the mutex, to avoid a race between the wait_until
        // and the setting of this terminate_expire_thread
        std::lock_guard<std::mutex> l(expire_cv->mutex());
        terminate_expire_thread = true;
      }
      expire_cv->notify_one();
      expire_thread.join();
      expire_reset_callback = std::function<void(size_t)>();
      expire_cv.reset();
    }
  }

  void
  start_expire_thread(std::function<void(size_t, int)> expire_ready_cb)
  {
    {
      std::lock_guard<std::recursive_mutex> lock(expire_thread_reentrant_mutex_);
      if(expire_cv) {
        return;
      }

      terminate_expire_thread = false;
      expire_cv = std::make_shared<rclcpp::ClockConditionalVariable>(clock_);
    }

    rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context();
    std::shared_ptr<rcl_context_t> rcl_context = context->get_rcl_context();

    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    rcl_ret_t ret =
      rcl_wait_set_init(&wait_set, num_subscriptions_, num_guard_conditions_, num_timers_,
        num_clients_, num_services_, 0, rcl_context.get(), rcl_get_default_allocator());
    if(ret != RCL_RET_OK) {
      throw std::runtime_error("Internal error starting timer thread");
    }

    ret = rcl_action_wait_set_add_action_server(
      &wait_set, action_server_.get(), NULL);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "ServerBase::add_to_wait_set() failed");
    }

    // extract the timer from the wait set
    const rcl_timer_t *expire_timer = wait_set.timers[0];

    // don't need the wait set any more
    ret = rcl_wait_set_fini(&wait_set);

    // the maybe_unused is needed here to satisfy cpplint
    expire_reset_callback = [this] ([[maybe_unused]] size_t reset_calls)
      {
        {
          std::lock_guard<std::mutex> l(expire_cv->mutex());
          expire_time_changed = true;
        }
        expire_cv->notify_one();
      };

    rcl_event_callback_t rcl_reset_callback = rclcpp::detail::cpp_callback_trampoline<
      decltype(expire_reset_callback), const void *, size_t>;

    ret = rcl_timer_set_on_reset_callback(expire_timer, rcl_reset_callback,
        static_cast<const void *>(&expire_reset_callback));
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set timer on reset callback");
    }

    expire_thread = std::thread([this, expire_timer, expire_ready_callback = expire_ready_cb]() {
          rcl_clock_type_t clock_type = clock_->get_clock_type();

          while(!terminate_expire_thread) {
            {
              std::unique_lock<std::mutex> l(expire_cv->mutex());

              // reset to false, we are handling the time change right now
              expire_time_changed = false;
              int64_t time_until_call = 0;

              auto ret2 = rcl_timer_get_time_until_next_call(expire_timer, &time_until_call);
              if (ret2 == RCL_RET_TIMER_CANCELED) {
                rclcpp::Duration endless(std::chrono::hours(100));
                expire_cv->wait_until(l, clock_->now() + endless, [this] () -> bool {
                  return expire_time_changed || terminate_expire_thread;
                });
                continue;
              }

              if(time_until_call > 0) {
                rclcpp::Time next_wakeup(clock_->now().nanoseconds() + time_until_call, clock_type);
                expire_cv->wait_until(l, next_wakeup, [this] () -> bool {
                  return expire_time_changed || terminate_expire_thread;
                });
              }
            }

            // don't call expire callback if we are terminating
            if(terminate_expire_thread) {
              return;
            }

            bool is_ready = false;
            if(rcl_timer_is_ready(expire_timer, &is_ready) == RCL_RET_OK && is_ready) {
              // we need to cancel the timer here, to avoid a endless loop
              // in case a new goal expires, the timer will be reset.
              auto ret2 = rcl_timer_cancel(const_cast<rcl_timer_t *>(expire_timer));
              if(ret2 != RCL_RET_OK) {
                rclcpp::exceptions::throw_from_rcl_error(ret2, "Failed to cancel timer");
              }

              std::lock_guard<std::recursive_mutex> lock(expire_thread_reentrant_mutex_);
              if(expire_ready_callback) {
                expire_ready_callback(1, static_cast<int>(ServerBase::EntityType::Expired));
              }
            }
          }
      });
  }

  // Lock for action_server_
  std::recursive_mutex action_server_reentrant_mutex_;

  rclcpp::Clock::SharedPtr clock_;

  // Do not declare this before clock_ as this depends on clock_(see #1526)
  std::shared_ptr<rcl_action_server_t> action_server_;

  size_t num_subscriptions_ = 0;
  size_t num_timers_ = 0;
  size_t num_clients_ = 0;
  size_t num_services_ = 0;
  size_t num_guard_conditions_ = 0;

  // Lock for unordered_maps
  std::recursive_mutex unordered_map_mutex_;

  // Results to be kept until the goal expires after reaching a terminal state
  std::unordered_map<GoalUUID, std::shared_ptr<void>> goal_results_;
  // Requests for results are kept until a result becomes available
  std::unordered_map<GoalUUID, std::vector<rmw_request_id_t>> result_requests_;
  // rcl goal handles are kept so api to send result doesn't try to access freed memory
  std::unordered_map<GoalUUID, std::shared_ptr<rcl_action_goal_handle_t>> goal_handles_;

  // next ready event for taking, will be set by is_ready and will be processed by take_data
  std::atomic<size_t> next_ready_event;

  rclcpp::Logger logger_;
};

}  // namespace rclcpp_action

ServerBase::ServerBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_server_options_t & options
)
: pimpl_(new ServerBaseImpl(
      node_clock->get_clock(), node_logging->get_logger().get_child("rclcpp_action")))
{
  auto deleter = [node_base](rcl_action_server_t * ptr)
    {
      if (nullptr != ptr) {
        rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
        rcl_ret_t ret = rcl_action_server_fini(ptr, rcl_node);
        if (RCL_RET_OK != ret) {
          RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp_action"),
            "failed to fini rcl_action_server_t in deleter");
        }
        delete ptr;
      }
    };

  pimpl_->action_server_.reset(new rcl_action_server_t, deleter);
  *(pimpl_->action_server_) = rcl_action_get_zero_initialized_server();

  rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
  rcl_clock_t * rcl_clock = pimpl_->clock_->get_clock_handle();

  rcl_ret_t ret = rcl_action_server_init(
    pimpl_->action_server_.get(), rcl_node, rcl_clock, type_support, name.c_str(), &options);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  ret = rcl_action_server_wait_set_get_num_entities(
    pimpl_->action_server_.get(),
    &pimpl_->num_subscriptions_,
    &pimpl_->num_guard_conditions_,
    &pimpl_->num_timers_,
    &pimpl_->num_clients_,
    &pimpl_->num_services_);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

ServerBase::~ServerBase()
{
}

size_t
ServerBase::get_number_of_ready_subscriptions()
{
  return pimpl_->num_subscriptions_;
}

size_t
ServerBase::get_number_of_ready_timers()
{
  return pimpl_->num_timers_;
}

size_t
ServerBase::get_number_of_ready_clients()
{
  return pimpl_->num_clients_;
}

size_t
ServerBase::get_number_of_ready_services()
{
  return pimpl_->num_services_;
}

size_t
ServerBase::get_number_of_ready_guard_conditions()
{
  return pimpl_->num_guard_conditions_;
}

void
ServerBase::add_to_wait_set(rcl_wait_set_t & wait_set)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
  rcl_ret_t ret = rcl_action_wait_set_add_action_server(
    &wait_set, pimpl_->action_server_.get(), NULL);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "ServerBase::add_to_wait_set() failed");
  }
}

bool
ServerBase::is_ready(const rcl_wait_set_t & wait_set)
{
  bool goal_request_ready;
  bool cancel_request_ready;
  bool result_request_ready;
  bool goal_expired;
  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_server_wait_set_get_entities_ready(
      &wait_set,
      pimpl_->action_server_.get(),
      &goal_request_ready,
      &cancel_request_ready,
      &result_request_ready,
      &goal_expired);
  }

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  pimpl_->next_ready_event = std::numeric_limits<uint32_t>::max();

  if (goal_request_ready) {
    pimpl_->next_ready_event = static_cast<uint32_t>(EntityType::GoalService);
    return true;
  }

  if (cancel_request_ready) {
    pimpl_->next_ready_event = static_cast<uint32_t>(EntityType::CancelService);
    return true;
  }

  if (result_request_ready) {
    pimpl_->next_ready_event = static_cast<uint32_t>(EntityType::ResultService);
    return true;
  }

  if (goal_expired) {
    pimpl_->next_ready_event = static_cast<uint32_t>(EntityType::Expired);
    return true;
  }

  return false;
}

std::shared_ptr<void>
ServerBase::take_data()
{
  size_t next_ready_event = pimpl_->next_ready_event.exchange(std::numeric_limits<uint32_t>::max());

  if (next_ready_event == std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("ServerBase::take_data() called but no data is ready");
  }

  return take_data_by_entity_id(next_ready_event);
}

std::shared_ptr<void>
ServerBase::take_data_by_entity_id(size_t id)
{
  std::shared_ptr<ServerBaseData> data_ptr;
  // Mark as ready the entity from which we want to take data
  switch (static_cast<EntityType>(id)) {
    case EntityType::GoalService:
      {
        rcl_ret_t ret;
        rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
        rmw_request_id_t request_header;

        std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

        std::shared_ptr<void> message = create_goal_request();
        ret = rcl_action_take_goal_request(
          pimpl_->action_server_.get(),
          &request_header,
          message.get());

        data_ptr = std::make_shared<ServerBaseData>(
          ServerBaseData::GoalRequestData(ret, goal_info, request_header, message));
      }
      break;
    case EntityType::ResultService:
      {
        rcl_ret_t ret;
        // Get the result request message
        rmw_request_id_t request_header;
        std::shared_ptr<void> result_request = create_result_request();
        std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
        ret = rcl_action_take_result_request(
          pimpl_->action_server_.get(), &request_header, result_request.get());

        data_ptr =
          std::make_shared<ServerBaseData>(
          ServerBaseData::ResultRequestData(ret, result_request, request_header));
      }
      break;
    case EntityType::CancelService:
      {
        rcl_ret_t ret;
        rmw_request_id_t request_header;

        // Initialize cancel request
        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

        std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
        ret = rcl_action_take_cancel_request(
          pimpl_->action_server_.get(),
          &request_header,
          request.get());

        data_ptr =
          std::make_shared<ServerBaseData>(
          ServerBaseData::CancelRequestData(ret, request, request_header));
      }
      break;
    case EntityType::Expired:
      {
        data_ptr =
          std::make_shared<ServerBaseData>(ServerBaseData::GoalExpiredData());
      }
      break;
  }

  return std::static_pointer_cast<void>(data_ptr);
}

void
ServerBase::execute(const std::shared_ptr<void> & data_in)
{
  if (!data_in) {
    throw std::runtime_error("ServerBase::execute: give data pointer was null");
  }

  std::shared_ptr<ServerBaseData> data_ptr = std::static_pointer_cast<ServerBaseData>(data_in);

  std::visit(
    [&](auto && data) -> void {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, ServerBaseData::GoalRequestData>) {
        execute_goal_request_received(
          std::get<0>(data), std::get<1>(data), std::get<2>(data),
          std::get<3>(data));
      }
      if constexpr (std::is_same_v<T, ServerBaseData::CancelRequestData>) {
        execute_cancel_request_received(std::get<0>(data), std::get<1>(data), std::get<2>(data));
      }
      if constexpr (std::is_same_v<T, ServerBaseData::ResultRequestData>) {
        execute_result_request_received(std::get<0>(data), std::get<1>(data), std::get<2>(data));
      }
      if constexpr (std::is_same_v<T, ServerBaseData::GoalExpiredData>) {
        execute_check_expired_goals();
      }
    },
    data_ptr->data);
}

void
ServerBase::execute_goal_request_received(
  rcl_ret_t ret,
  rcl_action_goal_info_t goal_info,
  rmw_request_id_t request_header,
  const std::shared_ptr<void> message)
{
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  GoalUUID uuid = get_goal_id_from_goal_request(message.get());
  convert(uuid, &goal_info);

  // Call user's callback, getting the user's response and a ros message to send back
  auto response_pair = call_handle_goal_callback(uuid, message);

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_send_goal_response(
      pimpl_->action_server_.get(),
      &request_header,
      response_pair.second.get());
  }

  if (RCL_RET_OK != ret) {
    if (ret == RCL_RET_TIMEOUT) {
      RCLCPP_WARN(
        pimpl_->logger_,
        "Failed to send goal response %s (timeout): %s",
        to_string(uuid).c_str(), rcl_get_error_string().str);
      rcl_reset_error();
      return;
    } else {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  const auto status = response_pair.first;

  // if goal is accepted, create a goal handle, and store it
  if (GoalResponse::ACCEPT_AND_EXECUTE == status || GoalResponse::ACCEPT_AND_DEFER == status) {
    RCLCPP_DEBUG(pimpl_->logger_, "Accepted goal %s", to_string(uuid).c_str());
    // rcl_action will set time stamp
    auto deleter = [](rcl_action_goal_handle_t * ptr)
      {
        if (nullptr != ptr) {
          rcl_ret_t fail_ret = rcl_action_goal_handle_fini(ptr);
          if (RCL_RET_OK != fail_ret) {
            RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp_action"),
              "failed to fini rcl_action_goal_handle_t in deleter");
          }
          delete ptr;
        }
      };
    rcl_action_goal_handle_t * rcl_handle;
    {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
      rcl_handle = rcl_action_accept_new_goal(pimpl_->action_server_.get(), &goal_info);
    }
    if (!rcl_handle) {
      throw std::runtime_error("Failed to accept new goal\n");
    }

    std::shared_ptr<rcl_action_goal_handle_t> handle(new rcl_action_goal_handle_t, deleter);
    // Copy out goal handle since action server storage disappears when it is fini'd
    *handle = *rcl_handle;

    {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);
      pimpl_->goal_handles_[uuid] = handle;
    }

    if (GoalResponse::ACCEPT_AND_EXECUTE == status) {
      // Change status to executing
      ret = rcl_action_update_goal_state(handle.get(), GOAL_EVENT_EXECUTE);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }
    // publish status since a goal's state has changed (was accepted or has begun execution)
    publish_status();

    // Tell user to start executing action
    call_goal_accepted_callback(handle, uuid, message);
  }
}

void
ServerBase::execute_cancel_request_received(
  rcl_ret_t ret,
  std::shared_ptr<action_msgs::srv::CancelGoal::Request> request,
  rmw_request_id_t request_header)
{
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // Convert c++ message to C message
  rcl_action_cancel_request_t cancel_request = rcl_action_get_zero_initialized_cancel_request();
  convert(request->goal_info.goal_id.uuid, &cancel_request.goal_info);
  cancel_request.goal_info.stamp.sec = request->goal_info.stamp.sec;
  cancel_request.goal_info.stamp.nanosec = request->goal_info.stamp.nanosec;

  // Get a list of goal info that should be attempted to be cancelled
  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_process_cancel_request(
      pimpl_->action_server_.get(),
      &cancel_request,
      &cancel_response);
  }

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  RCPPUTILS_SCOPE_EXIT(
  {
    ret = rcl_action_cancel_response_fini(&cancel_response);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini cancel response");
    }
  });

  auto response = std::make_shared<action_msgs::srv::CancelGoal::Response>();

  response->return_code = cancel_response.msg.return_code;
  auto & goals = cancel_response.msg.goals_canceling;
  // For each canceled goal, call cancel callback
  for (size_t i = 0; i < goals.size; ++i) {
    const rcl_action_goal_info_t & goal_info = goals.data[i];
    GoalUUID uuid;
    convert(goal_info, &uuid);
    auto response_code = call_handle_cancel_callback(uuid);
    if (CancelResponse::ACCEPT == response_code) {
      action_msgs::msg::GoalInfo cpp_info;
      cpp_info.goal_id.uuid = uuid;
      cpp_info.stamp.sec = goal_info.stamp.sec;
      cpp_info.stamp.nanosec = goal_info.stamp.nanosec;
      response->goals_canceling.push_back(cpp_info);
    }
  }

  // If the user rejects all individual requests to cancel goals,
  // then we consider the top-level cancel request as rejected.
  if (goals.size >= 1u && 0u == response->goals_canceling.size()) {
    response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_REJECTED;
  }

  if (!response->goals_canceling.empty()) {
    // at least one goal state changed, publish a new status message
    publish_status();
  }

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_send_cancel_response(
      pimpl_->action_server_.get(), &request_header, response.get());
  }

  if (ret == RCL_RET_TIMEOUT) {
    GoalUUID uuid = request->goal_info.goal_id.uuid;
    RCLCPP_WARN(
      pimpl_->logger_,
      "Failed to send cancel response %s (timeout): %s",
      to_string(uuid).c_str(), rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::execute_result_request_received(
  rcl_ret_t ret,
  std::shared_ptr<void> result_request,
  rmw_request_id_t request_header)
{
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  std::shared_ptr<void> result_response;

  // check if the goal exists
  GoalUUID uuid = get_goal_id_from_result_request(result_request.get());
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);
  bool goal_exists;
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);
  }
  if (!goal_exists) {
    // Goal does not exists
    result_response = create_result_response(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  } else {
    // Goal exists, check if a result is already available
    std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);
    auto iter = pimpl_->goal_results_.find(uuid);
    if (iter != pimpl_->goal_results_.end()) {
      result_response = iter->second;
    } else {
      // Store the request so it can be responded to later
      pimpl_->result_requests_[uuid].push_back(request_header);
    }
  }

  if (result_response) {
    // Send the result now
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    rcl_ret_t rcl_ret = rcl_action_send_result_response(
      pimpl_->action_server_.get(), &request_header, result_response.get());
    if (rcl_ret == RCL_RET_TIMEOUT) {
      RCLCPP_WARN(
        pimpl_->logger_,
        "Failed to send result response %s (timeout): %s",
        to_string(uuid).c_str(), rcl_get_error_string().str);
      rcl_reset_error();
      return;
    }
    if (RCL_RET_OK != rcl_ret) {
      rclcpp::exceptions::throw_from_rcl_error(rcl_ret);
    }
  }
}

void
ServerBase::execute_check_expired_goals()
{
  // Allocate expecting only one goal to expire at a time
  rcl_action_goal_info_t expired_goals[1];
  size_t num_expired = 1;

  // Loop in case more than 1 goal expired
  while (num_expired > 0u) {
    rcl_ret_t ret;
    {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
      ret = rcl_action_expire_goals(pimpl_->action_server_.get(), expired_goals, 1, &num_expired);
    }
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    } else if (num_expired) {
      // A goal expired!
      GoalUUID uuid;
      convert(expired_goals[0], &uuid);
      RCLCPP_DEBUG(pimpl_->logger_, "Expired goal %s", to_string(uuid).c_str());
      std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);
      pimpl_->goal_results_.erase(uuid);
      pimpl_->result_requests_.erase(uuid);
      pimpl_->goal_handles_.erase(uuid);
    }
  }
}

void
ServerBase::publish_status()
{
  rcl_ret_t ret;

  // We need to hold the lock across this entire method because
  // rcl_action_server_get_goal_handles() returns an internal pointer to the
  // goal data.
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

  // Get all goal handles known to C action server
  rcl_action_goal_handle_t ** goal_handles = NULL;
  size_t num_goals = 0;
  ret = rcl_action_server_get_goal_handles(
    pimpl_->action_server_.get(), &goal_handles, &num_goals);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  auto status_msg = std::make_shared<action_msgs::msg::GoalStatusArray>();
  status_msg->status_list.reserve(num_goals);
  // Populate a c++ status message with the goals and their statuses
  rcl_action_goal_status_array_t c_status_array =
    rcl_action_get_zero_initialized_goal_status_array();
  ret = rcl_action_get_goal_status_array(pimpl_->action_server_.get(), &c_status_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  RCPPUTILS_SCOPE_EXIT(
  {
    ret = rcl_action_goal_status_array_fini(&c_status_array);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini status array message");
    }
  });

  for (size_t i = 0; i < c_status_array.msg.status_list.size; ++i) {
    auto & c_status_msg = c_status_array.msg.status_list.data[i];

    action_msgs::msg::GoalStatus msg;
    msg.status = c_status_msg.status;
    // Convert C goal info to C++ goal info
    convert(c_status_msg.goal_info, &msg.goal_info.goal_id.uuid);
    msg.goal_info.stamp.sec = c_status_msg.goal_info.stamp.sec;
    msg.goal_info.stamp.nanosec = c_status_msg.goal_info.stamp.nanosec;

    status_msg->status_list.push_back(msg);
  }

  // Publish the message through the status publisher
  ret = rcl_action_publish_status(pimpl_->action_server_.get(), status_msg.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg)
{
  // Check that the goal exists
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);
  bool goal_exists;
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);
  }

  if (!goal_exists) {
    throw std::runtime_error("Asked to publish result for goal that does not exist");
  }

  {
    /**
    * NOTE: There is a potential deadlock issue if both unordered_map_mutex_ and
    * action_server_reentrant_mutex_ locked in other block scopes. Unless using
    * std::scoped_lock, locking order must be consistent with the current.
    *
    * Current locking order:
    *
    *   1. unordered_map_mutex_
    *   2. action_server_reentrant_mutex_
    *
    */
    std::lock_guard<std::recursive_mutex> unordered_map_lock(pimpl_->unordered_map_mutex_);
    pimpl_->goal_results_[uuid] = result_msg;

    // if there are clients who already asked for the result, send it to them
    auto iter = pimpl_->result_requests_.find(uuid);
    if (iter != pimpl_->result_requests_.end()) {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
      for (auto & request_header : iter->second) {
        rcl_ret_t ret = rcl_action_send_result_response(
          pimpl_->action_server_.get(), &request_header, result_msg.get());
        if (ret == RCL_RET_TIMEOUT) {
          RCLCPP_WARN(
            pimpl_->logger_,
            "Failed to send result response %s (timeout): %s",
            to_string(uuid).c_str(), rcl_get_error_string().str);
          rcl_reset_error();
        } else if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
      }
    }
  }
}

void
ServerBase::notify_goal_terminal_state()
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
  rcl_ret_t ret = rcl_action_notify_goal_done(pimpl_->action_server_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::publish_feedback(std::shared_ptr<void> feedback_msg)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
  rcl_ret_t ret = rcl_action_publish_feedback(pimpl_->action_server_.get(), feedback_msg.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to publish feedback");
  }
}

void
ServerBase::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  if (!callback) {
    throw std::invalid_argument(
            "The callback passed to set_on_ready_callback "
            "is not callable.");
  }

  pimpl_->start_expire_thread(callback);

  set_callback_to_entity(EntityType::GoalService, callback);
  set_callback_to_entity(EntityType::ResultService, callback);
  set_callback_to_entity(EntityType::CancelService, callback);
}

void
ServerBase::set_callback_to_entity(
  EntityType entity_type,
  std::function<void(size_t, int)> callback)
{
  // Note: we bind the int identifier argument to this waitable's entity types
  auto new_callback =
    [callback, entity_type, this](size_t number_of_events) {
      try {
        callback(number_of_events, static_cast<int>(entity_type));
      } catch (const std::exception & exception) {
        RCLCPP_ERROR_STREAM(
          pimpl_->logger_,
          "rclcpp_action::ServerBase@" << this <<
            " caught " << rmw::impl::cpp::demangle(exception) <<
            " exception in user-provided callback for the 'on ready' callback: " <<
            exception.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
          pimpl_->logger_,
          "rclcpp_action::ServerBase@" << this <<
            " caught unhandled exception in user-provided callback " <<
            "for the 'on ready' callback");
      }
    };


  // Set it temporarily to the new callback, while we replace the old one.
  // This two-step setting, prevents a gap where the old std::function has
  // been replaced but the middleware hasn't been told about the new one yet.
  set_on_ready_callback(
    entity_type,
    rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
    static_cast<const void *>(&new_callback));

  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);
  // Store the std::function to keep it in scope, also overwrites the existing one.
  auto it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    it->second = new_callback;
  } else {
    entity_type_to_on_ready_callback_.emplace(entity_type, new_callback);
  }

  // Set it again, now using the permanent storage.
  it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    auto & cb = it->second;
    set_on_ready_callback(
      entity_type,
      rclcpp::detail::cpp_callback_trampoline<decltype(it->second), const void *, size_t>,
      static_cast<const void *>(&cb));
  }

  on_ready_callback_set_ = true;
}

void
ServerBase::set_on_ready_callback(
  EntityType entity_type,
  rcl_event_callback_t callback,
  const void * user_data)
{
  rcl_ret_t ret = RCL_RET_ERROR;

  switch (entity_type) {
    case EntityType::GoalService:
      {
        ret = rcl_action_server_set_goal_service_callback(
          pimpl_->action_server_.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::ResultService:
      {
        ret = rcl_action_server_set_result_service_callback(
          pimpl_->action_server_.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::CancelService:
      {
        ret = rcl_action_server_set_cancel_service_callback(
          pimpl_->action_server_.get(),
          callback,
          user_data);
        break;
      }

    default:
      throw std::runtime_error("ServerBase::set_on_ready_callback: Unknown entity type.");
      break;
  }

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on ready callback for action client");
  }
}

void
ServerBase::clear_on_ready_callback()
{
  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);

  if (on_ready_callback_set_) {
    set_on_ready_callback(EntityType::GoalService, nullptr, nullptr);
    set_on_ready_callback(EntityType::ResultService, nullptr, nullptr);
    set_on_ready_callback(EntityType::CancelService, nullptr, nullptr);
    pimpl_->stop_expire_thread();
    on_ready_callback_set_ = false;
  }

  entity_type_to_on_ready_callback_.clear();
}
