// Copyright 2025 Sony Group Corporation.
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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp_action/generic_client.hpp"

namespace rclcpp_action
{
GenericClient::GenericClient(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & action_name,
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib,
  const rosidl_action_type_support_t * action_typesupport_handle,
  const rcl_action_client_options_t & client_options)
: ClientBase(
    node_base, node_graph, node_logging, action_name,
    action_typesupport_handle,
    client_options),
  ts_lib_(std::move(typesupport_lib))
{
  auto goal_service_request_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->goal_service_type_support->request_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  goal_service_request_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(goal_service_request_type_support_intro->data);

  auto goal_service_response_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->goal_service_type_support->response_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  goal_service_response_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(goal_service_response_type_support_intro->data);

  auto result_service_request_type_support_intro = get_message_typesupport_handle(
      action_typesupport_handle->result_service_type_support->request_typesupport,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
  result_service_request_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(result_service_request_type_support_intro->data);

  auto result_service_response_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->result_service_type_support->response_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  result_service_response_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(result_service_response_type_support_intro->data);

  auto cancel_service_reponse_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->cancel_service_type_support->response_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  cancel_service_response_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(cancel_service_reponse_type_support_intro->data);

  auto feedback_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->feedback_message_type_support,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  feedback_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(feedback_type_support_intro->data);
}

std::shared_future<typename GenericClientGoalHandle::SharedPtr>
GenericClient::async_send_goal(
  const Goal goal, size_t goal_size, const SendGoalOptions & options)
{
  // uuid + goal
  size_t goal_request_size = sizeof(rclcpp_action::GoalUUID) + goal_size;
  std::shared_ptr<uint8_t> goal_request_msg(
    new uint8_t[goal_request_size], std::default_delete<uint8_t[]>());

  rclcpp_action::GoalUUID * uuid =
    reinterpret_cast<rclcpp_action::GoalUUID *>(goal_request_msg.get());
  *uuid = this->generate_goal_id();

  std::memcpy(
    goal_request_msg.get() + sizeof(rclcpp_action::GoalUUID),
    goal, goal_size);

  return async_send_goal(static_cast<void *>(goal_request_msg.get()), options);
}

std::shared_future<typename GenericClientGoalHandle::SharedPtr>
GenericClient::async_send_goal(
  const GoalRequest goal_request,
  const SendGoalOptions & options)
{
  // Put promise in the heap to move it around.
  auto promise = std::make_shared<std::promise<typename GenericClientGoalHandle::SharedPtr>>();
  std::shared_future<typename GenericClientGoalHandle::SharedPtr> future(promise->get_future());

  auto do_nothing = [](void *ptr) {
      (void)ptr;
    };
  std::shared_ptr<void> goal_request_msg(const_cast<void *>(goal_request), do_nothing);

  GoalUUID uuid = *reinterpret_cast<GoalUUID *>(goal_request);

  this->send_goal_request(
    goal_request_msg,
    [this, uuid, options, promise](std::shared_ptr<void> response) mutable
    {
      size_t response_accepted_offset = 0;
      size_t response_timestamp_offset = 0;
      for(uint32_t i = 0; i < goal_service_response_type_members_->member_count_; i++) {
        if (!std::strcmp(goal_service_response_type_members_->members_[i].name_, "accepted")) {
          response_accepted_offset = goal_service_response_type_members_->members_[i].offset_;
          continue;
        }
        if (!std::strcmp(goal_service_response_type_members_->members_[i].name_, "stamp")) {
          response_timestamp_offset = goal_service_response_type_members_->members_[i].offset_;
          continue;
        }
      }

      bool response_accepted = false;
      std::memcpy(
        static_cast<bool *>(&response_accepted),
        static_cast<bool *>(static_cast<bool *>(response.get()) + response_accepted_offset),
        sizeof(bool));

      if (!response_accepted) {
        promise->set_value(nullptr);
        if (options.goal_response_callback) {
          options.goal_response_callback(nullptr);
        }
        return;
      }

      action_msgs::msg::GoalInfo goal_info;
      goal_info.goal_id.uuid = uuid;
      std::memcpy(
        static_cast<void *>(&goal_info.stamp),
        static_cast<void *>(static_cast<char *>(response.get()) + response_timestamp_offset),
        sizeof(goal_info.stamp));

      // Do not use std::make_shared as friendship cannot be forwarded.
      std::shared_ptr<GenericClientGoalHandle> goal_handle(
        new GenericClientGoalHandle(
          goal_info, options.feedback_callback, options.result_callback));
      {
        std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
        goal_handles_[goal_handle->get_goal_id()] = goal_handle;
      }
      promise->set_value(goal_handle);
      if (options.goal_response_callback) {
        options.goal_response_callback(goal_handle);
      }

      if (options.result_callback) {
        this->make_result_aware(goal_handle);
      }
    });

  // TODO(jacobperron): Encapsulate into it's own function and
  //                    consider exposing an option to disable this cleanup
  // To prevent the list from growing out of control, forget about any goals
  // with no more user references
  {
    std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
    auto goal_handle_it = goal_handles_.begin();
    while (goal_handle_it != goal_handles_.end()) {
      if (!goal_handle_it->second.lock()) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Dropping weak reference to goal handle during send_goal()");
        goal_handle_it = goal_handles_.erase(goal_handle_it);
      } else {
        ++goal_handle_it;
      }
    }
  }

  return future;
}

std::shared_future<GenericClient::WrappedResult>
GenericClient::async_get_result(
  typename GenericClientGoalHandle::SharedPtr goal_handle,
  ResultCallback result_callback)
{
  std::lock_guard<std::recursive_mutex> lock(goal_handles_mutex_);
  if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
    throw exceptions::UnknownGoalHandleError();
  }
  if (goal_handle->is_invalidated()) {
    // This case can happen if there was a failure to send the result request
    // during the goal response callback
    throw goal_handle->invalidate_exception_;
  }
  if (result_callback) {
    // This will override any previously registered callback
    goal_handle->set_result_callback(result_callback);
  }
  this->make_result_aware(goal_handle);
  return goal_handle->async_get_result();
}

std::shared_future<typename GenericClient::CancelResponse::SharedPtr>
GenericClient::async_cancel_goal(
  typename GenericClientGoalHandle::SharedPtr goal_handle,
  CancelCallback cancel_callback)
{
  std::lock_guard<std::recursive_mutex> lock(goal_handles_mutex_);
  if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
    throw exceptions::UnknownGoalHandleError();
  }
  auto cancel_request = std::make_shared<CancelRequest>();
  cancel_request->goal_info.goal_id.uuid = goal_handle->get_goal_id();
  return async_cancel(cancel_request, cancel_callback);
}

std::shared_future<typename GenericClient::CancelResponse::SharedPtr>
GenericClient::async_cancel_all_goals(CancelCallback cancel_callback)
{
  auto cancel_request = std::make_shared<CancelRequest>();
  std::fill(
    cancel_request->goal_info.goal_id.uuid.begin(),
    cancel_request->goal_info.goal_id.uuid.end(), 0u);
  return async_cancel(cancel_request, cancel_callback);
}

void
GenericClient::stop_callbacks(typename GenericClientGoalHandle::SharedPtr goal_handle)
{
  goal_handle->set_feedback_callback(typename GenericClientGoalHandle::FeedbackCallback());
  goal_handle->set_result_callback(typename GenericClientGoalHandle::ResultCallback());

  std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
  const GoalUUID & goal_id = goal_handle->get_goal_id();
  auto it = goal_handles_.find(goal_id);
  if (goal_handles_.end() == it) {
    // someone else already deleted the entry
    // e.g. the result callback
    RCLCPP_DEBUG(
      this->get_logger(),
      "Given goal is unknown. Ignoring...");
    return;
  }
  goal_handles_.erase(it);
}

void GenericClient::stop_callbacks(const GoalUUID & goal_id)
{
  typename GenericClientGoalHandle::SharedPtr goal_handle;
  {
    std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.find(goal_id);
    if (goal_handles_.end() == it) {
      // someone else already deleted the entry
      // e.g. the result callback
      RCLCPP_DEBUG(
        this->get_logger(),
        "Given goal is unknown. Ignoring...");
      return;
    }

    goal_handle = it->second.lock();
  }

  if (goal_handle) {
    stop_callbacks(goal_handle);
  }
}

std::shared_future<typename GenericClient::CancelResponse::SharedPtr>
GenericClient::async_cancel_goals_before(
  const rclcpp::Time & stamp,
  CancelCallback cancel_callback)
{
  auto cancel_request = std::make_shared<CancelRequest>();
  // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
  std::fill(
    cancel_request->goal_info.goal_id.uuid.begin(),
    cancel_request->goal_info.goal_id.uuid.end(), 0u);
  cancel_request->goal_info.stamp = stamp;
  return async_cancel(cancel_request, cancel_callback);
}

GenericClient::~GenericClient()
{
  std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
  auto it = goal_handles_.begin();
  while (it != goal_handles_.end()) {
    typename GenericClientGoalHandle::SharedPtr goal_handle = it->second.lock();
    if (goal_handle) {
      goal_handle->invalidate(exceptions::UnawareGoalHandleError());
    }
    it = goal_handles_.erase(it);
  }
}

std::shared_ptr<void>
GenericClient::create_message(IntrospectionMessageMembersPtr message_members) const
{
  void * message = new uint8_t[message_members->size_of_];
  message_members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(
    message,
    [message_members](void * p)
    {
      message_members->fini_function(p);
      delete[] reinterpret_cast<uint8_t *>(p);
    });
}

void
GenericClient::handle_feedback_message(std::shared_ptr<void> message)
{
  size_t goal_id_offset = 0;
  size_t feedback_offset = 0;
  for (uint32_t i = 0; i < feedback_type_members_->member_count_; i++) {
    if (!strcmp(feedback_type_members_->members_[i].name_, "goal_id")) {
      goal_id_offset = feedback_type_members_->members_[i].offset_;
      continue;
    }
    if (!strcmp(feedback_type_members_->members_[i].name_, "feedback")) {
      feedback_offset = feedback_type_members_->members_[i].offset_;
      continue;
    }
  }

  auto * uuid = reinterpret_cast<unique_identifier_msgs::msg::UUID *>(
    static_cast<char *>(message.get()) + goal_id_offset);

  std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
  const GoalUUID & goal_id = uuid->uuid;
  if (goal_handles_.count(goal_id) == 0) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Received feedback for unknown goal. Ignoring...");
    return;
  }
  typename GenericClientGoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();
  // Forget about the goal if there are no more user references
  if (!goal_handle) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Dropping weak reference to goal handle during feedback callback");
    goal_handles_.erase(goal_id);
    return;
  }

  void * feedback = static_cast<void *>(reinterpret_cast<char *>(message.get()) + feedback_offset);
  goal_handle->call_feedback_callback(goal_handle, feedback);
}

void
GenericClient::handle_status_message(std::shared_ptr<void> message)
{
  std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
  using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  auto status_message = std::static_pointer_cast<GoalStatusMessage>(message);
  for (const GoalStatus & status : status_message->status_list) {
    const GoalUUID & goal_id = status.goal_info.goal_id.uuid;
    if (goal_handles_.count(goal_id) == 0) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received status for unknown goal. Ignoring...");
      continue;
    }
    typename GenericClientGoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();
    // Forget about the goal if there are no more user references
    if (!goal_handle) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Dropping weak reference to goal handle during status callback");
      goal_handles_.erase(goal_id);
      continue;
    }
    goal_handle->set_status(status.status);
  }
}

void
GenericClient::make_result_aware(typename GenericClientGoalHandle::SharedPtr goal_handle)
{
  // Avoid making more than one request
  if (goal_handle->set_result_awareness(true)) {
    return;
  }

  auto goal_result_request = create_result_request();

  size_t goal_id_offset = 0;
  for (uint32_t i = 0; i < goal_service_request_type_members_->member_count_; i++) {
    if (!strcmp(goal_service_request_type_members_->members_[i].name_, "goal_id")) {
      goal_id_offset = goal_service_request_type_members_->members_[i].offset_;
      break;
    }
  }

  // Set uuid for result request message
  auto * uuid = reinterpret_cast<unique_identifier_msgs::msg::UUID *>(
    static_cast<char *>(goal_result_request.get()) + goal_id_offset);
  uuid->uuid = goal_handle->get_goal_id();

  try {
    this->send_result_request(
      std::static_pointer_cast<void>(goal_result_request),
      [goal_handle, this](std::shared_ptr<void> response) mutable
      {
        // Wrap the response in a struct with the fields a user cares about
        WrappedResult wrapped_result;

        // Get the offsets for the status and result fields
        size_t status_offset{0};
        size_t result_offset{0};
        for (uint32_t i = 0; i < result_service_response_type_members_->member_count_; i++) {
          if (!std::strcmp(result_service_response_type_members_->members_[i].name_, "result")) {
            result_offset = result_service_response_type_members_->members_[i].offset_;
            continue;
          }
          if (!std::strcmp(result_service_response_type_members_->members_[i].name_, "status")) {
            status_offset = result_service_response_type_members_->members_[i].offset_;
            continue;
          }
        }

        // the result part will not be copied here since it is hard to quickly obtain the size of
        // the result part. Instead, a pointer will be used to reference the result part.
        wrapped_result.result = reinterpret_cast<char *>(response.get()) + result_offset;

        std::memcpy(
          static_cast<void *>(&wrapped_result.code),
          static_cast<void *>(reinterpret_cast<char *>(response.get()) + status_offset),
          sizeof(int8_t));  // ROS_TYPE_INT8

        wrapped_result.goal_id = goal_handle->get_goal_id();

        // Ensure that the original result response message is not released since result part
        // is not copied.
        wrapped_result.result_response = response;

        goal_handle->set_result(wrapped_result);

        std::lock_guard<std::recursive_mutex> lock(goal_handles_mutex_);
        goal_handles_.erase(goal_handle->get_goal_id());
      });
  } catch (rclcpp::exceptions::RCLError & ex) {
    // This will cause an exception when the user tries to access the result
    goal_handle->invalidate(exceptions::UnawareGoalHandleError(ex.message));
  }
}

std::shared_future<typename GenericClient::CancelResponse::SharedPtr>
GenericClient::async_cancel(
  typename CancelRequest::SharedPtr cancel_request,
  CancelCallback cancel_callback)
{
  // Put promise in the heap to move it around.
  auto promise = std::make_shared<std::promise<typename CancelResponse::SharedPtr>>();
  std::shared_future<typename CancelResponse::SharedPtr> future(promise->get_future());
  this->send_cancel_request(
    std::static_pointer_cast<void>(cancel_request),
    [cancel_callback, promise](std::shared_ptr<void> response) mutable
    {
      auto cancel_response = std::static_pointer_cast<CancelResponse>(response);
      promise->set_value(cancel_response);
      if (cancel_callback) {
        cancel_callback(cancel_response);
      }
    });
  return future;
}
}  // namespace rclcpp_action
