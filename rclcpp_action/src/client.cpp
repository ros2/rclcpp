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

#include <algorithm>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "rcl_action/action_client.h"
#include "rcl_action/wait.h"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action
{

struct ClientBaseData
{
  struct FeedbackReadyData
  {
    FeedbackReadyData(rcl_ret_t retIn, std::shared_ptr<void> msg)
    : ret(retIn), feedback_message(msg) {}
    rcl_ret_t ret;
    std::shared_ptr<void> feedback_message;
  };
  struct StatusReadyData
  {
    StatusReadyData(rcl_ret_t retIn, std::shared_ptr<void> msg)
    : ret(retIn), status_message(msg) {}
    rcl_ret_t ret;
    std::shared_ptr<void> status_message;
  };
  struct GoalResponseData
  {
    GoalResponseData(rcl_ret_t retIn, rmw_request_id_t header, std::shared_ptr<void> response)
    : ret(retIn), response_header(header), goal_response(response) {}
    rcl_ret_t ret;
    rmw_request_id_t response_header;
    std::shared_ptr<void> goal_response;
  };
  struct CancelResponseData
  {
    CancelResponseData(rcl_ret_t retIn, rmw_request_id_t header, std::shared_ptr<void> response)
    : ret(retIn), response_header(header), cancel_response(response) {}
    rcl_ret_t ret;
    rmw_request_id_t response_header;
    std::shared_ptr<void> cancel_response;
  };
  struct ResultResponseData
  {
    ResultResponseData(rcl_ret_t retIn, rmw_request_id_t header, std::shared_ptr<void> response)
    : ret(retIn), response_header(header), result_response(response) {}
    rcl_ret_t ret;
    rmw_request_id_t response_header;
    std::shared_ptr<void> result_response;
  };

  std::variant<
    FeedbackReadyData,
    StatusReadyData,
    GoalResponseData,
    CancelResponseData,
    ResultResponseData
  > data;

  explicit ClientBaseData(FeedbackReadyData && data_in)
  : data(std::move(data_in)) {}
  explicit ClientBaseData(StatusReadyData && data_in)
  : data(std::move(data_in)) {}
  explicit ClientBaseData(GoalResponseData && data_in)
  : data(std::move(data_in)) {}
  explicit ClientBaseData(CancelResponseData && data_in)
  : data(std::move(data_in)) {}
  explicit ClientBaseData(ResultResponseData && data_in)
  : data(std::move(data_in)) {}
};

class ClientBaseImpl
{
public:
  ClientBaseImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & client_options)
  : node_graph_(node_graph),
    node_handle(node_base->get_shared_rcl_node_handle()),
    logger(node_logging->get_logger().get_child("rclcpp_action")),
    random_bytes_generator(std::random_device{}())
  {
    std::weak_ptr<rcl_node_t> weak_node_handle(node_handle);
    client_handle = std::shared_ptr<rcl_action_client_t>(
      new rcl_action_client_t, [weak_node_handle](rcl_action_client_t * client)
      {
        auto handle = weak_node_handle.lock();
        if (handle) {
          if (RCL_RET_OK != rcl_action_client_fini(client, handle.get())) {
            RCLCPP_ERROR(
              rclcpp::get_logger(rcl_node_get_logger_name(handle.get())).get_child("rclcpp_action"),
              "Error in destruction of rcl action client handle: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp_action"),
            "Error in destruction of rcl action client handle: "
            "the Node Handle was destructed too early. You will leak memory");
        }
        delete client;
      });
    *client_handle = rcl_action_get_zero_initialized_client();
    rcl_ret_t ret = rcl_action_client_init(
      client_handle.get(), node_handle.get(), type_support,
      action_name.c_str(), &client_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not initialize rcl action client");
    }

    ret = rcl_action_client_wait_set_get_num_entities(
      client_handle.get(),
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not retrieve rcl action client details");
    }
  }

  size_t num_subscriptions{0u};
  size_t num_guard_conditions{0u};
  size_t num_timers{0u};
  size_t num_clients{0u};
  size_t num_services{0u};

  // Lock for action_client_
  std::recursive_mutex action_client_mutex_;

  // next ready event for taking, will be set by is_ready and will be processed by take_data
  std::atomic<size_t> next_ready_event;

  rclcpp::Context::SharedPtr context_;
  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  // node_handle must be destroyed after client_handle to prevent memory leak
  std::shared_ptr<rcl_node_t> node_handle{nullptr};
  std::shared_ptr<rcl_action_client_t> client_handle{nullptr};
  rclcpp::Logger logger;

  using ResponseCallback = std::function<void (std::shared_ptr<void> response)>;

  std::map<int64_t, ResponseCallback> pending_goal_responses;
  std::mutex goal_requests_mutex;

  std::map<int64_t, ResponseCallback> pending_result_responses;
  std::mutex result_requests_mutex;

  std::map<int64_t, ResponseCallback> pending_cancel_responses;
  std::mutex cancel_requests_mutex;

  std::independent_bits_engine<
    std::mt19937, 8, unsigned int> random_bytes_generator;
};

ClientBase::ClientBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & action_name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_client_options_t & client_options)
: pimpl_(new ClientBaseImpl(
      node_base, node_graph, node_logging, action_name, type_support, client_options))
{
}

ClientBase::~ClientBase()
{
}

bool
ClientBase::action_server_is_ready() const
{
  bool is_ready;
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
  rcl_ret_t ret = rcl_action_server_is_available(
    this->pimpl_->node_handle.get(),
    this->pimpl_->client_handle.get(),
    &is_ready);
  if (RCL_RET_NODE_INVALID == ret) {
    const rcl_node_t * node_handle = this->pimpl_->node_handle.get();
    if (node_handle && !rcl_context_is_valid(node_handle->context)) {
      // context is shutdown, do a soft failure
      return false;
    }
  }
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "rcl_action_server_is_available failed");
  }
  return is_ready;
}

bool
ClientBase::wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  auto node_ptr = pimpl_->node_graph_.lock();
  if (!node_ptr) {
    throw rclcpp::exceptions::InvalidNodeError();
  }
  // check to see if the server is ready immediately
  if (this->action_server_is_ready()) {
    return true;
  }
  // make an event to reuse, rather than create a new one each time
  auto event = node_ptr->get_graph_event();
  if (timeout == std::chrono::nanoseconds(0)) {
    // check was non-blocking, return immediately
    return false;
  }
  // update the time even on the first loop to account for time spent in the first call
  // to this->server_is_ready()
  std::chrono::nanoseconds time_to_wait =
    timeout > std::chrono::nanoseconds(0) ?
    timeout - (std::chrono::steady_clock::now() - start) :
    std::chrono::nanoseconds::max();
  if (time_to_wait < std::chrono::nanoseconds(0)) {
    // Do not allow the time_to_wait to become negative when timeout was originally positive.
    // Setting time_to_wait to 0 will allow one non-blocking wait because of the do-while.
    time_to_wait = std::chrono::nanoseconds(0);
  }
  do {
    if (!rclcpp::ok(this->pimpl_->context_)) {
      return false;
    }
    // Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up
    // (see https://github.com/ros2/rmw_connext/issues/201)
    // If no other graph events occur, the wait set will not be triggered again until the timeout
    // has been reached, despite the service being available, so we artificially limit the wait
    // time to limit the delay.
    node_ptr->wait_for_graph_change(
      event, std::min(time_to_wait, std::chrono::nanoseconds(RCL_MS_TO_NS(100))));
    // Because of the aforementioned race condition, we check if the service is ready even if the
    // graph event wasn't triggered.
    event->check_and_clear();
    if (this->action_server_is_ready()) {
      return true;
    }
    // server is not ready, loop if there is time left
    if (timeout > std::chrono::nanoseconds(0)) {
      time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
    }
    // if timeout is negative, time_to_wait will never reach zero
  } while (time_to_wait > std::chrono::nanoseconds(0));
  return false;  // timeout exceeded while waiting for the server to be ready
}

rclcpp::Logger
ClientBase::get_logger()
{
  return pimpl_->logger;
}

size_t
ClientBase::get_number_of_ready_subscriptions()
{
  return pimpl_->num_subscriptions;
}

size_t
ClientBase::get_number_of_ready_guard_conditions()
{
  return pimpl_->num_guard_conditions;
}

size_t
ClientBase::get_number_of_ready_timers()
{
  return pimpl_->num_timers;
}

size_t
ClientBase::get_number_of_ready_clients()
{
  return pimpl_->num_clients;
}

size_t
ClientBase::get_number_of_ready_services()
{
  return pimpl_->num_services;
}

void
ClientBase::add_to_wait_set(rcl_wait_set_t & wait_set)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
  rcl_ret_t ret = rcl_action_wait_set_add_action_client(
    &wait_set, pimpl_->client_handle.get(), nullptr, nullptr);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "ClientBase::add_to_wait_set() failed");
  }
}

bool
ClientBase::is_ready(const rcl_wait_set_t & wait_set)
{
  bool is_feedback_ready{false};
  bool is_status_ready{false};
  bool is_goal_response_ready{false};
  bool is_cancel_response_ready{false};
  bool is_result_response_ready{false};

  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
    ret = rcl_action_client_wait_set_get_entities_ready(
      &wait_set, pimpl_->client_handle.get(),
      &is_feedback_ready,
      &is_status_ready,
      &is_goal_response_ready,
      &is_cancel_response_ready,
      &is_result_response_ready);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to check for any ready entities");
    }
  }

  pimpl_->next_ready_event = std::numeric_limits<size_t>::max();

  if (is_feedback_ready) {
    pimpl_->next_ready_event = static_cast<size_t>(EntityType::FeedbackSubscription);
    return true;
  }

  if (is_status_ready) {
    pimpl_->next_ready_event = static_cast<size_t>(EntityType::StatusSubscription);
    return true;
  }

  if (is_goal_response_ready) {
    pimpl_->next_ready_event = static_cast<size_t>(EntityType::GoalClient);
    return true;
  }

  if (is_result_response_ready) {
    pimpl_->next_ready_event = static_cast<size_t>(EntityType::ResultClient);
    return true;
  }

  if (is_cancel_response_ready) {
    pimpl_->next_ready_event = static_cast<size_t>(EntityType::CancelClient);
    return true;
  }

  return false;
}

void
ClientBase::handle_goal_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> guard(pimpl_->goal_requests_mutex);
  const int64_t & sequence_number = response_header.sequence_number;
  if (pimpl_->pending_goal_responses.count(sequence_number) == 0) {
    RCLCPP_ERROR(pimpl_->logger, "unknown goal response, ignoring...");
    return;
  }
  pimpl_->pending_goal_responses[sequence_number](response);
  pimpl_->pending_goal_responses.erase(sequence_number);
}

void
ClientBase::send_goal_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::unique_lock<std::mutex> guard(pimpl_->goal_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_goal_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send goal request");
  }
  assert(pimpl_->pending_goal_responses.count(sequence_number) == 0);
  pimpl_->pending_goal_responses[sequence_number] = callback;
}

void
ClientBase::handle_result_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::map<int64_t, ResponseCallback>::node_type pending_result_response;
  {
    std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);
    const int64_t & sequence_number = response_header.sequence_number;
    if (pimpl_->pending_result_responses.count(sequence_number) == 0) {
      RCLCPP_ERROR(pimpl_->logger, "unknown result response, ignoring...");
      return;
    }
    pending_result_response =
      pimpl_->pending_result_responses.extract(sequence_number);
  }
  auto & response_callback = pending_result_response.mapped();
  response_callback(response);
}

void
ClientBase::send_result_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_result_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send result request");
  }
  assert(pimpl_->pending_result_responses.count(sequence_number) == 0);
  pimpl_->pending_result_responses[sequence_number] = callback;
}

void
ClientBase::handle_cancel_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);
  const int64_t & sequence_number = response_header.sequence_number;
  if (pimpl_->pending_cancel_responses.count(sequence_number) == 0) {
    RCLCPP_ERROR(pimpl_->logger, "unknown cancel response, ignoring...");
    return;
  }
  pimpl_->pending_cancel_responses[sequence_number](response);
  pimpl_->pending_cancel_responses.erase(sequence_number);
}

void
ClientBase::send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_cancel_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send cancel request");
  }
  assert(pimpl_->pending_cancel_responses.count(sequence_number) == 0);
  pimpl_->pending_cancel_responses[sequence_number] = callback;
}

GoalUUID
ClientBase::generate_goal_id()
{
  GoalUUID goal_id;
  // TODO(hidmic): Do something better than this for UUID generation.
  // std::generate(
  //   goal_id.uuid.begin(), goal_id.uuid.end(),
  //   std::ref(pimpl_->random_bytes_generator));
  std::generate(
    goal_id.begin(), goal_id.end(),
    std::ref(pimpl_->random_bytes_generator));
  return goal_id;
}

void
ClientBase::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  if (!callback) {
    throw std::invalid_argument(
            "The callback passed to set_on_ready_callback "
            "is not callable.");
  }

  set_callback_to_entity(EntityType::GoalClient, callback);
  set_callback_to_entity(EntityType::ResultClient, callback);
  set_callback_to_entity(EntityType::CancelClient, callback);
  set_callback_to_entity(EntityType::FeedbackSubscription, callback);
  set_callback_to_entity(EntityType::StatusSubscription, callback);
}

void
ClientBase::set_callback_to_entity(
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
          pimpl_->logger,
          "rclcpp_action::ClientBase@" << this <<
            " caught " << rmw::impl::cpp::demangle(exception) <<
            " exception in user-provided callback for the 'on ready' callback: " <<
            exception.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
          pimpl_->logger,
          "rclcpp_action::ClientBase@" << this <<
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
ClientBase::set_on_ready_callback(
  EntityType entity_type,
  rcl_event_callback_t callback,
  const void * user_data)
{
  rcl_ret_t ret = RCL_RET_ERROR;

  switch (entity_type) {
    case EntityType::GoalClient:
      {
        ret = rcl_action_client_set_goal_client_callback(
          pimpl_->client_handle.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::ResultClient:
      {
        ret = rcl_action_client_set_result_client_callback(
          pimpl_->client_handle.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::CancelClient:
      {
        ret = rcl_action_client_set_cancel_client_callback(
          pimpl_->client_handle.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::FeedbackSubscription:
      {
        ret = rcl_action_client_set_feedback_subscription_callback(
          pimpl_->client_handle.get(),
          callback,
          user_data);
        break;
      }

    case EntityType::StatusSubscription:
      {
        ret = rcl_action_client_set_status_subscription_callback(
          pimpl_->client_handle.get(),
          callback,
          user_data);
        break;
      }

    default:
      throw std::runtime_error("ClientBase::set_on_ready_callback: Unknown entity type.");
      break;
  }

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on ready callback for action client");
  }
}

void
ClientBase::clear_on_ready_callback()
{
  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);

  if (on_ready_callback_set_) {
    set_on_ready_callback(EntityType::GoalClient, nullptr, nullptr);
    set_on_ready_callback(EntityType::ResultClient, nullptr, nullptr);
    set_on_ready_callback(EntityType::CancelClient, nullptr, nullptr);
    set_on_ready_callback(EntityType::FeedbackSubscription, nullptr, nullptr);
    set_on_ready_callback(EntityType::StatusSubscription, nullptr, nullptr);
    on_ready_callback_set_ = false;
  }

  entity_type_to_on_ready_callback_.clear();
}

std::shared_ptr<void>
ClientBase::take_data()
{
  // next_ready_event is an atomic, caching localy
  size_t next_ready_event = pimpl_->next_ready_event.exchange(std::numeric_limits<uint32_t>::max());

  if (next_ready_event == std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("Taking data from action client but nothing is ready");
  }

  return take_data_by_entity_id(next_ready_event);
}

std::shared_ptr<void>
ClientBase::take_data_by_entity_id(size_t id)
{
  std::shared_ptr<ClientBaseData> data_ptr;
  rcl_ret_t ret;

  // Mark as ready the entity from which we want to take data
  switch (static_cast<EntityType>(id)) {
    case EntityType::GoalClient:
      {
        rmw_request_id_t response_header;
        std::shared_ptr<void> goal_response;
        {
          std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);

          goal_response = this->create_goal_response();
          ret = rcl_action_take_goal_response(
            pimpl_->client_handle.get(), &response_header, goal_response.get());
        }
        data_ptr = std::make_shared<ClientBaseData>(
          ClientBaseData::GoalResponseData(
            ret, response_header, goal_response));
      }
      break;
    case EntityType::ResultClient:
      {
        rmw_request_id_t response_header;
        std::shared_ptr<void> result_response;
        {
          std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
          result_response = this->create_result_response();
          ret = rcl_action_take_result_response(
            pimpl_->client_handle.get(), &response_header, result_response.get());
        }
        data_ptr =
          std::make_shared<ClientBaseData>(
          ClientBaseData::ResultResponseData(
            ret, response_header, result_response));
      }
      break;
    case EntityType::CancelClient:
      {
        rmw_request_id_t response_header;
        std::shared_ptr<void> cancel_response;
        {
          std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
          cancel_response = this->create_cancel_response();
          ret = rcl_action_take_cancel_response(
            pimpl_->client_handle.get(), &response_header, cancel_response.get());
        }
        data_ptr =
          std::make_shared<ClientBaseData>(
          ClientBaseData::CancelResponseData(
            ret, response_header, cancel_response));
      }
      break;
    case EntityType::FeedbackSubscription:
      {
        std::shared_ptr<void> feedback_message;
        {
          std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
          feedback_message = this->create_feedback_message();
          ret = rcl_action_take_feedback(
            pimpl_->client_handle.get(), feedback_message.get());
        }
        data_ptr =
          std::make_shared<ClientBaseData>(
          ClientBaseData::FeedbackReadyData(
            ret, feedback_message));
      }
      break;
    case EntityType::StatusSubscription:
      {
        std::shared_ptr<void> status_message;
        {
          std::lock_guard<std::recursive_mutex> lock(pimpl_->action_client_mutex_);
          status_message = this->create_status_message();
          ret = rcl_action_take_status(
            pimpl_->client_handle.get(), status_message.get());
        }
        data_ptr =
          std::make_shared<ClientBaseData>(
          ClientBaseData::StatusReadyData(
            ret, status_message));
      }
      break;
  }

  return std::static_pointer_cast<void>(data_ptr);
}

void
ClientBase::execute(const std::shared_ptr<void> & data_in)
{
  if (!data_in) {
    throw std::invalid_argument("'data_in' is unexpectedly empty");
  }

  std::shared_ptr<ClientBaseData> data_ptr = std::static_pointer_cast<ClientBaseData>(data_in);

  std::visit(
    [&](auto && data) -> void {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, ClientBaseData::FeedbackReadyData>) {
        if (RCL_RET_OK == data.ret) {
          this->handle_feedback_message(data.feedback_message);
        } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != data.ret) {
          rclcpp::exceptions::throw_from_rcl_error(data.ret, "error taking feedback");
        }
      }
      if constexpr (std::is_same_v<T, ClientBaseData::StatusReadyData>) {
        if (RCL_RET_OK == data.ret) {
          this->handle_status_message(data.status_message);
        } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != data.ret) {
          rclcpp::exceptions::throw_from_rcl_error(data.ret, "error taking status");
        }
      }
      if constexpr (std::is_same_v<T, ClientBaseData::GoalResponseData>) {
        if (RCL_RET_OK == data.ret) {
          this->handle_goal_response(data.response_header, data.goal_response);
        } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != data.ret) {
          rclcpp::exceptions::throw_from_rcl_error(data.ret, "error taking goal response");
        }
      }
      if constexpr (std::is_same_v<T, ClientBaseData::ResultResponseData>) {
        if (RCL_RET_OK == data.ret) {
          this->handle_result_response(data.response_header, data.result_response);
        } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != data.ret) {
          rclcpp::exceptions::throw_from_rcl_error(data.ret, "error taking result response");
        }
      }
      if constexpr (std::is_same_v<T, ClientBaseData::CancelResponseData>) {
        if (RCL_RET_OK == data.ret) {
          this->handle_cancel_response(data.response_header, data.cancel_response);
        } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != data.ret) {
          rclcpp::exceptions::throw_from_rcl_error(data.ret, "error taking cancel response");
        }
      }
    }, data_ptr->data);
}

}  // namespace rclcpp_action
