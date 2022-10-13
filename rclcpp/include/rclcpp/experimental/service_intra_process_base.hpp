// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/context.hpp"
#include "rclcpp/experimental/client_intra_process_base.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace experimental
{

class ServiceIntraProcessBase : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(ServiceIntraProcessBase)

  enum class EntityType : std::size_t
  {
    Service,
  };

  RCLCPP_PUBLIC
  ServiceIntraProcessBase(
    rclcpp::Context::SharedPtr context,
    const std::string & service_name,
    const rclcpp::QoS & qos_profile)
  : gc_(context), service_name_(service_name), qos_profile_(qos_profile)
  {}

  virtual ~ServiceIntraProcessBase() = default;

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() {return 1;}

  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set);

  virtual bool
  is_ready(rcl_wait_set_t * wait_set) = 0;

  virtual
  std::shared_ptr<void>
  take_data() = 0;

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return take_data();
  }

  virtual void
  execute(std::shared_ptr<void> & data) = 0;

  RCLCPP_PUBLIC
  const char *
  get_service_name() const;

  RCLCPP_PUBLIC
  QoS
  get_actual_qos() const;

  RCLCPP_PUBLIC
  void
  add_intra_process_client(
    rclcpp::experimental::ClientIntraProcessBase::SharedPtr client,
    uint64_t client_id);

  /// Set a callback to be called when each new request arrives.
  /**
   * The callback receives a size_t which is the number of requests received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if requests were received before any
   * callback was set.
   *
   * The callback also receives an int identifier argument.
   * This is needed because a Waitable may be composed of several distinct entities,
   * such as subscriptions, services, etc.
   * The application should provide a generic callback function that will be then
   * forwarded by the waitable to all of its entities.
   * Before forwarding, a different value for the identifier argument will be
   * bound to the function.
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \param[in] callback functor to be called when a new request is received.
   */
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_ready_callback "
              "is not callable.");
    }

    // Note: we bind the int identifier argument to this waitable's entity types
    auto new_callback =
      [callback, this](size_t number_of_events) {
        try {
          callback(number_of_events, static_cast<int>(EntityType::Service));
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_STREAM(
            // TODO(wjwwood): get this class access to the node logger it is associated with
            rclcpp::get_logger("rclcpp"),
            "rclcpp::ServiceIntraProcessBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on ready' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "rclcpp::ServiceIntraProcessBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on ready' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
    on_new_request_callback_ = new_callback;

    if (unread_count_ > 0) {
      if (qos_profile_.history() == HistoryPolicy::KeepAll) {
        on_new_request_callback_(unread_count_);
      } else {
        // Use qos profile depth as upper bound for unread_count_
        on_new_request_callback_(std::min(unread_count_, qos_profile_.depth()));
      }
      unread_count_ = 0;
    }
  }

  /// Unset the callback registered for new messages, if any.
  void
  clear_on_ready_callback() override
  {
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
    on_new_request_callback_ = nullptr;
  }

protected:
  std::recursive_mutex reentrant_mutex_;
  rclcpp::GuardCondition gc_;
  std::function<void(size_t)> on_new_request_callback_ {nullptr};
  size_t unread_count_{0};

  void
  invoke_on_new_request()
  {
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
    if (on_new_request_callback_) {
      on_new_request_callback_(1);
    } else {
      unread_count_++;
    }
  }

  using ClientMap =
    std::unordered_map<uint64_t, rclcpp::experimental::ClientIntraProcessBase::WeakPtr>;

  ClientMap clients_;

private:
  std::string service_name_;
  QoS qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_
