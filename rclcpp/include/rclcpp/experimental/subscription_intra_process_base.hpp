// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "rcl/wait.h"
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace experimental
{

class SubscriptionIntraProcessBase : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(SubscriptionIntraProcessBase)

  enum class EntityType : std::size_t
  {
    Subscription,
  };

  RCLCPP_PUBLIC
  SubscriptionIntraProcessBase(
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile)
  : gc_(context), topic_name_(topic_name), qos_profile_(qos_profile)
  {}

  RCLCPP_PUBLIC
  virtual ~SubscriptionIntraProcessBase();

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  bool
  is_ready(rcl_wait_set_t * wait_set) override = 0;

  std::shared_ptr<void>
  take_data() override = 0;

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return take_data();
  }

  void
  execute(std::shared_ptr<void> & data) override = 0;

  virtual
  bool
  use_take_shared_method() const = 0;

  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  RCLCPP_PUBLIC
  QoS
  get_actual_qos() const;

  /// Set a callback to be called when each new message arrives.
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
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
   * \param[in] callback functor to be called when a new message is received.
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
          callback(number_of_events, static_cast<int>(EntityType::Subscription));
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_STREAM(
            // TODO(wjwwood): get this class access to the node logger it is associated with
            rclcpp::get_logger("rclcpp"),
            "rclcpp::SubscriptionIntraProcessBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on ready' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "rclcpp::SubscriptionIntraProcessBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on ready' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    on_new_message_callback_ = new_callback;

    if (unread_count_ > 0) {
      if (qos_profile_.history() == HistoryPolicy::KeepAll) {
        on_new_message_callback_(unread_count_);
      } else {
        // Use qos profile depth as upper bound for unread_count_
        on_new_message_callback_(std::min(unread_count_, qos_profile_.depth()));
      }
      unread_count_ = 0;
    }
  }

  /// Unset the callback registered for new messages, if any.
  void
  clear_on_ready_callback() override
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    on_new_message_callback_ = nullptr;
  }

protected:
  std::recursive_mutex callback_mutex_;
  std::function<void(size_t)> on_new_message_callback_ {nullptr};
  size_t unread_count_{0};
  rclcpp::GuardCondition gc_;

  virtual void
  trigger_guard_condition() = 0;

  void
  invoke_on_new_message()
  {
    std::lock_guard<std::recursive_mutex> lock(this->callback_mutex_);
    if (this->on_new_message_callback_) {
      this->on_new_message_callback_(1);
    } else {
      this->unread_count_++;
    }
  }

private:
  std::string topic_name_;
  QoS qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
