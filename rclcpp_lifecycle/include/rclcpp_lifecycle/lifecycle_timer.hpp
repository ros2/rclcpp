// Copyright 2023 Elroy Air, Inc.
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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_TIMER_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_TIMER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle
{

/// @brief Timer class that is aware of the lifecycle state.
/**
 * This class acts like a normal rclcpp::GenericTimer, but it will not start until the
 * lifecycle node is in the "active" state. If the lifecycle node transitions
 * out of the "active" state, the timer will be canceled.
 */
using VoidCallbackType = std::function<void ()>;
using TimerCallbackType = std::function<void (rclcpp::TimerBase &)>;

template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value>::type * = nullptr>
class LifecycleTimer : public SimpleManagedEntity, public rclcpp::GenericTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleTimer)

  /// Lifecycle wall timer constructor
  /**
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   * \param context node context
   */
  LifecycleTimer(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context,
    bool autostart = true)
  : rclcpp::GenericTimer<FunctorT>(
      clock,
      period,
      std::move(callback),
      context,
      false),
    autostart_(autostart)
  {
  }

  void on_activate() override
  {
    SimpleManagedEntity::on_activate();
    if (autostart_) {
      rclcpp::GenericTimer<FunctorT>::reset();
    }
  }

  void on_deactivate() override
  {
    SimpleManagedEntity::on_deactivate();
    rclcpp::GenericTimer<FunctorT>::cancel();
  }

protected:
  RCLCPP_DISABLE_COPY(LifecycleTimer)

private:
  bool autostart_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_TIMER_HPP_
