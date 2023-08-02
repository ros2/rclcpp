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

#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle
{

/// @brief Child class of rclcpp::WallTimer class that is aware of the lifecycle state.
/**
 * This class acts like a normal rclcpp::WallTimer, but it will not start until the
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
class LifecycleWallTimer : public SimpleManagedEntity, public rclcpp::GenericTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleWallTimer)

  /// Lifecycle wall timer constructor
  /**
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   * \param context node context
   */
  LifecycleWallTimer(
    std::chrono::nanoseconds period, FunctorT && callback, rclcpp::Context::SharedPtr context,
    bool autostart = true)
  : rclcpp::GenericTimer<FunctorT>(
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME),
      period,
      std::move(callback),
      context,
      false),
    autostart_(autostart)
  {
  }

  void on_activate() override
  {
    rclcpp::GenericTimer<FunctorT>::reset();
  }

  void on_deactivate() override
  {
    rclcpp::GenericTimer<FunctorT>::cancel();
  }

protected:
  RCLCPP_DISABLE_COPY(LifecycleWallTimer)

private:
  bool autostart_;

};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_TIMER_HPP_
