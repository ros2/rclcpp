// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/timer.hpp"

#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle
{

template<typename FunctorT>
class LifecycleGenericTimer : public SimpleManagedEntity,
  public rclcpp::GenericTimer<FunctorT> {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleGenericTimer)

  /// Default constructor.
  /**
   * \param[in] clock The clock providing the current time.
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   * \param[in] context custom context to be used.
   */
  explicit LifecycleGenericTimer(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context)
  : rclcpp::GenericTimer<FunctorT>(clock, period, std::move(callback),
      context, false) {}

  void on_activate() override
  {
    SimpleManagedEntity::on_activate();
    rclcpp::GenericTimer<FunctorT>::reset();
  }

  void on_deactivate() override
  {
    SimpleManagedEntity::on_deactivate();
    rclcpp::GenericTimer<FunctorT>::cancel();
  }

protected:
  RCLCPP_DISABLE_COPY(LifecycleGenericTimer)
};

template<typename FunctorT>
class LifecycleWallTimer : public LifecycleGenericTimer<FunctorT> {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleWallTimer)

  /// Default constructor.
  /**
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   * \param context node context
   */
  LifecycleWallTimer(
    std::chrono::nanoseconds period, FunctorT && callback,
    rclcpp::Context::SharedPtr context)
  : LifecycleGenericTimer<FunctorT>(
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), period,
      std::move(callback), context) {}

protected:
  RCLCPP_DISABLE_COPY(LifecycleWallTimer)
};

} // namespace rclcpp_lifecycle

#endif
