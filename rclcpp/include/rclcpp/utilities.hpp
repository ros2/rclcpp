/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_UTILITIES_HPP_
#define RCLCPP_RCLCPP_UTILITIES_HPP_

#include <iostream>

#include <csignal>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

namespace
{
  volatile sig_atomic_t g_signal_status = 0;
  ros_middleware_interface::GuardConditionHandle g_sigint_guard_cond_handle_;

  void signal_handler(int signal_value)
  {
    std::cout << "signal_handler(" << signal_value << ")" << std::endl;
    g_signal_status = signal_value;
    ros_middleware_interface::trigger_guard_condition(g_sigint_guard_cond_handle_);
  }
}

namespace rclcpp
{
namespace utilities
{

void init(int argc, char *argv[])
{
  ::g_sigint_guard_cond_handle_ = \
    ros_middleware_interface::create_guard_condition();
  std::signal(SIGINT, ::signal_handler);
}

bool ok()
{
  return ::g_signal_status == 0;
}

ros_middleware_interface::GuardConditionHandle get_global_sigint_guard_cond()
{
  return ::g_sigint_guard_cond_handle_;
}

} /* namespace utilities */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_UTILITIES_HPP_ */
