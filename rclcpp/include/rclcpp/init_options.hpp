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

#ifndef RCLCPP__INIT_OPTIONS_HPP_
#define RCLCPP__INIT_OPTIONS_HPP_

#include <memory>

#include "rcl/init_options.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Encapsulation of options for initializing rclcpp.
class InitOptions
{
public:
  /// Constructor which allows you to specify the allocator used within the init options.
  RCLCPP_PUBLIC
  explicit
  InitOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Constructor which is initialized by an existing init_options.
  RCLCPP_PUBLIC
  explicit
  InitOptions(const rcl_init_options_t & init_options);

  /// Copy constructor.
  RCLCPP_PUBLIC
  InitOptions(const InitOptions & other);

  /// Assignment operator.
  RCLCPP_PUBLIC
  InitOptions &
  operator=(const InitOptions & other);

  RCLCPP_PUBLIC
  virtual
  ~InitOptions();

  /// Return the rcl init options.
  RCLCPP_PUBLIC
  const rcl_init_options_t *
  get_rcl_init_options() const;

protected:
  void
  finalize_init_options();

private:
  std::unique_ptr<rcl_init_options_t> init_options_;
};

}  // namespace rclcpp

#endif  // RCLCPP__INIT_OPTIONS_HPP_
