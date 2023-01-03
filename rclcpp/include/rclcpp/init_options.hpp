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
#include "rcpputils/mutex.hpp"

namespace rclcpp
{

/// Encapsulation of options for initializing rclcpp.
class InitOptions
{
public:
  /// If true, the context will be shutdown on SIGINT by the signal handler (if it was installed).
  bool shutdown_on_signal = true;

  /// Constructor
  /**
   * It allows you to specify the allocator used within the init options.
   * \param[in] allocator used allocate memory within the init options
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  explicit InitOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Constructor which is initialized by an existing init_options.
  /**
   * Initialized by an existing init_options.
   * \param[in] init_options rcl_init_options_t to initialized
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  explicit InitOptions(const rcl_init_options_t & init_options);

  /// Copy constructor.
  RCLCPP_PUBLIC
  InitOptions(const InitOptions & other);

  /// Return `true` if logging should be initialized when `rclcpp::Context::init` is called.
  RCLCPP_PUBLIC
  bool
  auto_initialize_logging() const;

  /// Set flag indicating if logging should be initialized or not.
  RCLCPP_PUBLIC
  InitOptions &
  auto_initialize_logging(bool initialize_logging);

  /// Assignment operator.
  RCLCPP_PUBLIC
  InitOptions &
  operator=(const InitOptions & other);

  RCLCPP_PUBLIC
  virtual
  ~InitOptions();

  /// Return the rcl init options.
  /**
   * \return the rcl init options.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  const rcl_init_options_t *
  get_rcl_init_options() const;

  /// Retrieve default domain id and set.
  RCLCPP_PUBLIC
  void
  use_default_domain_id();

  /// Set the domain id.
  RCLCPP_PUBLIC
  void
  set_domain_id(size_t domain_id);

  /// Return domain id.
  RCLCPP_PUBLIC
  size_t
  get_domain_id() const;

protected:
  void
  finalize_init_options();

private:
  void
  finalize_init_options_impl();

  mutable rcpputils::PIMutex init_options_mutex_;
  std::unique_ptr<rcl_init_options_t> init_options_;
  bool initialize_logging_{true};
};

}  // namespace rclcpp

#endif  // RCLCPP__INIT_OPTIONS_HPP_
