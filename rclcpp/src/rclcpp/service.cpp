// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/service.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::service::ServiceBase;

ServiceBase::ServiceBase(
  std::shared_ptr<rmw_node_t> node_handle,
  rmw_service_t * service_handle,
  const std::string service_name)
: node_handle_(node_handle), service_handle_(service_handle), service_name_(service_name)
{}

ServiceBase::~ServiceBase()
{
  if (service_handle_) {
    if (rmw_destroy_service(service_handle_) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "Error in destruction of rmw service_handle_ handle: " <<
        rmw_get_error_string_safe() << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
}

std::string
ServiceBase::get_service_name()
{
  return this->service_name_;
}

const rmw_service_t *
ServiceBase::get_service_handle()
{
  return this->service_handle_;
}
