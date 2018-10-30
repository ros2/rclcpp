// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/expand_topic_or_service_name.hpp"

#include <string>

#include "rcl/expand_topic_name.h"
#include "rcl/validate_topic_name.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rcutils/logging_macros.h"
#include "rcutils/types/string_map.h"
#include "rmw/error_handling.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"
#include "rmw/validate_full_topic_name.h"

using rclcpp::exceptions::throw_from_rcl_error;

std::string
rclcpp::expand_topic_or_service_name(
  const std::string & name,
  const std::string & node_name,
  const std::string & namespace_,
  bool is_service)
{
  char * expanded_topic = nullptr;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  rcutils_string_map_t substitutions_map = rcutils_get_zero_initialized_string_map();

  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_BAD_ALLOC) {
      throw_from_rcl_error(RCL_RET_BAD_ALLOC, "", rcutils_get_error_state(), rcutils_reset_error);
    } else {
      throw_from_rcl_error(RCL_RET_ERROR, "", rcutils_get_error_state(), rcutils_reset_error);
    }
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    const rcutils_error_state_t * error_state = rcl_get_error_state();
    // finalize the string map before throwing
    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "failed to fini string_map (%d) during error handling: %s",
        rcutils_ret,
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }
    throw_from_rcl_error(ret, "", error_state);
  }

  ret = rcl_expand_topic_name(
    name.c_str(),
    node_name.c_str(),
    namespace_.c_str(),
    &substitutions_map,
    allocator,
    &expanded_topic);

  std::string result;
  if (ret == RCL_RET_OK) {
    result = expanded_topic;
    allocator.deallocate(expanded_topic, allocator.state);
  }

  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    throw_from_rcl_error(RCL_RET_ERROR, "", rcutils_get_error_state(), rcutils_reset_error);
  }

  // expansion failed
  if (ret != RCL_RET_OK) {
    // if invalid topic or unknown substitution
    if (ret == RCL_RET_TOPIC_NAME_INVALID || ret == RCL_RET_UNKNOWN_SUBSTITUTION) {
      rcl_reset_error();  // explicitly discard error from rcl_expand_topic_name()
      int validation_result;
      size_t invalid_index;
      rcl_ret_t ret = rcl_validate_topic_name(name.c_str(), &validation_result, &invalid_index);
      if (ret != RCL_RET_OK) {
        throw_from_rcl_error(ret);
      }

      if (validation_result != RCL_TOPIC_NAME_VALID) {
        const char * validation_message =
          rcl_topic_name_validation_result_string(validation_result);
        if (is_service) {
          using rclcpp::exceptions::InvalidServiceNameError;
          throw InvalidServiceNameError(name.c_str(), validation_message, invalid_index);
        } else {
          using rclcpp::exceptions::InvalidTopicNameError;
          throw InvalidTopicNameError(name.c_str(), validation_message, invalid_index);
        }
      } else {
        throw std::runtime_error("topic name unexpectedly valid");
      }

      // if invalid node name
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      rcl_reset_error();  // explicitly discard error from rcl_expand_topic_name()
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_node_name(node_name.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(
            RCL_RET_INVALID_ARGUMENT, "failed to validate node name",
            rmw_get_error_state(), rmw_reset_error);
        }
        throw_from_rcl_error(
          RCL_RET_ERROR, "failed to validate node name",
          rmw_get_error_state(), rmw_reset_error);
      }

      if (validation_result != RMW_NODE_NAME_VALID) {
        throw rclcpp::exceptions::InvalidNodeNameError(
                node_name.c_str(),
                rmw_node_name_validation_result_string(validation_result),
                invalid_index);
      } else {
        throw std::runtime_error("invalid rcl node name but valid rmw node name");
      }

      // if invalid namespace
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      rcl_reset_error();  // explicitly discard error from rcl_expand_topic_name()
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_namespace(namespace_.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(
            RCL_RET_INVALID_ARGUMENT, "failed to validate namespace",
            rmw_get_error_state(), rmw_reset_error);
        }
        throw_from_rcl_error(
          RCL_RET_ERROR, "failed to validate namespace",
          rmw_get_error_state(), rmw_reset_error);
      }

      if (validation_result != RMW_NAMESPACE_VALID) {
        throw rclcpp::exceptions::InvalidNamespaceError(
                namespace_.c_str(),
                rmw_namespace_validation_result_string(validation_result),
                invalid_index);
      } else {
        throw std::runtime_error("invalid rcl namespace but valid rmw namespace");
      }
      // something else happened
    } else {
      throw_from_rcl_error(ret);
    }
  }

  // expand succeeded, but full name validation may fail still
  int validation_result;
  size_t invalid_index;
  rmw_ret_t rmw_ret =
    rmw_validate_full_topic_name(result.c_str(), &validation_result, &invalid_index);
  if (rmw_ret != RMW_RET_OK) {
    if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
      throw_from_rcl_error(
        RCL_RET_INVALID_ARGUMENT, "failed to validate full topic name",
        rmw_get_error_state(), rmw_reset_error);
    }
    throw_from_rcl_error(
      RCL_RET_ERROR, "failed to validate full topic name",
      rmw_get_error_state(), rmw_reset_error);
  }

  if (validation_result != RMW_TOPIC_VALID) {
    if (is_service) {
      throw rclcpp::exceptions::InvalidServiceNameError(
              result.c_str(),
              rmw_full_topic_name_validation_result_string(validation_result),
              invalid_index);
    } else {
      throw rclcpp::exceptions::InvalidTopicNameError(
              result.c_str(),
              rmw_full_topic_name_validation_result_string(validation_result),
              invalid_index);
    }
  }

  return result;
}
