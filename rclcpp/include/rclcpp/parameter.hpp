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

#ifndef RCLCPP_RCLCPP_PARAMETER_HPP_
#define RCLCPP_RCLCPP_PARAMETER_HPP_

#include <string>

#include <rmw/rmw.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>

#include <rcl_interfaces/GetParameters.h>
#include <rcl_interfaces/GetParameterTypes.h>
#include <rcl_interfaces/Parameter.h>
#include <rcl_interfaces/ParameterDescriptor.h>
#include <rcl_interfaces/ParameterType.h>
#include <rcl_interfaces/SetParameters.h>
#include <rcl_interfaces/SetParametersAtomically.h>
#include <rcl_interfaces/ListParameters.h>
#include <rcl_interfaces/DescribeParameters.h>

namespace rclcpp
{

namespace parameter
{

enum ParameterType {
  PARAMETER_NOT_SET=rcl_interfaces::ParameterType::PARAMETER_NOT_SET,
  PARAMETER_BOOL=rcl_interfaces::ParameterType::PARAMETER_BOOL,
  PARAMETER_INTEGER=rcl_interfaces::ParameterType::PARAMETER_INTEGER,
  PARAMETER_DOUBLE=rcl_interfaces::ParameterType::PARAMETER_DOUBLE,
  PARAMETER_STRING=rcl_interfaces::ParameterType::PARAMETER_STRING,
  PARAMETER_BYTES=rcl_interfaces::ParameterType::PARAMETER_BYTES,
};

// Structure to store an arbitrary parameter with templated get/set methods
class ParameterVariant
{
public:
  ParameterVariant()
  : name_("")
  {
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_NOT_SET;
  }
  explicit ParameterVariant(const std::string & name, const bool bool_value)
  : name_(name)
  {
    value_.bool_value = bool_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_BOOL;
  }
  explicit ParameterVariant(const std::string & name, const int int_value)
  : name_(name)
  {
    value_.integer_value = int_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_INTEGER;
  }
  explicit ParameterVariant(const std::string & name, const int64_t int_value)
  : name_(name)
  {
    value_.integer_value = int_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_INTEGER;
  }
  explicit ParameterVariant(const std::string & name, const float double_value)
  : name_(name)
  {
    value_.double_value = double_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_DOUBLE;
  }
  explicit ParameterVariant(const std::string & name, const double double_value)
  : name_(name)
  {
    value_.double_value = double_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_DOUBLE;
  }
  explicit ParameterVariant(const std::string & name, const std::string & string_value)
  : name_(name)
  {
    value_.string_value = string_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_STRING;
  }
  explicit ParameterVariant(const std::string & name, const std::vector<int8_t> & bytes_value)
  : name_(name)
  {
    value_.bytes_value = bytes_value;
    value_.parameter_type = rcl_interfaces::ParameterType::PARAMETER_BYTES;
  }

  /* Templated getter */
  template<typename T>
  T
  get_value() const;

  inline ParameterType get_type() const {return static_cast<ParameterType>(value_.parameter_type); }

  inline std::string get_name() const & {return name_; }

  inline rcl_interfaces::ParameterValue get_parameter_value() const
  {
    return value_;
  }

private:
  std::string name_;
  rcl_interfaces::ParameterValue value_;
};

template<>
inline int64_t ParameterVariant::get_value() const
{
  if (value_.parameter_type != rcl_interfaces::ParameterType::PARAMETER_INTEGER) {
    // TODO: use custom exception
    throw std::runtime_error("Invalid type");
  }
  return value_.integer_value;
}
template<>
inline double ParameterVariant::get_value() const
{
  if (value_.parameter_type != rcl_interfaces::ParameterType::PARAMETER_DOUBLE) {
    // TODO: use custom exception
    throw std::runtime_error("Invalid type");
  }
  return value_.double_value;
}
template<>
inline std::string ParameterVariant::get_value() const
{
  if (value_.parameter_type != rcl_interfaces::ParameterType::PARAMETER_STRING) {
    // TODO: use custom exception
    throw std::runtime_error("Invalid type");
  }
  return value_.string_value;
}
template<>
inline bool ParameterVariant::get_value() const
{
  if (value_.parameter_type != rcl_interfaces::ParameterType::PARAMETER_BOOL) {
    // TODO: use custom exception
    throw std::runtime_error("Invalid type");
  }
  return value_.bool_value;
}
template<>
inline std::vector<int8_t> ParameterVariant::get_value() const
{
  if (value_.parameter_type != rcl_interfaces::ParameterType::PARAMETER_BYTES) {
    // TODO: use custom exception
    throw std::runtime_error("Invalid type");
  }
  return value_.bytes_value;
}

class AsyncParametersClient
{

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(AsyncParametersClient);

  AsyncParametersClient(const rclcpp::node::Node::SharedPtr & node)
  : node_(node)
  {
    get_parameters_client_ = node_->create_client<rcl_interfaces::GetParameters>(
      "get_parameters");
    get_parameter_types_client_ = node_->create_client<rcl_interfaces::GetParameterTypes>(
      "get_parameter_types");
    set_parameters_client_ = node_->create_client<rcl_interfaces::SetParameters>(
      "set_parameters");
    list_parameters_client_ = node_->create_client<rcl_interfaces::ListParameters>(
      "list_parameters");
    describe_parameters_client_ = node_->create_client<rcl_interfaces::DescribeParameters>(
      "describe_parameters");
  }

  std::shared_future<std::vector<ParameterVariant>>
  get_parameters(
    std::vector<std::string> names,
    std::function<void(
      std::shared_future<std::vector<ParameterVariant>>)> callback = nullptr)
  {
    std::shared_future<std::vector<ParameterVariant>> f;
    return f;
  }

  std::shared_future<std::vector<ParameterType>>
  get_parameter_types(
    std::vector<std::string> parameter_names,
    std::function<void(
      std::shared_future<std::vector<ParameterType>>)> callback = nullptr)
  {
    std::shared_future<std::vector<ParameterType>> f;
    return f;
  }

  std::shared_future<std::vector<rcl_interfaces::ParameterSetResult>>
  set_parameters(
    std::vector<ParameterVariant> parameters,
    std::function<void(
      std::shared_future<std::vector<rcl_interfaces::ParameterSetResult>>)> callback = nullptr)
  {
    std::shared_future<std::vector<rcl_interfaces::ParameterSetResult>> f;
    return f;
  }

  std::shared_future<rcl_interfaces::ParameterSetResult>
  set_parameters_atomically(
    std::vector<ParameterVariant> parameters,
    std::function<void(
      std::shared_future<rcl_interfaces::ParameterSetResult>)> callback = nullptr)
  {
    std::shared_future<rcl_interfaces::ParameterSetResult> f;
    return f;
  }

  std::shared_future<rcl_interfaces::ParameterListResult>
  list_parameters(
    std::vector<std::string> parameter_prefixes,
    uint64_t depth,
    std::function<void(
      std::shared_future<rcl_interfaces::ParameterListResult>)> callback = nullptr)
  {
    std::shared_future<rcl_interfaces::ParameterListResult> f;
    return f;
  }

private:
  const rclcpp::node::Node::SharedPtr node_;
  rclcpp::client::Client<rcl_interfaces::GetParameters>::SharedPtr get_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::GetParameterTypes>::SharedPtr get_parameter_types_client_;
  rclcpp::client::Client<rcl_interfaces::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_client_;
  rclcpp::client::Client<rcl_interfaces::ListParameters>::SharedPtr list_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::DescribeParameters>::SharedPtr describe_parameters_client_;
};

class SyncParametersClient
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(SyncParametersClient);

  SyncParametersClient(
    rclcpp::node::Node::SharedPtr & node)
  : node_(node)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    async_parameters_client_ = std::make_shared<AsyncParametersClient>(node);
  }

  SyncParametersClient(
    rclcpp::executor::Executor::SharedPtr & executor,
    rclcpp::node::Node::SharedPtr & node)
  : executor_(executor), node_(node)
  {
    async_parameters_client_ = std::make_shared<AsyncParametersClient>(node);
  }

  std::vector<ParameterVariant>
  get_parameters(std::vector<std::string> parameter_names)
  {
    auto f = async_parameters_client_->get_parameters(parameter_names);
    return rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f).get();
  }

  std::vector<ParameterType>
  get_parameter_types(std::vector<std::string> parameter_names)
  {
    auto f = async_parameters_client_->get_parameter_types(parameter_names);
    return rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f).get();
  }

  std::vector<rcl_interfaces::ParameterSetResult>
  set_parameters(std::vector<ParameterVariant> parameters)
  {
    auto f = async_parameters_client_->set_parameters(parameters);
    return rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f).get();
  }

  rcl_interfaces::ParameterSetResult
  set_parameters_atomically(std::vector<ParameterVariant> parameters)
  {
    auto f = async_parameters_client_->set_parameters_atomically(parameters);
    return rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f).get();
  }

  rcl_interfaces::ParameterListResult
  list_parameters(
    std::vector<std::string> parameter_prefixes,
    uint64_t depth)
  {
    auto f = async_parameters_client_->list_parameters(parameter_prefixes, depth);
    return rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f).get();
  }

private:
  rclcpp::executor::Executor::SharedPtr executor_;
  rclcpp::node::Node::SharedPtr node_;
  AsyncParametersClient::SharedPtr async_parameters_client_;
};

} /* namespace parameter */

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
