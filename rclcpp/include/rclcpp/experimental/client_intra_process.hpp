// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__CLIENT_INTRA_PROCESS_HPP_
#define RCLCPP__EXPERIMENTAL__CLIENT_INTRA_PROCESS_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>  // NOLINT, cpplint doesn't think this is a cpp std header

#include "rcutils/logging_macros.h"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/client_intra_process_base.hpp"

namespace rclcpp
{
namespace experimental
{

template<typename ServiceT>
class ClientIntraProcess : public ClientIntraProcessBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ClientIntraProcess)

  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using CallbackType = std::function<void (SharedFuture)>;
  using CallbackWithRequestType = std::function<void (SharedFutureWithRequest)>;

  using CallbackTypeValueVariant = std::tuple<CallbackType, SharedFuture, Promise>;
  using CallbackWithRequestTypeValueVariant = std::tuple<
    CallbackWithRequestType, SharedRequest, SharedFutureWithRequest, PromiseWithRequest>;

  using CallbackInfoVariant = std::variant<
    std::promise<SharedResponse>,
    CallbackTypeValueVariant,
    CallbackWithRequestTypeValueVariant>;

  using ServiceResponse = std::pair<SharedResponse, CallbackInfoVariant>;

  ClientIntraProcess(
    rclcpp::Context::SharedPtr context,
    const std::string & service_name,
    const rclcpp::QoS & qos_profile)
  : ClientIntraProcessBase(context, service_name, qos_profile)
  {
    // Create the intra-process buffer.
    buffer_ = rclcpp::experimental::create_service_intra_process_buffer<
      ServiceResponse>(qos_profile);
  }

  virtual ~ClientIntraProcess() = default;

  bool
  is_ready(rcl_wait_set_t * wait_set)
  {
    (void) wait_set;
    return buffer_->has_data();
  }

  void
  store_intra_process_response(ServiceResponse && response)
  {
    buffer_->add(std::move(response));
    gc_.trigger();
  }

  std::shared_ptr<void>
  take_data() override
  {
    auto data = std::make_shared<ServiceResponse>(std::move(buffer_->consume()));
    return std::static_pointer_cast<void>(data);
  }

  void execute(std::shared_ptr<void> & data)
  {
    if (!data) {
      throw std::runtime_error("'data' is empty");
    }

    auto data_ptr = std::static_pointer_cast<ServiceResponse>(data);
    auto & typed_response = data_ptr->first;
    auto & value = data_ptr->second;

    if (std::holds_alternative<Promise>(value)) {
      auto & promise = std::get<Promise>(value);
      promise.set_value(std::move(typed_response));
    } else if (std::holds_alternative<CallbackTypeValueVariant>(value)) {
      auto & inner = std::get<CallbackTypeValueVariant>(value);
      const auto & callback = std::get<CallbackType>(inner);
      auto & promise = std::get<Promise>(inner);
      auto & future = std::get<SharedFuture>(inner);
      promise.set_value(std::move(typed_response));
      callback(std::move(future));
    } else if (std::holds_alternative<CallbackWithRequestTypeValueVariant>(value)) {
      auto & inner = std::get<CallbackWithRequestTypeValueVariant>(value);
      const auto & callback = std::get<CallbackWithRequestType>(inner);
      auto & promise = std::get<PromiseWithRequest>(inner);
      auto & future = std::get<SharedFutureWithRequest>(inner);
      auto & request = std::get<SharedRequest>(inner);
      promise.set_value(std::make_pair(std::move(request), std::move(typed_response)));
      callback(std::move(future));
    }
  }

protected:
  using BufferUniquePtr =
    typename rclcpp::experimental::buffers::ServiceIntraProcessBuffer<
    ServiceResponse>::UniquePtr;

  BufferUniquePtr buffer_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__CLIENT_INTRA_PROCESS_HPP_
