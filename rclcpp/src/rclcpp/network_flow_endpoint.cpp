// Copyright 2020 Ericsson AB
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

#include <string>

#include "rclcpp/network_flow_endpoint.hpp"

namespace rclcpp
{

const std::string &
NetworkFlowEndpoint::transport_protocol() const
{
  return transport_protocol_;
}

const std::string &
NetworkFlowEndpoint::internet_protocol() const
{
  return internet_protocol_;
}

uint16_t NetworkFlowEndpoint::transport_port() const
{
  return transport_port_;
}

uint32_t NetworkFlowEndpoint::flow_label() const
{
  return flow_label_;
}

uint8_t NetworkFlowEndpoint::dscp() const
{
  return dscp_;
}

const std::string &
NetworkFlowEndpoint::internet_address() const
{
  return internet_address_;
}

bool operator==(const NetworkFlowEndpoint & left, const NetworkFlowEndpoint & right)
{
  return left.transport_protocol_ == right.transport_protocol_ &&
         left.internet_protocol_ == right.internet_protocol_ &&
         left.transport_port_ == right.transport_port_ &&
         left.flow_label_ == right.flow_label_ &&
         left.dscp_ == right.dscp_ &&
         left.internet_address_ == right.internet_address_;
}

bool operator!=(const NetworkFlowEndpoint & left, const NetworkFlowEndpoint & right)
{
  return !(left == right);
}

std::ostream & operator<<(std::ostream & os, const NetworkFlowEndpoint & network_flow_endpoint)
{
  // Stream out in JSON-like format
  os << "{" <<
    "\"transportProtocol\": \"" << network_flow_endpoint.transport_protocol_ << "\", " <<
    "\"internetProtocol\": \"" << network_flow_endpoint.internet_protocol_ << "\", " <<
    "\"transportPort\": \"" << network_flow_endpoint.transport_port_ << "\", " <<
    "\"flowLabel\": \"" << std::to_string(network_flow_endpoint.flow_label_) << "\", " <<
    "\"dscp\": \"" << std::to_string(network_flow_endpoint.dscp_) << "\", " <<
    "\"internetAddress\": \"" << network_flow_endpoint.internet_address_ << "\"" <<
    "}";
  return os;
}

}  // namespace rclcpp
