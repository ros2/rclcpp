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

#include "rclcpp/network_flow.hpp"

namespace rclcpp
{

const std::string &
NetworkFlow::transport_protocol() const
{
  return transport_protocol_;
}

const std::string &
NetworkFlow::internet_protocol() const
{
  return internet_protocol_;
}

uint16_t NetworkFlow::transport_port() const
{
  return transport_port_;
}

uint32_t NetworkFlow::flow_label() const
{
  return flow_label_;
}

const std::string &
NetworkFlow::internet_address() const
{
  return internet_address_;
}

bool operator==(const NetworkFlow & left, const NetworkFlow & right)
{
  return left.transport_protocol_ == right.transport_protocol_ &&
         left.internet_protocol_ == right.internet_protocol_ &&
         left.transport_port_ == right.transport_port_ &&
         left.flow_label_ == right.flow_label_ &&
         left.internet_address_ == right.internet_address_;
}

bool operator!=(const NetworkFlow & left, const NetworkFlow & right)
{
  return !(left == right);
}

std::ostream & operator<<(std::ostream & os, const NetworkFlow & network_flow)
{
  os << '{' <<
    "\"transportProtocol\": " << network_flow.transport_protocol_ << ", " <<
    "\"transportPort\": " << network_flow.transport_port_ << ", " <<
    "\"internetProtocol\": " << network_flow.internet_protocol_ << ", " <<
    "\"internetAddress\": " << network_flow.internet_address_ << ", " <<
    "\"flowLabel\": " << network_flow.flow_label_ << ", " <<
    '}';
  return os;
}

}  // namespace rclcpp
