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

#ifndef RCLCPP__NETWORK_FLOW_HPP_
#define RCLCPP__NETWORK_FLOW_HPP_

#include <cstdint>
#include <string>
#include <iostream>

#include "rcl/network_flow.h"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/**
 * Class describes endpoints of network flow such as transport protocol,
 * internet protocol, transport port, flow label, and internet address
 */
class NetworkFlow
{
public:
  /// Construct from rcl_network_flow_t
  RCLCPP_PUBLIC
  explicit NetworkFlow(const rcl_network_flow_t & network_flow)
  : transport_protocol_(
      rcl_network_flow_get_transport_protocol_string(network_flow.transport_protocol)),
    internet_protocol_(
        rcl_network_flow_get_internet_protocol_string(network_flow.internet_protocol)),
    transport_port_(network_flow.transport_port),
    flow_label_(network_flow.flow_label),
    internet_address_(network_flow.internet_address)
  {
  }

  /// Get transport protocol
  RCLCPP_PUBLIC
  const std::string & transport_protocol() const;

  /// Get internet protocol
  RCLCPP_PUBLIC
  const std::string & internet_protocol() const;

  /// Get transport port
  RCLCPP_PUBLIC
  uint16_t transport_port() const;

  /// Get flow label
  RCLCPP_PUBLIC
  uint32_t flow_label() const;

  /// Get internet address
  RCLCPP_PUBLIC
  const std::string & internet_address() const;

  /// Compare two NetworkFlow instances
  friend bool operator==(const NetworkFlow & left, const NetworkFlow & right);
  friend bool operator!=(const NetworkFlow & left, const NetworkFlow & right);

  /// Streaming helper
  friend std::ostream & operator<<(std::ostream & os, const NetworkFlow & network_flow);

private:
  std::string transport_protocol_;
  std::string internet_protocol_;
  uint16_t transport_port_;
  uint32_t flow_label_;
  std::string internet_address_;
};

/// Check if two NetworkFlow instances are equal
RCLCPP_PUBLIC
bool operator==(const NetworkFlow & left, const NetworkFlow & right);

/// Check if two NetworkFlow instances are not equal
RCLCPP_PUBLIC
bool operator!=(const NetworkFlow & left, const NetworkFlow & right);

/// Streaming helper
RCLCPP_PUBLIC
std::ostream & operator<<(std::ostream & os, const NetworkFlow & network_flow);

}  // namespace rclcpp

#endif  // RCLCPP__NETWORK_FLOW_HPP_
