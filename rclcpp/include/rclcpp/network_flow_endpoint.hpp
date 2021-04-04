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

#ifndef RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_
#define RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_

#include <cstdint>
#include <string>
#include <iostream>

#include "rcl/network_flow_endpoints.h"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Forward declaration
class NetworkFlowEndpoint;

/// Check if two NetworkFlowEndpoint instances are equal
RCLCPP_PUBLIC
bool operator==(const NetworkFlowEndpoint & left, const NetworkFlowEndpoint & right);

/// Check if two NetworkFlowEndpoint instances are not equal
RCLCPP_PUBLIC
bool operator!=(const NetworkFlowEndpoint & left, const NetworkFlowEndpoint & right);

/// Streaming helper for NetworkFlowEndpoint
RCLCPP_PUBLIC
std::ostream & operator<<(std::ostream & os, const NetworkFlowEndpoint & network_flow_endpoint);

/**
 * Class describes a network flow endpoint based on the counterpart definition
 * in the RMW layer.
 */
class NetworkFlowEndpoint
{
public:
  /// Construct from rcl_network_flow_endpoint_t
  RCLCPP_PUBLIC
  explicit NetworkFlowEndpoint(const rcl_network_flow_endpoint_t & network_flow_endpoint)
  : transport_protocol_(
      rcl_network_flow_endpoint_get_transport_protocol_string(network_flow_endpoint.
      transport_protocol)),
    internet_protocol_(
      rcl_network_flow_endpoint_get_internet_protocol_string(
        network_flow_endpoint.internet_protocol)),
    transport_port_(network_flow_endpoint.transport_port),
    flow_label_(network_flow_endpoint.flow_label),
    dscp_(network_flow_endpoint.dscp),
    internet_address_(network_flow_endpoint.internet_address)
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

  /// Get DSCP
  RCLCPP_PUBLIC
  uint8_t dscp() const;

  /// Get internet address
  RCLCPP_PUBLIC
  const std::string & internet_address() const;

  /// Compare two NetworkFlowEndpoint instances
  friend bool rclcpp::operator==(
    const NetworkFlowEndpoint & left,
    const NetworkFlowEndpoint & right);
  friend bool rclcpp::operator!=(
    const NetworkFlowEndpoint & left,
    const NetworkFlowEndpoint & right);

  /// Streaming helper
  friend std::ostream & rclcpp::operator<<(
    std::ostream & os,
    const NetworkFlowEndpoint & network_flow_endpoint);

private:
  std::string transport_protocol_;
  std::string internet_protocol_;
  uint16_t transport_port_;
  uint32_t flow_label_;
  uint8_t dscp_;
  std::string internet_address_;
};

}  // namespace rclcpp

#endif  // RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_
