// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PAYLOAD_HPP_
#define RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PAYLOAD_HPP_

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{

/// Mechanism for passing rmw implementation specific settings through the ROS interfaces.
class RCLCPP_PUBLIC RMWImplementationSpecificPayload
{
public:
  virtual
  ~RMWImplementationSpecificPayload() = default;

  /// Return false if this class has not been customized, otherwise true.
  /**
   * It does this based on the value of the rmw implementation identifier that
   * this class reports, and so it is important for a specialization of this
   * class to override the get_rmw_implementation_identifier() method to return
   * something other than nullptr.
   */
  bool
  has_been_customized() const;

  /// Derrived classes should override this and return the identifier of its rmw implementation.
  virtual
  const char *
  get_implementation_identifier() const;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PAYLOAD_HPP_
