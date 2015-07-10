// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_PUBLISHER_HPP_
#define RCLCPP_RCLCPP_PUBLISHER_HPP_

#include <memory>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node
{
class Node;
} // namespace node

namespace publisher
{

class Publisher
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(Publisher);

  Publisher(rmw_publisher_t * publisher_handle)
  : publisher_handle_(publisher_handle)
  {}

  template<typename MessageT>
  void
  publish(std::shared_ptr<MessageT> & msg)
  {
    rmw_ret_t status = rmw_publish(publisher_handle_, msg.get());
    if (status != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to publish message: ") +
        (rmw_get_error_string() ? rmw_get_error_string() : ""));
      // *INDENT-ON*
    }
  }

private:
  rmw_publisher_t * publisher_handle_;

};

} /* namespace publisher */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PUBLISHER_HPP_ */
