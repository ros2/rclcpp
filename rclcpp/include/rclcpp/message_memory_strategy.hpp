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

#ifndef RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_

#include <memory>

#include <rclcpp/macros.hpp>

namespace rclcpp
{
namespace message_memory_strategy
{

// TODO
template<typename MessageT>
class MessageMemoryStrategy
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS(MessageMemoryStrategy);

  static SharedPtr create_default()
  {
    return SharedPtr(new MessageMemoryStrategy<MessageT>);
  }

  virtual std::shared_ptr<MessageT> borrow_message()
  {
    return std::shared_ptr<MessageT>(new MessageT);
  }

  virtual void return_message(std::shared_ptr<MessageT> & msg)
  {
    msg.reset();
  }
};

}  /* message_memory_strategy */
}  /* rclcpp */

#endif  /* RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_ */
