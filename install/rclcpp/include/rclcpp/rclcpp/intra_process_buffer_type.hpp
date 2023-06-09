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

#ifndef RCLCPP__INTRA_PROCESS_BUFFER_TYPE_HPP_
#define RCLCPP__INTRA_PROCESS_BUFFER_TYPE_HPP_

namespace rclcpp
{

/// Used as argument in create_publisher and create_subscriber
/// when intra-process communication is enabled
enum class IntraProcessBufferType
{
  /// Set the data type used in the intra-process buffer as std::shared_ptr<MessageT>
  SharedPtr,
  /// Set the data type used in the intra-process buffer as std::unique_ptr<MessageT>
  UniquePtr,
  /// Set the data type used in the intra-process buffer as the same used in the callback
  CallbackDefault
};

}  // namespace rclcpp

#endif  // RCLCPP__INTRA_PROCESS_BUFFER_TYPE_HPP_
