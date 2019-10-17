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

#ifndef RCLCPP__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_
#define RCLCPP__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_

namespace rclcpp
{
namespace intra_process_buffer
{

template<typename BufferT>
class BufferImplementationBase
{
public:
  virtual ~BufferImplementationBase() {}

  virtual BufferT dequeue() = 0;
  virtual void enqueue(BufferT request) = 0;

  virtual void clear() = 0;
  virtual bool has_data() const = 0;
};

}  // namespace intra_process_buffer
}  // namespace rclcpp

#endif  // RCLCPP__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_
