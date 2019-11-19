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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

template<typename BufferT>
class RingBufferImplementation : public BufferImplementationBase<BufferT>
{
public:
  explicit RingBufferImplementation(size_t capacity)
  : capacity_(capacity),
    ring_buffer_(capacity),
    write_index_(capacity_ - 1),
    read_index_(0),
    size_(0)
  {
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }

  virtual ~RingBufferImplementation() {}

  void enqueue(BufferT request)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    write_index_ = next(write_index_);
    ring_buffer_[write_index_] = std::move(request);

    if (is_full()) {
      read_index_ = next(read_index_);
    } else {
      size_++;
    }
  }

  BufferT dequeue()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_data()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Calling dequeue on empty intra-process buffer");
      throw std::runtime_error("Calling dequeue on empty intra-process buffer");
    }

    auto request = std::move(ring_buffer_[read_index_]);
    read_index_ = next(read_index_);

    size_--;

    return request;
  }

  inline size_t next(size_t val)
  {
    return (val + 1) % capacity_;
  }

  inline bool has_data() const
  {
    return size_ != 0;
  }

  inline bool is_full()
  {
    return size_ == capacity_;
  }

  void clear() {}

private:
  size_t capacity_;

  std::vector<BufferT> ring_buffer_;

  size_t write_index_;
  size_t read_index_;
  size_t size_;

  std::mutex mutex_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
