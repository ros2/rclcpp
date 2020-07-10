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
#include <string>
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
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
    ring_callback_buffer_(capacity),
    ring_is_untaken_buffer_(capacity, true),
    write_index_(capacity_ - 1),
    read_callback_index_(0),
    read_is_untaken_index_(0),
    size_(0)
  {
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }

  virtual ~RingBufferImplementation() {}

  void enqueue(BufferT request)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    log_idxs("enqueue()-str");

    write_index_ = next(write_index_);
    ring_callback_buffer_[write_index_] = std::move(request);
    ring_is_untaken_buffer_[write_index_] = true;

    if (size_ == capacity_) {
      if (read_callback_index_ == read_is_untaken_index_) {
        read_is_untaken_index_ = write_index_;
      }
      read_callback_index_ = write_index_;
    } else {
      size_++;
    }
    log_idxs("enqueue()-end");
  }

  BufferT dequeue()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    log_idxs("dequeue()-str");

    if (!has_data()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Calling dequeue on empty intra-process buffer");
      throw std::runtime_error("Calling dequeue on empty intra-process buffer");
    }
    if (ring_is_untaken_buffer_[read_callback_index_]) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Calling dequeue on intra-process buffer that has not been taken");
      throw std::runtime_error("Calling dequeue on intra-process buffer that has not been taken");
    }

    auto request = std::move(ring_callback_buffer_[read_callback_index_]);
    ring_is_untaken_buffer_[read_callback_index_] = true;
    read_callback_index_ = next(read_callback_index_);

    size_--;

    log_idxs("dequeue()-end");

    return request;
  }

  void take_data()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    log_idxs("take_data()-str");

    if (!has_data()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Calling take_data on empty intra-process buffer");
      throw std::runtime_error("Calling take_data on empty intra-process buffer");
    }

    ring_is_untaken_buffer_[read_is_untaken_index_] = false;
    read_is_untaken_index_ = next(read_is_untaken_index_);

    log_idxs("take_data()-end");
  }
  
  inline void log_idxs(std::string fn_name) const
  {
    std::ostringstream oss;
    oss 
     // << std::this_thread::get_id() << "-"
      << fn_name << ": untaken_idx=" << read_is_untaken_index_
      << "; cb_idx=" << read_callback_index_
      << "; size=" << size_;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), oss.str());
  }

  inline size_t next(size_t val) const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return (val + 1) % capacity_;
  }

  inline bool is_ready() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return has_data() && is_not_taken();
  }

  inline bool is_not_taken() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return ring_is_untaken_buffer_[read_is_untaken_index_];
  }

  inline bool has_data() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return size_ != 0;
  }

  inline bool is_full() const { 
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return size_ == capacity_; 
  } 
  
  void clear() {} 

private: 
  size_t capacity_; 

  std::vector<BufferT> ring_callback_buffer_;
  std::vector<bool> ring_is_untaken_buffer_;

  size_t write_index_;
  size_t read_callback_index_;
  size_t read_is_untaken_index_;
  size_t size_;

  mutable std::recursive_mutex mutex_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
