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
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/// Store elements in a fixed-size, FIFO buffer
/**
 * All public member functions are thread-safe.
 */
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
    TRACETOOLS_TRACEPOINT(
      rclcpp_construct_ring_buffer,
      static_cast<const void *>(this),
      capacity_);
  }

  virtual ~RingBufferImplementation() {}

  /// Add a new element to store in the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \param request the element to be stored in the ring buffer
   */
  void enqueue(BufferT request) override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    write_index_ = next_(write_index_);
    ring_buffer_[write_index_] = std::move(request);
    TRACETOOLS_TRACEPOINT(
      rclcpp_ring_buffer_enqueue,
      static_cast<const void *>(this),
      write_index_,
      size_ + 1,
      is_full_());

    if (is_full_()) {
      read_index_ = next_(read_index_);
    } else {
      size_++;
    }
  }

  /// Remove the oldest element from ring buffer
  /**
   * This member function is thread-safe.
   *
   * \return the element that is being removed from the ring buffer
   */
  BufferT dequeue() override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_data_()) {
      return BufferT();
    }

    auto request = std::move(ring_buffer_[read_index_]);
    TRACETOOLS_TRACEPOINT(
      rclcpp_ring_buffer_dequeue,
      static_cast<const void *>(this),
      read_index_,
      size_ - 1);
    read_index_ = next_(read_index_);

    size_--;

    return request;
  }

  /// Get all the elements from the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \return a vector containing all the elements from the ring buffer
   */
  std::vector<BufferT> get_all_data() override
  {
    return get_all_data_impl();
  }

  /// Get the next index value for the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \param val the current index value
   * \return the next index value
   */
  inline size_t next(size_t val)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return next_(val);
  }

  /// Get if the ring buffer has at least one element stored
  /**
   * This member function is thread-safe.
   *
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_();
  }

  /// Get if the size of the buffer is equal to its capacity
  /**
   * This member function is thread-safe.
   *
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_full_();
  }

  /// Get the remaining capacity to store messages
  /**
   * This member function is thread-safe.
   *
   * \return the number of free capacity for new messages
   */
  size_t available_capacity() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return available_capacity_();
  }

  void clear() override
  {
    TRACETOOLS_TRACEPOINT(rclcpp_ring_buffer_clear, static_cast<const void *>(this));
  }

private:
  /// Get the next index value for the ring buffer
  /**
   * This member function is not thread-safe.
   *
   * \param val the current index value
   * \return the next index value
   */
  inline size_t next_(size_t val)
  {
    return (val + 1) % capacity_;
  }

  /// Get if the ring buffer has at least one element stored
  /**
   * This member function is not thread-safe.
   *
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data_() const
  {
    return size_ != 0;
  }

  /// Get if the size of the buffer is equal to its capacity
  /**
   * This member function is not thread-safe.
   *
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full_() const
  {
    return size_ == capacity_;
  }

  /// Get the remaining capacity to store messages
  /**
   * This member function is not thread-safe.
   *
   * \return the number of free capacity for new messages
   */
  inline size_t available_capacity_() const
  {
    return capacity_ - size_;
  }

  /// Traits for checking if a type is std::unique_ptr
  template<typename ...>
  struct is_std_unique_ptr final : std::false_type {};
  template<class T, typename ... Args>
  struct is_std_unique_ptr<std::unique_ptr<T, Args...>> final : std::true_type
  {
    typedef T Ptr_type;
  };

  /// Get all the elements from the ring buffer
  /**
   * This member function is thread-safe.
   * Two versions for the implementation of the function.
   * One for buffer containing unique_ptr and the other for other types
   *
   * \return a vector containing all the elements from the ring buffer
   */
  template<typename T = BufferT, std::enable_if_t<is_std_unique_ptr<T>::value &&
    std::is_copy_constructible<
      typename is_std_unique_ptr<T>::Ptr_type
    >::value,
    void> * = nullptr>
  std::vector<BufferT> get_all_data_impl()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<BufferT> result_vtr;
    result_vtr.reserve(size_);
    for (size_t id = 0; id < size_; ++id) {
      const auto & elem(ring_buffer_[(read_index_ + id) % capacity_]);
      if (elem != nullptr) {
        result_vtr.emplace_back(new typename is_std_unique_ptr<T>::Ptr_type(
          *elem));
      } else {
        result_vtr.emplace_back(nullptr);
      }
    }
    return result_vtr;
  }

  template<typename T = BufferT, std::enable_if_t<
      std::is_copy_constructible<T>::value, void> * = nullptr>
  std::vector<BufferT> get_all_data_impl()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<BufferT> result_vtr;
    result_vtr.reserve(size_);
    for (size_t id = 0; id < size_; ++id) {
      result_vtr.emplace_back(ring_buffer_[(read_index_ + id) % capacity_]);
    }
    return result_vtr;
  }

  template<typename T = BufferT, std::enable_if_t<!is_std_unique_ptr<T>::value &&
    !std::is_copy_constructible<T>::value, void> * = nullptr>
  std::vector<BufferT> get_all_data_impl()
  {
    throw std::logic_error("Underlined type results in invalid get_all_data_impl()");
    return {};
  }

  template<typename T = BufferT, std::enable_if_t<is_std_unique_ptr<T>::value &&
    !std::is_copy_constructible<typename is_std_unique_ptr<T>::Ptr_type>::value,
    void> * = nullptr>
  std::vector<BufferT> get_all_data_impl()
  {
    throw std::logic_error("Underlined type in unique_ptr results in invalid get_all_data_impl()");
    return {};
  }

  size_t capacity_;

  std::vector<BufferT> ring_buffer_;

  size_t write_index_;
  size_t read_index_;
  size_t size_;

  mutable std::mutex mutex_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
