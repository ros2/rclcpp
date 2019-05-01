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

#ifndef RCLCPP__MAPPED_RING_BUFFER_HPP_
#define RCLCPP__MAPPED_RING_BUFFER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace mapped_ring_buffer
{

class RCLCPP_PUBLIC MappedRingBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MappedRingBufferBase)
};

/// Ring buffer container of shared_ptr's or unique_ptr's of T, which can be accessed by a key.
/**
 * T must be a CopyConstructable and CopyAssignable.
 * This class can be used in a container by using the base class MappedRingBufferBase.
 * This class must have a positive, non-zero size.
 * This class cannot be resized nor can it reserve additional space after construction.
 * This class is not CopyConstructable nor CopyAssignable.
 *
 * The key's are not guaranteed to be unique because push_and_replace does not
 * check for colliding keys.
 * It is up to the user to only use unique keys.
 * A side effect of this is that when get_copy_at_key or pop_at_key are called,
 * they return the first encountered instance of the key.
 * But iteration does not begin with the ring buffer's head, and therefore
 * there is no guarantee on which value is returned if a key is used multiple
 * times.
 */
template<typename T, typename Alloc = std::allocator<void>>
class MappedRingBuffer : public MappedRingBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MappedRingBuffer<T, Alloc>)
  using ElemAllocTraits = allocator::AllocRebind<T, Alloc>;
  using ElemAlloc = typename ElemAllocTraits::allocator_type;
  using ElemDeleter = allocator::Deleter<ElemAlloc, T>;

  using ConstElemSharedPtr = std::shared_ptr<const T>;
  using ElemUniquePtr = std::unique_ptr<T, ElemDeleter>;

  /// Constructor.
  /**
   * The constructor will allocate memory while reserving space.
   *
   * \param size size of the ring buffer; must be positive and non-zero.
   * \param allocator optional custom allocator
   */
  explicit MappedRingBuffer(size_t size, std::shared_ptr<Alloc> allocator = nullptr)
  : elements_(size), head_(0)
  {
    if (size == 0) {
      throw std::invalid_argument("size must be a positive, non-zero value");
    }
    if (!allocator) {
      allocator_ = std::make_shared<ElemAlloc>();
    } else {
      allocator_ = std::make_shared<ElemAlloc>(*allocator.get());
    }
  }

  virtual ~MappedRingBuffer() {}

  /// Return a copy of the value stored in the ring buffer at the given key.
  /**
   * The key is matched if an element in the ring buffer has a matching key.
   * This method will allocate in order to return a copy.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  get(uint64_t key, ElemUniquePtr & value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = get_iterator_of_key(key);
    value = nullptr;
    if (it != elements_.end() && it->in_use) {
      if (it->unique_value) {
        ElemDeleter deleter = it->unique_value.get_deleter();
        auto ptr = ElemAllocTraits::allocate(*allocator_.get(), 1);
        ElemAllocTraits::construct(*allocator_.get(), ptr, *it->unique_value);
        value = ElemUniquePtr(ptr, deleter);
      } else if (it->shared_value) {
        ElemDeleter * deleter = std::get_deleter<ElemDeleter, const T>(it->shared_value);
        auto ptr = ElemAllocTraits::allocate(*allocator_.get(), 1);
        ElemAllocTraits::construct(*allocator_.get(), ptr, *it->shared_value);
        if (deleter) {
          value = ElemUniquePtr(ptr, *deleter);
        } else {
          value = ElemUniquePtr(ptr);
        }
      } else {
        throw std::runtime_error("Unexpected empty MappedRingBuffer element.");
      }
    }
  }

  /// Share ownership of the value stored in the ring buffer at the given key.
  /**
   * The key is matched if an element in the ring buffer has a matching key.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  get(uint64_t key, ConstElemSharedPtr & value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = get_iterator_of_key(key);
    value.reset();
    if (it != elements_.end() && it->in_use) {
      if (!it->shared_value) {
        // The stored unique_ptr is upgraded to a shared_ptr here.
        // All the remaining get and pop calls done with unique_ptr
        // signature will receive a copy.
        if (!it->unique_value) {
          throw std::runtime_error("Unexpected empty MappedRingBuffer element.");
        }
        it->shared_value = std::move(it->unique_value);
      }
      value = it->shared_value;
    }
  }

  /// Give the ownership of the stored value to the caller if possible, or copy and release.
  /**
   * The key is matched if an element in the ring buffer has a matching key.
   * This method may allocate in order to return a copy.
   *
   * If the stored value is a shared_ptr, it is not possible to downgrade it to a unique_ptr.
   * In that case, a copy is returned and the stored value is released.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  pop(uint64_t key, ElemUniquePtr & value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = get_iterator_of_key(key);
    value = nullptr;
    if (it != elements_.end() && it->in_use) {
      if (it->unique_value) {
        value = std::move(it->unique_value);
      } else if (it->shared_value) {
        auto ptr = ElemAllocTraits::allocate(*allocator_.get(), 1);
        ElemAllocTraits::construct(*allocator_.get(), ptr, *it->shared_value);
        auto deleter = std::get_deleter<ElemDeleter, const T>(it->shared_value);
        if (deleter) {
          value = ElemUniquePtr(ptr, *deleter);
        } else {
          value = ElemUniquePtr(ptr);
        }
        it->shared_value.reset();
      } else {
        throw std::runtime_error("Unexpected empty MappedRingBuffer element.");
      }
      it->in_use = false;
    }
  }

  /// Give the ownership of the stored value to the caller, at the given key.
  /**
   * The key is matched if an element in the ring buffer has a matching key.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  pop(uint64_t key, ConstElemSharedPtr & value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = get_iterator_of_key(key);
    if (it != elements_.end() && it->in_use) {
      if (it->shared_value) {
        value = std::move(it->shared_value);
      } else if (it->unique_value) {
        value = std::move(it->unique_value);
      } else {
        throw std::runtime_error("Unexpected empty MappedRingBuffer element.");
      }
      it->in_use = false;
    }
  }

  /// Insert a key-value pair, displacing an existing pair if necessary.
  /**
   * The key's uniqueness is not checked on insertion.
   * It is up to the user to ensure the key is unique.
   * This method should not allocate memory.
   *
   * After insertion the value will be a nullptr.
   * If a pair were replaced, its smart pointer is reset.
   *
   * \param key the key associated with the value to be stored
   * \param value the value to store, and optionally the value displaced
   */
  bool
  push_and_replace(uint64_t key, ConstElemSharedPtr value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool did_replace = elements_[head_].in_use;
    Element & element = elements_[head_];
    element.key = key;
    element.unique_value.reset();
    element.shared_value.reset();
    element.shared_value = value;
    element.in_use = true;
    head_ = (head_ + 1) % elements_.size();
    return did_replace;
  }

  /// Insert a key-value pair, displacing an existing pair if necessary.
  /**
   * See `bool push_and_replace(uint64_t key, const ConstElemSharedPtr & value)`.
   */
  bool
  push_and_replace(uint64_t key, ElemUniquePtr value)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool did_replace = elements_[head_].in_use;
    Element & element = elements_[head_];
    element.key = key;
    element.unique_value.reset();
    element.shared_value.reset();
    element.unique_value = std::move(value);
    element.in_use = true;
    head_ = (head_ + 1) % elements_.size();
    return did_replace;
  }

  /// Return true if the key is found in the ring buffer, otherwise false.
  bool
  has_key(uint64_t key)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return elements_.end() != get_iterator_of_key(key);
  }

private:
  RCLCPP_DISABLE_COPY(MappedRingBuffer<T, Alloc>)

  struct Element
  {
    uint64_t key;
    ElemUniquePtr unique_value;
    ConstElemSharedPtr shared_value;
    bool in_use;
  };

  using VectorAlloc = typename std::allocator_traits<Alloc>::template rebind_alloc<Element>;

  typename std::vector<Element, VectorAlloc>::iterator
  get_iterator_of_key(uint64_t key)
  {
    auto it = std::find_if(
      elements_.begin(), elements_.end(),
      [key](Element & e) -> bool {
        return e.key == key && e.in_use;
      });
    return it;
  }

  std::vector<Element, VectorAlloc> elements_;
  size_t head_;
  std::shared_ptr<ElemAlloc> allocator_;
  std::mutex data_mutex_;
};

}  // namespace mapped_ring_buffer
}  // namespace rclcpp

#endif  // RCLCPP__MAPPED_RING_BUFFER_HPP_
