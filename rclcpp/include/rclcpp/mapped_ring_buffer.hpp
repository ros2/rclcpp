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

#ifndef RCLCPP_RCLCPP_RING_BUFFER_HPP_
#define RCLCPP_RCLCPP_RING_BUFFER_HPP_

#include <rclcpp/macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace rclcpp
{
namespace mapped_ring_buffer
{

class MappedRingBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MappedRingBufferBase);
};

/// Ring buffer container of unique_ptr's of T, which can be accessed by a key.
/* T must be a CopyConstructable and CopyAssignable.
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
template<typename T, typename Allocator = std::allocator<T>>
class MappedRingBuffer : public MappedRingBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MappedRingBuffer<T, Allocator>);

  /// Constructor.
  /* The constructor will allocate memory while reserving space.
   *
   * \param size size of the ring buffer; must be positive and non-zero.
   */
  MappedRingBuffer(size_t size, Allocator * message_allocator)
  : elements_(size), head_(0), allocator_(message_allocator)
  {
    if (size == 0) {
      throw std::invalid_argument("size must be a positive, non-zero value");
    }
    if (!allocator_) {
      throw std::invalid_argument("NULL allocator received in MappedRingBuffer constructor!");
    }
  }
  virtual ~MappedRingBuffer() {}

  /// Return a copy of the value stored in the ring buffer at the given key.
  /* The key is matched if an element in the ring buffer has a matching key.
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
  get_copy_at_key(uint64_t key, std::unique_ptr<T> & value)
  {
    auto it = get_iterator_of_key(key);
    value = nullptr;
    if (it != elements_.end() && it->in_use) {
      auto ptr = allocator_->allocate(1);
      std::allocator_traits<Allocator>::construct(*allocator_, ptr, *it->value);
      value = std::unique_ptr<T>(ptr);
    }
  }

  /// Return ownership of the value stored in the ring buffer, leaving a copy.
  /* The key is matched if an element in the ring bufer has a matching key.
   * This method will allocate in order to store a copy.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The ownership of the currently stored object is returned, but a copy is
   * made and stored in its place.
   * This means that multiple calls to this function for a particular element
   * will result in returning the copied and stored object not the original.
   * This also means that later calls to pop_at_key will not return the
   * originally stored object, since it was returned by the first call to this
   * method.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  get_ownership_at_key(uint64_t key, std::unique_ptr<T> & value)
  {
    auto it = get_iterator_of_key(key);
    value = nullptr;
    if (it != elements_.end() && it->in_use) {
      // Make a copy.
      auto ptr = allocator_->allocate(1);
      std::allocator_traits<Allocator>::construct(*allocator_, ptr, *it->value);
      auto copy = std::unique_ptr<T>(ptr);
      // Return the original.
      value.swap(it->value);
      // Store the copy.
      it->value.swap(copy);
    }
  }

  /// Return ownership of the value stored in the ring buffer at the given key.
  /* The key is matched if an element in the ring buffer has a matching key.
   *
   * The key is not guaranteed to be unique, see the class docs for more.
   *
   * The contents of value before the method is called are discarded.
   *
   * \param key the key associated with the stored value
   * \param value if the key is found, the value is stored in this parameter
   */
  void
  pop_at_key(uint64_t key, std::unique_ptr<T> & value)
  {
    auto it = get_iterator_of_key(key);
    value = nullptr;
    if (it != elements_.end() && it->in_use) {
      value.swap(it->value);
      it->in_use = false;
    }
  }

  /// Insert a key-value pair, displacing an existing pair if necessary.
  /* The key's uniqueness is not checked on insertion.
   * It is up to the user to ensure the key is unique.
   * This method should not allocate memory.
   *
   * After insertion, if a pair was replaced, then value will contain ownership
   * of that displaced value. Otherwise it will be a nullptr.
   *
   * \param key the key associated with the value to be stored
   * \param value the value to store, and optionally the value displaced
   */
  bool
  push_and_replace(uint64_t key, std::unique_ptr<T> & value)
  {
    bool did_replace = elements_[head_].in_use;
    elements_[head_].key = key;
    elements_[head_].value.swap(value);
    elements_[head_].in_use = true;
    head_ = (head_ + 1) % elements_.size();
    return did_replace;
  }

  bool
  push_and_replace(uint64_t key, std::unique_ptr<T> && value)
  {
    std::unique_ptr<T> temp = std::move(value);
    return push_and_replace(key, temp);
  }

  /// Return true if the key is found in the ring buffer, otherwise false.
  bool
  has_key(uint64_t key)
  {
    return elements_.end() != get_iterator_of_key(key);
  }

private:
  RCLCPP_DISABLE_COPY(MappedRingBuffer<T, Allocator>);

  struct element
  {
    uint64_t key;
    std::unique_ptr<T> value;
    bool in_use;
  };

  typedef std::vector<element, Allocator> ElementVector;

  typename ElementVector::iterator
  get_iterator_of_key(uint64_t key)
  {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    auto it = std::find_if(elements_.begin(), elements_.end(), [key](element & e) -> bool {
      return e.key == key && e.in_use;
    });
    // *INDENT-ON*
    return it;
  }

  ElementVector elements_;
  size_t head_;

  Allocator * allocator_;

};

} /* namespace ring_buffer */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_RING_BUFFER_HPP_ */
