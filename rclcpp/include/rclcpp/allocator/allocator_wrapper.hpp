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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_WRAPPER_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_WRAPPER_HPP_

#include <memory>
#include <iostream>
#include <rclcpp/allocator/allocator_deleter.hpp>

namespace rclcpp
{

namespace allocator
{

// Type-erased interface to AllocatorWrapper
class AllocatorWrapper
{
  virtual void * allocate(size_t size) = 0;
  virtual void deallocate(void * pointer, size_t size) = 0;
  // Construct will have to be through placement new, since pure virtual function can't be templated

  virtual void destroy(T* pointer) = 0;
}

template<typename T, typename Alloc>
class TypedAllocatorWrapper : public AllocatorWrapper
{
public:
  /*
  using Deleter = typename std::conditional<
      std::is_same<Alloc, std::allocator<T>>::value,
      std::default_delete<T>,
      AllocatorDeleter<T, Alloc>
      >::type;
  */

  using DeleterT = Deleter<Alloc, T>;

  TypedAllocatorWrapper(Alloc * allocator)
  : allocator_(allocator)
  {
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    initialize_deleter(deleter_, allocator_);
    if (deleter_ == NULL) {
      throw std::invalid_argument("Failed to initialize deleter");
    }
  }

  TypedAllocatorWrapper(Alloc * allocator, DeleterT * deleter)
  : allocator_(allocator), deleter_(deleter)
  {
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    if (!deleter_) {
      throw std::invalid_argument("Deleter argument was NULL");
    }
  }

  TypedAllocatorWrapper(Alloc & allocator)
  {
    allocator_ = &allocator;
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    initialize_deleter(deleter_, allocator_);
    if (!deleter_) {
      throw std::invalid_argument("Failed to initialize deleter");
    }
  }

  TypedAllocatorWrapper()
  {
    allocator_ = new Alloc();
    initialize_deleter(deleter_, allocator_);
    if (!deleter_) {
      //throw std::invalid_argument("Failed to initialize deleter");
      deleter_ = new DeleterT;
    }
  }

  void * allocate(size_t size)
  {
    return std::allocator_traits<Alloc>::allocate(*allocator_, size);
  }

  void deallocate(void * pointer, size_t size)
  {
    deallocate(static_cast<T*>(pointer), size);
  }

  void deallocate(T * pointer, size_t size)
  {
    std::allocator_traits<Alloc>::deallocate(*allocator_, pointer, size);
  }

  template<class ... Args>
  void construct(T * pointer, Args && ... args)
  {
    std::allocator_traits<Alloc>::construct(*allocator_, pointer, std::forward<Args>(args)...);
  }

  void destroy(void * pointer)
  {
    destroy(static_cast<T*>(pointer));
  }

  template<class T>
  void destroy(T * pointer)
  {
    std::allocator_traits<Alloc>::destroy(*allocator_, pointer);
  }

  DeleterT * get_deleter() const
  {
    return deleter_;
  }
  Alloc * get_underlying_allocator() const
  {
    return allocator_;
  }

private:
  Alloc * allocator_;
  DeleterT * deleter_;
};

template<typename T>
using DefaultAllocator = TypedAllocatorWrapper<T, std::allocator<T>>;

}
}

#endif
