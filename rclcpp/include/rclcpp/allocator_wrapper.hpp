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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_WRAPPER_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_WRAPPER_HPP_

#include <memory>
#include <iostream>
#include <rclcpp/allocator_deleter.hpp>


template<typename Alloc, typename T, typename D>
void initialize_deleter(D * deleter, Alloc * alloc)
{
  (void) deleter;
  (void) alloc;
  throw std::runtime_error("Reached unexpected template specialization");
}

template<typename T>
void initialize_deleter(std::default_delete<T> * deleter, std::allocator<T> * alloc)
{
  (void) alloc;
  std::cout << "calling default specialization of initialize_deleter" << std::endl;
  deleter = new std::default_delete<T>;
  if (!deleter) {
    throw std::runtime_error("initialize_deleter failed");
  }
}

template<typename Alloc, typename T>
void initialize_deleter(AllocatorDeleter<T, Alloc> * deleter, Alloc * alloc)
{
  if (!alloc) {
    throw std::invalid_argument("Allocator argument was NULL");
  }
  deleter = new AllocatorDeleter<T, Alloc>(alloc);
  if (!deleter) {
    throw std::runtime_error("initialize_deleter failed");
  }
}

template<typename T, typename Alloc>
class AllocatorWrapper
{
public:
  using Deleter = typename std::conditional<
      std::is_same<Alloc, std::allocator<T>>::value,
      std::default_delete<T>,
      AllocatorDeleter<T, Alloc>
      >::type;

  AllocatorWrapper(Alloc * allocator)
  : allocator_(allocator)
  {
    std::cout << "constructor 1" << std::endl;
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    initialize_deleter(deleter_, allocator_);
    if (deleter_ == NULL) {
      throw std::invalid_argument("Failed to initialize deleter");
    }
  }

  AllocatorWrapper(Alloc * allocator, Deleter * deleter)
  : allocator_(allocator), deleter_(deleter)
  {
    std::cout << "constructor 2" << std::endl;
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    if (!deleter_) {
      throw std::invalid_argument("Deleter argument was NULL");
    }
  }

  AllocatorWrapper(Alloc & allocator)
  {
    std::cout << "constructor 3" << std::endl;
    allocator_ = &allocator;
    if (!allocator_) {
      throw std::invalid_argument("Allocator argument was NULL");
    }
    initialize_deleter(deleter_, allocator_);
    if (!deleter_) {
      throw std::invalid_argument("Failed to initialize deleter");
    }
  }

  AllocatorWrapper()
  {
    std::cout << "constructor 4" << std::endl;
    allocator_ = new Alloc();
    initialize_deleter(deleter_, allocator_);
    if (!deleter_) {
      //throw std::invalid_argument("Failed to initialize deleter");
      deleter_ = new Deleter;
    }
  }

  T * allocate(size_t size)
  {
    return std::allocator_traits<Alloc>::allocate(*allocator_, size);
  }

  T * deallocate(void * pointer, size_t size)
  {
    std::allocator_traits<Alloc>::deallocate(*allocator_, pointer, size);
  }

  template<class ... Args>
  void construct(T * pointer, Args && ... args)
  {
    std::allocator_traits<Alloc>::construct(*allocator_, pointer, std::forward<Args>(args) ...);
  }

  Deleter * get_deleter() const
  {
    return deleter_;
  }
  Alloc * get_underlying_allocator() const
  {
    return allocator_;
  }

private:
  Alloc * allocator_;
  Deleter * deleter_;
};

template<typename T>
using DefaultAllocator = AllocatorWrapper<T, std::allocator<T>>;

#endif
