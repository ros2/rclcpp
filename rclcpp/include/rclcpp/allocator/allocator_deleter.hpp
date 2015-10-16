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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_DELETER_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_DELETER_HPP_

#include <memory>

namespace rclcpp
{

namespace allocator
{

template<typename T, typename Allocator>
class AllocatorDeleter
{
public:
  AllocatorDeleter()
  : allocator_(NULL)
  {
  }

  AllocatorDeleter(Allocator * a)
  : allocator_(a)
  {
  }

  template<typename U, typename B>
  AllocatorDeleter(const AllocatorDeleter<U, B> & a)
  {
    allocator_ = a.get_allocator();
  }

  void operator()(T * ptr)
  {
    std::allocator_traits<Allocator>::destroy(*allocator_, ptr);
    std::allocator_traits<Allocator>::deallocate(*allocator_, ptr, sizeof(T));
    ptr = NULL;
  }

  Allocator * get_allocator() const
  {
    return allocator_;
  }

  void set_allocator(Allocator * alloc)
  {
    allocator_ = alloc;
  }

private:
  Allocator * allocator_;
};

/*
template<template<typename> class Alloc, typename T, typename D>
D initialize_deleter(Alloc<T> * alloc)
{
  (void) alloc;
  throw std::runtime_error("Reached unexpected template specialization");
}

template<typename T>
std::default_delete<T> initialize_deleter(std::allocator<T> * alloc)
{
  (void) alloc;
  return std::default_delete<T>();
}

template<template<typename> class Alloc, typename T>
AllocatorDeleter<T, Alloc<T>> initialize_deleter(Alloc<T> * alloc)
{
  if (!alloc) {
    throw std::invalid_argument("Allocator argument was NULL");
  }
  return AllocatorDeleter<T, Alloc<T>>(alloc);
}
*/
template<template<typename> class Alloc, typename T, typename D>
void set_allocator_for_deleter(D * deleter, Alloc<T> * alloc)
{
  (void) alloc;
  throw std::runtime_error("Reached unexpected template specialization");
}

template<typename T>
void set_allocator_for_deleter(std::default_delete<T> * deleter, std::allocator<T> * alloc)
{
  (void) deleter;
  (void) alloc;
}

template<template<typename> class Alloc, typename T>
void set_allocator_for_deleter(AllocatorDeleter<T, Alloc<T>> * deleter, Alloc<T> * alloc)
{
  if (!deleter || !alloc) {
    throw std::invalid_argument("Argument was NULL to set_allocator_for_deleter");
  }
  //return AllocatorDeleter<T, Alloc<T>>(alloc);
  deleter->set_allocator(alloc);
}

template<typename Alloc, typename T>
using Deleter = typename std::conditional<
    std::is_same<Alloc, std::allocator<T>>::value,
    std::default_delete<T>,
    AllocatorDeleter<T, Alloc>
    >::type;

}
}

#endif
