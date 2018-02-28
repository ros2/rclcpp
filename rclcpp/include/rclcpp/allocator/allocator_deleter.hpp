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

#ifndef RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_
#define RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_

#include <memory>
#include <stdexcept>

namespace rclcpp
{
namespace allocator
{

template<typename Allocator>
class AllocatorDeleter
{
  template<typename T>
  using AllocRebind = typename std::allocator_traits<Allocator>::template rebind_alloc<T>;

public:
  AllocatorDeleter()
  : allocator_(nullptr)
  {
  }

  explicit AllocatorDeleter(Allocator * a)
  : allocator_(a)
  {
  }

  template<typename T>
  AllocatorDeleter(const AllocatorDeleter<T> & a)
  {
    allocator_ = a.get_allocator();
  }

  template<typename T>
  void operator()(T * ptr)
  {
    std::allocator_traits<AllocRebind<T>>::destroy(*allocator_, ptr);
    std::allocator_traits<AllocRebind<T>>::deallocate(*allocator_, ptr, 1);
    ptr = nullptr;
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

template<typename Alloc, typename T, typename D>
void set_allocator_for_deleter(D * deleter, Alloc * alloc)
{
  (void) alloc;
  (void) deleter;
  throw std::runtime_error("Reached unexpected template specialization");
}

template<typename T, typename U>
void set_allocator_for_deleter(std::default_delete<T> * deleter, std::allocator<U> * alloc)
{
  (void) deleter;
  (void) alloc;
}

template<typename Alloc, typename T>
void set_allocator_for_deleter(AllocatorDeleter<T> * deleter, Alloc * alloc)
{
  if (!deleter || !alloc) {
    throw std::invalid_argument("Argument was NULL to set_allocator_for_deleter");
  }
  deleter->set_allocator(alloc);
}

template<typename Alloc, typename T>
using Deleter = typename std::conditional<
  std::is_same<typename std::allocator_traits<Alloc>::template rebind_alloc<T>,
  typename std::allocator<void>::template rebind<T>::other>::value,
  std::default_delete<T>,
  AllocatorDeleter<Alloc>
  >::type;
}  // namespace allocator
}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_
