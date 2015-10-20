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

template<typename Allocator>
class AllocatorDeleter
{
  template<typename U>
  using AllocRebind = typename std::allocator_traits<Allocator>::template rebind_alloc<U>;

public:
  AllocatorDeleter()
  : allocator_(NULL)
  {
  }

  AllocatorDeleter(Allocator * a)
  : allocator_(a)
  {
  }

  template<typename U>
  AllocatorDeleter(const AllocatorDeleter<U> & a)
  {
    allocator_ = a.get_allocator();
  }

  template<typename U>
  void operator()(U * ptr)
  {
    std::allocator_traits<AllocRebind<U>>::destroy(*allocator_, ptr);
    std::allocator_traits<AllocRebind<U>>::deallocate(*allocator_, ptr, sizeof(U));
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
    std::is_same<Alloc, std::allocator<void>>::value,
    std::default_delete<T>,
    AllocatorDeleter<Alloc>
    >::type;
}
}

#endif
