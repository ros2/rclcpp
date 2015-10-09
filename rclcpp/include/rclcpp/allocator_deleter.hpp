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

template<typename T, typename Allocator>
class AllocatorDeleter
{
public:
  AllocatorDeleter()
  : allocator_(new Allocator)
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

private:
  Allocator * allocator_;
};

#endif
