// Copyright 2023 eSOL Co.,Ltd.
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

#ifndef RCLCPP__THREADS__STD__THREAD_ATTRIBUTE_HPP_
#define RCLCPP__THREADS__STD__THREAD_ATTRIBUTE_HPP_

#include <pthread.h>
#include <string>
#include <utility>

#include "rcl_yaml_param_parser/types.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

struct Thread;

namespace detail
{
struct ThreadAttribute;
}  // namespace detail

namespace this_thread
{
template<typename F, typename ... Args>
void run_with_thread_attribute(
  detail::ThreadAttribute & attr, F && f, Args && ... args);
}  // namespace this_thread

namespace detail
{

struct CpuSet
{
  using NativeCpuSetType = std::size_t;
  CpuSet() {}
  explicit CpuSet(std::size_t) {}
  CpuSet(const CpuSet &) {}
  CpuSet & operator=(const CpuSet &)
  {
    return *this;
  }
  CpuSet(CpuSet &&) = delete;
  CpuSet & operator=(CpuSet &&) = delete;
  ~CpuSet() {}
  void set(std::size_t) {}
  void unset(std::size_t) {}
  void clear() {}
  bool is_set(std::size_t)
  {
    return false;
  }
  std::size_t get_max_processors() const
  {
    return 0;
  }
  NativeCpuSetType native_cpu_set() const
  {
    return 0;
  }
};

struct ThreadAttribute
{
  using PriorityType = int;

  ThreadAttribute()
  : set_unavailable_items_(false) {}

  ThreadAttribute(const ThreadAttribute &) = default;
  ThreadAttribute(ThreadAttribute &&) = default;
  ThreadAttribute & operator=(const ThreadAttribute &) = default;
  ThreadAttribute & operator=(ThreadAttribute &&) = default;

  ThreadAttribute & set_affinity(CpuSet &)
  {
    set_unavailable_items_ = true;
    return *this;
  }
  CpuSet get_affinity()
  {
    return CpuSet{};
  }

  ThreadAttribute & set_stack_size(std::size_t)
  {
    set_unavailable_items_ = true;
    return *this;
  }
  std::size_t get_stack_size() const
  {
    return 0;
  }

  ThreadAttribute & set_priority(int prio)
  {
    (void)prio;
    set_unavailable_items_ = true;
    return *this;
  }
  int get_priority() const
  {
    return 0;
  }

  ThreadAttribute & set_run_as_detached(bool detach)
  {
    run_as_detached_ = detach;
    return *this;
  }
  bool get_run_as_detached() const
  {
    return run_as_detached_;
  }

  ThreadAttribute & set_name(std::string const &)
  {
    set_unavailable_items_ = true;
    return *this;
  }
  const char * get_name() const
  {
    return "";
  }

  void
  set_thread_attribute(
    const rcl_thread_attr_t &)
  {
    set_unavailable_items_ = true;
  }

  void swap(
    ThreadAttribute & other)
  {
    std::swap(*this, other);
  }

private:
  friend struct rclcpp::Thread;
  template<typename F, typename ... Args>
  friend void this_thread::run_with_thread_attribute(
    ThreadAttribute & attr, F && f, Args && ... args);

  bool set_unavailable_items_;
  bool run_as_detached_;
};

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__STD__THREAD_ATTRIBUTE_HPP_
