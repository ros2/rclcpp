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

#ifndef RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_
#define RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_

#include <pthread.h>
#include <string>
#include <utility>

#include "rcl_yaml_param_parser/types.h"
#include "rclcpp/visibility_control.hpp"

#ifdef __linux__
#include "rclcpp/threads/posix/linux/cpu_set.hpp"
#endif

namespace rclcpp
{

namespace detail
{

struct ThreadAttribute
{
  ThreadAttribute();

  ThreadAttribute(const ThreadAttribute &) = default;
  ThreadAttribute(ThreadAttribute &&) = default;
  ThreadAttribute & operator=(const ThreadAttribute &) = default;
  ThreadAttribute & operator=(ThreadAttribute &&) = default;

  using NativeAttributeType = pthread_attr_t;

  ThreadAttribute & set_affinity(CpuSet cs)
  {
    cpu_set_ = std::move(cs);
    return *this;
  }
  const CpuSet & get_affinity() const
  {
    return cpu_set_;
  }

  ThreadAttribute & set_sched_policy(rcl_thread_scheduling_policy_type_t sp)
  {
    sched_policy_ = rcl_scheduling_policy_to_sched_policy(sp);
    return *this;
  }
  int get_sched_policy() const
  {
    return sched_policy_;
  }

  ThreadAttribute & set_stack_size(std::size_t sz)
  {
    stack_size_ = sz;
    return *this;
  }
  std::size_t get_stack_size() const
  {
    return stack_size_;
  }

  ThreadAttribute & set_priority(int prio)
  {
    priority_ = prio;
    return *this;
  }
  int get_priority() const
  {
    return priority_;
  }

  ThreadAttribute & set_run_as_detached(bool detach)
  {
    detached_flag_ = detach;
    return *this;
  }
  bool get_run_as_detached() const
  {
    return detached_flag_;
  }

  ThreadAttribute & set_name(std::string name)
  {
    name_ = std::move(name);
    return *this;
  }
  const std::string & get_name() const
  {
    return name_;
  }

  void
  set_thread_attribute(
    const rcl_thread_attr_t & attr)
  {
    CpuSet cpu_set(attr.core_affinity);
    set_affinity(std::move(cpu_set));
    set_sched_policy(attr.scheduling_policy);
    set_priority(attr.priority);
    set_name(attr.name);
  }

  void
  swap(
    ThreadAttribute & other)
  {
    using std::swap;
    swap(cpu_set_, other.cpu_set_);
    swap(sched_policy_, other.sched_policy_);
    swap(stack_size_, other.stack_size_);
    swap(priority_, other.priority_);
    swap(detached_flag_, other.detached_flag_);
    swap(name_, other.name_);
  }

private:
  CpuSet cpu_set_;
  int sched_policy_;
  std::size_t stack_size_;
  int priority_;
  bool detached_flag_;
  std::string name_;

  int rcl_scheduling_policy_to_sched_policy(
    rcl_thread_scheduling_policy_type_t sched_policy)
  {
    switch (sched_policy) {
#ifdef SCHED_FIFO
      case RCL_THREAD_SCHEDULING_POLICY_FIFO:
        return SCHED_FIFO;
#endif
#ifdef SCHED_RR
      case RCL_THREAD_SCHEDULING_POLICY_RR:
        return SCHED_RR;
#endif
#ifdef SCHED_OTHER
      case RCL_THREAD_SCHEDULING_POLICY_OTHER:
        return SCHED_OTHER;
#endif
#ifdef SCHED_IDLE
      case RCL_THREAD_SCHEDULING_POLICY_IDLE:
        return SCHED_IDLE;
#endif
#ifdef SCHED_BATCH
      case RCL_THREAD_SCHEDULING_POLICY_BATCH:
        return SCHED_BATCH;
#endif
#ifdef SCHED_SPORADIC
      case RCL_THREAD_SCHEDULING_POLICY_SPORADIC:
        return SCHED_SPORADIC;
#endif
      /* Todo: Necessity and setting method need to be considered
      #ifdef SCHED_DEADLINE
          case RCL_THREAD_SCHEDULING_POLICY_DEADLINE:
            return SCHED_DEADLINE;
            break;
      #endif
      */
      default:
        throw std::runtime_error("Invalid scheduling policy");
    }
    return -1;
  }
};

inline void swap(ThreadAttribute & a, ThreadAttribute & b)
{
  a.swap(b);
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_
