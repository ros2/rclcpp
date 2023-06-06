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

#include <pthread.h>
#include <sched.h>

#include <iostream>

#include "rclcpp/threads/posix/thread.hpp"
#include "rclcpp/threads/posix/utilities.hpp"

static void set_pthread_attr(pthread_attr_t & native_attr, rclcpp::Thread::Attribute const & attr);
static void * thread_main(void * p);

namespace rclcpp
{

Thread::Thread(Attribute const * attr, ThreadFuncBase::UniquePtr func)
: handle_(NativeHandleType{}), name_(attr ? attr->get_name() : std::string{})
{
  Attribute::NativeAttributeType native_attr;
  int r = pthread_attr_init(&native_attr);
  detail::throw_if_error(r, "Error in pthread_attr_init ");

  if (attr != nullptr) {
    set_pthread_attr(native_attr, *attr);
  }

  NativeHandleType h;
  r = pthread_create(&h, &native_attr, thread_main, func.get());
  detail::throw_if_error(r, "Error in pthread_create ");

  if (attr == nullptr || !attr->get_run_as_detached()) {
    this->handle_ = h;
  }

  pthread_attr_destroy(&native_attr);

  func.release();
}

void Thread::apply_attr(Attribute const & attr)
{
  int r;
  int policy = attr.get_sched_policy();
#if __linux__
  if (policy != SCHED_FIFO && policy != SCHED_RR && policy != SCHED_OTHER) {
    sched_param param;
    param.sched_priority = attr.get_priority();
    r = pthread_setschedparam(pthread_self(), policy, &param);
    detail::throw_if_error(r, "Error in pthread_setschedparam ");
  }
#endif  // #if __linux__
}

namespace detail
{

ThreadAttribute::ThreadAttribute()
{
  NativeAttributeType attr;
  int r;

  r = pthread_attr_init(&attr);
  throw_if_error(r, "Error in pthread_attr_init ");

  r = pthread_attr_getschedpolicy(&attr, &sched_policy_);
  throw_if_error(r, "Error in pthread_attr_getschedpolicy ");

  r = pthread_attr_getstacksize(&attr, &stack_size_);
  throw_if_error(r, "Error in pthread_attr_getstacksize ");

  sched_param param;
  r = pthread_attr_getschedparam(&attr, &param);
  throw_if_error(r, "Error in pthread_attr_getschedparam ");
  priority_ = param.sched_priority;

  int flag;
  r = pthread_attr_getdetachstate(&attr, &flag);
  throw_if_error(r, "Error in pthread_attr_getdetachstate ");
  detached_flag_ = (flag == PTHREAD_CREATE_DETACHED);

  pthread_attr_destroy(&attr);
}


void apply_attr_to_current_thread(ThreadAttribute const & attr)
{
  int r;

#if __linux__
  CpuSet cpu_set = attr.get_affinity();
  CpuSet::NativeCpuSetType * native_cpu_set = cpu_set.native_cpu_set();
  if (native_cpu_set) {
    std::size_t alloc_size = cpu_set.alloc_size();
    r = pthread_setaffinity_np(pthread_self(), alloc_size, native_cpu_set);
    throw_if_error(r, "Error in sched_setaffinity ");
  }
#endif  // #if __linux__

  sched_param param;
  param.sched_priority = attr.get_priority();
  int policy = attr.get_sched_policy();
  r = pthread_setschedparam(pthread_self(), policy, &param);
  throw_if_error(r, "Error in sched_setscheduler");
}

}  // namespace  detail

}  // namespace  rclcpp

static void * thread_main(void * p)
{
  using rclcpp::detail::ThreadFuncBase;
  ThreadFuncBase::UniquePtr func(reinterpret_cast<ThreadFuncBase *>(p));

  try {
    func->run();
  } catch (...) {
    std::cerr << "failed to run thread" << std::endl;
    std::terminate();
  }

  return nullptr;
}

static void set_pthread_attr(pthread_attr_t & native_attr, rclcpp::Thread::Attribute const & attr)
{
  int r;
  using rclcpp::detail::throw_if_error;

#if defined(__linux__)
  rclcpp::detail::CpuSet affinity = attr.get_affinity();
  size_t cpu_size = CPU_ALLOC_SIZE(static_cast<std::size_t>(sysconf(_SC_NPROCESSORS_ONLN)));
  r = pthread_attr_setaffinity_np(&native_attr, cpu_size, affinity.native_cpu_set());
  throw_if_error(r, "Error in pthread_attr_setaffinity_np ");
#endif

  std::size_t stack_size = attr.get_stack_size();
  r = pthread_attr_setstacksize(&native_attr, stack_size);
  throw_if_error(r, "Error in pthread_attr_setstacksize ");

  int flag = attr.get_run_as_detached() ? PTHREAD_CREATE_DETACHED : PTHREAD_CREATE_JOINABLE;
  r = pthread_attr_setdetachstate(&native_attr, flag);
  throw_if_error(r, "Error in pthread_attr_setdetachstate ");

  int sched_policy = attr.get_sched_policy();
  if (sched_policy == SCHED_FIFO || sched_policy == SCHED_RR) {
    r = pthread_attr_setinheritsched(&native_attr, PTHREAD_EXPLICIT_SCHED);
    throw_if_error(r, "Error in pthread_attr_setinheritsched ");

    r = pthread_attr_setschedpolicy(&native_attr, sched_policy);
    throw_if_error(r, "Error in pthread_attr_setschedpolicy ");

    sched_param param;
    param.sched_priority = attr.get_priority();
    r = pthread_attr_setschedparam(&native_attr, &param);
    throw_if_error(r, "Error in pthread_attr_setschedparam ");
  }
}
