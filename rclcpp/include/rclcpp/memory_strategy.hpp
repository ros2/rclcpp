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

#ifndef RCLCPP_RCLCPP_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_MEMORY_STRATEGY_HPP_

#include <iterator>
#include <vector>

#include <rclcpp/any_executable.hpp>

namespace rclcpp
{

// TODO move HandleType somewhere where it makes sense
enum class HandleType {subscriber_handle, service_handle, client_handle, guard_condition_handle};

namespace executor
{
class Executor;
}

namespace memory_strategy
{

template<typename T>
class ContainerInterface
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ContainerInterface);
  virtual T& operator[](size_t pos) = 0;
  virtual T& at(size_t pos) = 0;
  virtual size_t size() const = 0;
  virtual T* data() = 0;
  virtual T* begin() = 0;
  virtual T* end() = 0;
  virtual void add_vector(const std::vector<T> &vec) = 0;
  virtual void reset_container() = 0;
};


template<typename T>
class DefaultContainerInterface : public ContainerInterface<T>
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(DefaultContainerInterface);

  T& operator[](size_t pos)
  {
    return container_[pos];
  }
  T& at(size_t pos)
  {
    return container_.at(pos);
  }
  size_t size() const
  {
    return container_.size();
  }

  T* data()
  {
    return container_.data();
  }

  T* begin()
  {
    return data();
  }

  T* end()
  {
    return data() + size();
  }

  void add_vector(const std::vector<T> &vec)
  {
    for (auto & entry : vec)
    {
      container_.push_back(entry);
    }
  }

  void reset_container()
  {
    container_.clear();
  }

private:
  std::vector<T> container_;
};

class MemoryStrategy
{

  friend class executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(MemoryStrategy);
  virtual void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    (void)type;
    return static_cast<void **>(alloc(sizeof(void *) * number_of_handles));
  }

  virtual void return_handles(HandleType type, void ** handles)
  {
    (void)type;
    this->free(handles);
  }

  virtual executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    return executor::AnyExecutable::SharedPtr(new executor::AnyExecutable);
  }

  virtual void * alloc(size_t size)
  {
    return std::malloc(size);
  }

  virtual void free(void * ptr)
  {
    return std::free(ptr);
  }

  virtual ContainerInterface<subscription::SubscriptionBase::SharedPtr>::SharedPtr get_subscription_container_interface()
  {
    return get_default_container_interface<subscription::SubscriptionBase::SharedPtr>();
  }

  virtual ContainerInterface<service::ServiceBase::SharedPtr>::SharedPtr get_service_container_interface()
  {
    return get_default_container_interface<service::ServiceBase::SharedPtr>();
  }

  virtual ContainerInterface<client::ClientBase::SharedPtr>::SharedPtr get_client_container_interface()
  {
    return get_default_container_interface<client::ClientBase::SharedPtr>();
  }

  virtual ContainerInterface<timer::TimerBase::SharedPtr>::SharedPtr get_timer_container_interface()
  {
    return get_default_container_interface<timer::TimerBase::SharedPtr>();
  }

private:
  template<typename T>
  typename ContainerInterface<T>::SharedPtr get_default_container_interface()
  {
    printf("Got default container interface\n");
    return std::shared_ptr<ContainerInterface<T>>(new DefaultContainerInterface<T>);
  }
};


MemoryStrategy::SharedPtr create_default_strategy()
{
  return std::make_shared<MemoryStrategy>(MemoryStrategy());
}

}  /* memory_strategy */

}  /* rclcpp */

#endif
