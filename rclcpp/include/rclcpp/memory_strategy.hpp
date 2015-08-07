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
class SharedPtrContainer
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(SharedPtrContainer);
  virtual std::shared_ptr<T> & operator[](size_t pos) = 0;
  virtual std::shared_ptr<T> & at(size_t pos) = 0;
  virtual size_t size() const = 0;
  virtual std::shared_ptr<T> * data() = 0;
  virtual std::shared_ptr<T> * begin() = 0;
  virtual std::shared_ptr<T> * end() = 0;
  virtual void add_vector(const std::vector<std::shared_ptr<T>> & vec) = 0;
  virtual void reset_container() = 0;
};


template<typename T>
class DefaultSharedPtrContainer : public SharedPtrContainer<T>
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(DefaultSharedPtrContainer);

  std::shared_ptr<T> & operator[](size_t pos)
  {
    return container_[pos];
  }
  std::shared_ptr<T> & at(size_t pos)
  {
    return container_.at(pos);
  }
  size_t size() const
  {
    return container_.size();
  }

  std::shared_ptr<T> * data()
  {
    return container_.data();
  }

  std::shared_ptr<T> * begin()
  {
    return data();
  }

  std::shared_ptr<T> * end()
  {
    return data() + size();
  }

  void add_vector(const std::vector<std::shared_ptr<T>> & vec)
  {
    for (auto & entry : vec) {
      container_.push_back(entry);
    }
  }

  void reset_container()
  {
    container_.clear();
  }

private:
  std::vector<std::shared_ptr<T>> container_;
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

  virtual SharedPtrContainer<subscription::SubscriptionBase>::SharedPtr
  get_subscription_container_interface()
  {
    return get_default_container_interface<subscription::SubscriptionBase>();
  }

  virtual SharedPtrContainer<service::ServiceBase>::SharedPtr get_service_container_interface()
  {
    return get_default_container_interface<service::ServiceBase>();
  }

  virtual SharedPtrContainer<client::ClientBase>::SharedPtr get_client_container_interface()
  {
    return get_default_container_interface<client::ClientBase>();
  }

  virtual SharedPtrContainer<timer::TimerBase>::SharedPtr get_timer_container_interface()
  {
    return get_default_container_interface<timer::TimerBase>();
  }

private:
  template<typename T>
  typename SharedPtrContainer<T>::SharedPtr get_default_container_interface()
  {
    return std::shared_ptr<SharedPtrContainer<T>>(new DefaultSharedPtrContainer<T>);
  }
};


MemoryStrategy::SharedPtr create_default_strategy()
{
  return std::make_shared<MemoryStrategy>(MemoryStrategy());
}

}  /* memory_strategy */

}  /* rclcpp */

#endif
