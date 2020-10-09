#include <array>
#include <chrono>

#include <rclcpp/clock.hpp>
#include <rclcpp/timer.hpp>

using namespace std::chrono_literals;

namespace rclcpp
{
namespace executors
{

struct TimersHeap
{
public:
  /**
   * @brief Construct a new Timers Heap object
   */
  TimersHeap()
  {
    clock = rclcpp::Clock(RCL_ROS_TIME);
    size = 0;
  }

  /**
   * @brief Adds a new TimerBase to the heap
   * @param timer the timer to be added
   * @return int 0 if success, -1 if the heap is already full
   */
  inline int add_timer(rclcpp::TimerBase::SharedPtr timer)
  {
    // Add timer to vector and order by expiration time
    timers_storage.emplace_back(TimerInternal(timer, clock));
    std::sort(timers_storage.begin(), timers_storage.end());

    // Clear heap as the pointers likely become invalid after the above emplace_back.
    heap.clear();
    for (auto& t : timers_storage) {
      heap.push_back(&t);
    }

    size = heap.size();

    return 0;
  }

  /**
   * @brief Get the time before the first timer in the heap expires
   *
   * @return std::chrono::nanoseconds to wait, 0 if the timer is already expired
   */
  inline std::chrono::nanoseconds get_head_timeout()
  {
    auto min_timeout = std::chrono::nanoseconds::max();
    if (peek(&head) == 0) {
      min_timeout = std::chrono::nanoseconds(head->expire_time - clock.now().nanoseconds());
    }

    if (min_timeout < 0ns) {
      min_timeout = 0ns;
    }
    return min_timeout;
  }

  /**
   * @brief Executes all the ready timers in the heap
   * These timers are refreshed and added back to the heap
   * NOTE: may block indefinitely if the time for processing callbacks is longer than the timers period
   */
  inline void execute_ready_timers()
  {
    while (peek(&head) == 0 && head->timer->is_ready()) {
      head->timer->execute_callback();

      head->refresh(clock);
      remove_at(0);
      push(head);
    }
  }

  inline void clear_all()
  {
    // Todo: Implement clear all timers.
  }

  inline void remove_timer(rclcpp::TimerBase::SharedPtr timer)
  {
    // Todo: Implement
    (void)timer;
  }

private:
  struct TimerInternal
  {
    inline TimerInternal()
    {
      timer = nullptr;
      expire_time = INT64_MAX;
    }

    inline TimerInternal(rclcpp::TimerBase::SharedPtr t, rclcpp::Clock& clock)
    {
      timer = t;
      refresh(clock);
    }

    inline void refresh(rclcpp::Clock& clock)
    {
      expire_time = clock.now().nanoseconds() + timer->time_until_trigger().count();
    }

    bool operator < (const TimerInternal& t) const
    {
        return (timer->time_until_trigger() < t.timer->time_until_trigger());
    }

    rclcpp::TimerBase::SharedPtr timer;
    int64_t expire_time;
  };

  using TimerInternalPtr = TimerInternal*;

  inline void push(TimerInternalPtr x)
  {
    size_t i = size++;
    heap[i] = x;
    while (i && (x->expire_time < heap[(i-1)/2]->expire_time)) {
      heap[i] = heap[(i-1)/2];
      heap[(i-1)/2] = x;
      i = (i-1)/2;
    }
  }

  inline void remove_at(size_t i)
  {
    TimerInternalPtr y = heap[--size];
    heap[i] = y;

    // Heapify upwards.
    while (i > 0) {
      size_t parent = (i-1)/2;
      if (y->expire_time < heap[parent]->expire_time) {
        heap[i] = heap[parent];
        heap[parent] = y;
        i = parent;
      } else {
        break;
      }
    }

    // Heapify downwards
    while (2*i + 1 < size) {
      size_t hi = i;
      size_t left = 2*i+1;
      size_t right = left + 1;
      if (y->expire_time > heap[left]->expire_time) {
        hi = left;
      }
      if (right < size && (heap[hi]->expire_time > heap[right]->expire_time)) {
        hi = right;
      }
      if (hi != i) {
        heap[i] = heap[hi];
        heap[hi] = y;
        i = hi;
      } else {
        break;
      }
    }
  }

  inline int pop(TimerInternalPtr x)
  {
    if (size == 0) {
      // The heap is empty, can't pop
      return -1;
    }

    x = heap[0];
    remove_at(0);
    return 0;
  }

  inline int peek(TimerInternalPtr* x)
  {
    if (size == 0) {
      // The heap is empty, can't peek
      return -1;
    }

    *x = heap[0];
    return 0;
  }

  inline int remove(TimerInternalPtr x)
  {
    size_t i;
    for (i = 0; i < size; ++i) {
      if (x == heap[i]) {
        break;
      }
    }
    if (i == size) {
      return -1;
    }

    remove_at(i);
    return 0;
  }

  // Vector to keep ownership of the timers
  std::vector<TimerInternal> timers_storage;
  // Vector of pointers to stored timers used to implement the priority queue
  std::vector<TimerInternalPtr> heap;
  // Helper to access first element in the heap
  TimerInternalPtr head;
  // Current number of elements in the heap
  size_t size;
  // Clock to update expiration times and generate timeouts
  rclcpp::Clock clock;
};

}
}
