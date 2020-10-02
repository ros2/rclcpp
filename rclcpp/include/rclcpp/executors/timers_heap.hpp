#include <array>
#include <chrono>

#include <rclcpp/clock.hpp>
#include <rclcpp/timer.hpp>

using namespace std::chrono_literals;

namespace rclcpp
{
namespace executors
{

template<size_t MAX_SIZE = 20>
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
    if (size == MAX_SIZE) {
      // The heap is full, can't push
      return -1;
    }

    // Create a TimerInternal object to store the timer
    timers_storage[size] = TimerInternal(timer, clock);
    TimerInternalPtr t = &(timers_storage[size]);

    // Push a TimerInternalPtr into the heap
    push(t);
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

    rclcpp::TimerBase::SharedPtr timer;
    int64_t expire_time;
  };

  using TimerInternalPtr = TimerInternal*;

  inline void push(TimerInternalPtr x)
  {
    size_t i = size++;
    a[i] = x;
    while (i && (x->expire_time < a[(i-1)/2]->expire_time)) {
      a[i] = a[(i-1)/2];
      a[(i-1)/2] = x;
      i = (i-1)/2;
    }
  }

  inline void remove_at(size_t i)
  {
    TimerInternalPtr y = a[--size];
    a[i] = y;

    // Heapify upwards.
    while (i > 0) {
      size_t parent = (i-1)/2;
      if (y->expire_time < a[parent]->expire_time) {
        a[i] = a[parent];
        a[parent] = y;
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
      if (y->expire_time > a[left]->expire_time) {
        hi = left;
      }
      if (right < size && (a[hi]->expire_time > a[right]->expire_time)) {
        hi = right;
      }
      if (hi != i) {
        a[i] = a[hi];
        a[hi] = y;
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

    x = a[0];
    remove_at(0);
    return 0;
  }

  inline int peek(TimerInternalPtr* x)
  {
    if (size == 0) {
      // The heap is empty, can't peek
      return -1;
    }

    *x = a[0];
    return 0;
  }

  inline int remove(TimerInternalPtr x)
  {
    size_t i;
    for (i = 0; i < size; ++i) {
      if (x == a[i]) {
        break;
      }
    }
    if (i == size) {
      return -1;
    }

    remove_at(i);
    return 0;
  }

  // Array to keep ownership of the timers
  std::array<TimerInternal, MAX_SIZE> timers_storage;
  // Array of pointers to stored timers used to implement the priority queue
  std::array<TimerInternalPtr, MAX_SIZE> a;
  // Helper to access first element in the heap
  TimerInternalPtr head;
  // Current number of elements in the heap
  size_t size;
  // Clock to update expiration times and generate timeouts
  rclcpp::Clock clock;
};

}
}
