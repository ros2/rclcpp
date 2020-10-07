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
  TimersHeap() = default;

  /**
   * @brief Adds a new TimerBase to the queue
   * @param timer the timer to be added
   * @return int 0 if success, -1 if the queue is already full
   */
  inline void add_timer(rclcpp::TimerBase::SharedPtr timer)
  {
    timers_queue.emplace_back(std::move(timer));
    reorder_queue();
  }

  /**
   * @brief Get the time before the first timer in the queue expires
   *
   * @return std::chrono::nanoseconds to wait, can return a negative number
   * if the timer is already expired
   */
  inline std::chrono::nanoseconds get_head_timeout()
  {
    auto min_timeout = std::chrono::nanoseconds::max();

    auto head = peek();

    if (head != nullptr) {
      min_timeout = head->time_until_trigger();
    }

    return min_timeout;
  }

  /**
   * @brief Executes all the ready timers in the queue
   * These timers are refreshed and added back to the queue
   */
  inline void execute_ready_timers()
  {
    auto head = peek();

    while (head != nullptr && head->is_ready()) {
      head->execute_callback();
      head = peek();
    }

    reorder_queue();
  }

  inline void clear_all()
  {
    timers_queue.clear();
  }

  inline void remove_timer(rclcpp::TimerBase::SharedPtr timer)
  {
    for (auto it = timers_queue.begin(); it != timers_queue.end(); ++it) {
      if((*it).get() == timer.get()) {
        timers_queue.erase(it);
        break;
      }
    }
  }

private:

  struct timer_less_than_comparison
  {
    inline bool operator() (
      const rclcpp::TimerBase::SharedPtr& timer1,
      const rclcpp::TimerBase::SharedPtr& timer2)
    {
      return (timer1->time_until_trigger() < timer2->time_until_trigger());
    }
  };

  inline void reorder_queue()
  {
    std::sort(timers_queue.begin(), timers_queue.end(), timer_less_than_comparison());
  }

  inline rclcpp::TimerBase::SharedPtr peek()
  {
    if (timers_queue.empty()) {
      return nullptr;
    }
    return timers_queue.front();
  }

  // Ordered queue of timers
  std::vector<rclcpp::TimerBase::SharedPtr> timers_queue;
};

}
}
