#include <benchmark/benchmark.h>
#include <memory>
#include <variant>
#include <functional>
#include <atomic>
#include <vector>
#include <cstdlib>

// ---------------------------------------------------------
// 1. CUSTOM ALLOCATOR TRACKING
// Intercepts and counts every heap allocation (new/malloc)
// ---------------------------------------------------------
static std::atomic<int> g_allocations{0};

void* operator new(std::size_t size) {
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(size);
  if (!p) throw std::bad_alloc{};
  return p;
}
void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }

// ---------------------------------------------------------
// 2. ARCHITECTURE MOCK (Isolating the ROS 2 Memory Layout)
// ---------------------------------------------------------
struct MockTimer {
    void call() {}
};

struct ReadyTimerWithExecutedCallback {
    std::shared_ptr<MockTimer> timer_ptr;
    std::function<void()> timer_was_executed;
};

struct ReadyEntityMock {
    std::variant<std::shared_ptr<int>, ReadyTimerWithExecutedCallback> entity;

    // LEGACY: The current ROS 2 rolling implementation
    std::function<void()> get_execute_function() const {
        return std::visit([](auto && e) -> std::function<void()> {
            using T = std::decay_t<decltype(e)>;
            if constexpr (std::is_same_v<T, std::shared_ptr<int>>) {
                return [](){};
            } else if constexpr (std::is_same_v<T, ReadyTimerWithExecutedCallback>) {
                // THE FLAW: Capturing shared_ptr (16B) + std::function (32B) = 48 Bytes.
                // This blows past the ~32 Byte Small Object Optimization (SOO) limit,
                // forcing a dynamic heap allocation every single time.
                return [shr_ptr = e.timer_ptr, cb = e.timer_was_executed]() {
                    shr_ptr->call();
                    cb();
                };
            }
            return nullptr;
        }, entity);
    }

    // OPTIMIZED: The proposed direct-execution fix
    void execute_direct() const {
        std::visit([](auto && e) {
            using T = std::decay_t<decltype(e)>;
            if constexpr (std::is_same_v<T, std::shared_ptr<int>>) {
                // no-op
            } else if constexpr (std::is_same_v<T, ReadyTimerWithExecutedCallback>) {
                if (e.timer_ptr) {
                    e.timer_ptr->call();
                    e.timer_was_executed();
                }
            }
        }, entity);
    }
};

// ---------------------------------------------------------
// 3. THE BENCHMARKS
// ---------------------------------------------------------
static void BM_Legacy_StdFunction_SOO_Blowout(benchmark::State& state) {
    auto timer = std::make_shared<MockTimer>();
    ReadyTimerWithExecutedCallback cb;
    cb.timer_ptr = timer;
    cb.timer_was_executed = [](){ benchmark::DoNotOptimize(1); };

    // Flood with 1000 entities
    std::vector<ReadyEntityMock> entities;
    for (int i = 0; i < 1000; ++i) {
        entities.push_back(ReadyEntityMock{cb});
    }

    for (auto _ : state) {
        // Reset allocation counter right before the hot path
        g_allocations.store(0, std::memory_order_relaxed);

        for (const auto& entity : entities) {
            // This line triggers the SOO heap allocation
            std::function<void()> exec = entity.get_execute_function();
            exec();
        }

        // Record exactly how many allocations happened in this spin loop
        state.counters["Heap_Allocations"] = g_allocations.load(std::memory_order_relaxed);
    }
}
BENCHMARK(BM_Legacy_StdFunction_SOO_Blowout);

static void BM_Optimized_Direct_Execution(benchmark::State& state) {
    auto timer = std::make_shared<MockTimer>();
    ReadyTimerWithExecutedCallback cb;
    cb.timer_ptr = timer;
    cb.timer_was_executed = [](){ benchmark::DoNotOptimize(1); };

    // Flood with 1000 entities
    std::vector<ReadyEntityMock> entities;
    for (int i = 0; i < 1000; ++i) {
        entities.push_back(ReadyEntityMock{cb});
    }

    for (auto _ : state) {
        // Reset allocation counter right before the hot path
        g_allocations.store(0, std::memory_order_relaxed);

        for (const auto& entity : entities) {
            // Direct visit execution (Zero allocations)
            entity.execute_direct();
        }

        // Record exactly how many allocations happened in this spin loop
        state.counters["Heap_Allocations"] = g_allocations.load(std::memory_order_relaxed);
    }
}
BENCHMARK(BM_Optimized_Direct_Execution);