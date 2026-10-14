#include "core/Parallel.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
#include <immintrin.h>
#endif

namespace pe {

namespace {

void cpuRelax() {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}

// A solver step issues dozens of short jobs back to back. Workers spin for a moment between jobs,
// because putting a thread to sleep and waking it costs more than most of the jobs themselves,
// and only block on the condition variable once the burst is over.
class Pool {
public:
    Pool() {
        // a few thousand particles per job: past ~8 threads the extra wakeups cost more than they save
        unsigned n = std::clamp(std::thread::hardware_concurrency(), 1u, 8u) - 1;
        for (unsigned i = 0; i < n; ++i) workers_.emplace_back([this] { workerLoop(); });
    }

    ~Pool() {
        {
            std::lock_guard lock(mutex_);
            quit_.store(true, std::memory_order_relaxed);
            generation_.fetch_add(1, std::memory_order_release);
        }
        wake_.notify_all();
        for (auto& t : workers_) t.join();
    }

    size_t threads() const { return workers_.size() + 1; }

    void run(size_t count, size_t chunk, const std::function<void(size_t, size_t)>& fn) {
        job_ = &fn;
        count_ = count;
        chunk_ = chunk;
        next_.store(0, std::memory_order_relaxed);
        busy_.store(workers_.size(), std::memory_order_relaxed);
        bool anySleeping;
        {
            std::lock_guard lock(mutex_);
            generation_.fetch_add(1, std::memory_order_release);
            anySleeping = sleeping_ > 0;
        }
        if (anySleeping) wake_.notify_all();

        work();
        while (busy_.load(std::memory_order_acquire) != 0) cpuRelax();
    }

private:
    static constexpr auto kSpinTime = std::chrono::microseconds(500);

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable wake_;
    int sleeping_ = 0;
    std::atomic<bool> quit_{false};
    std::atomic<unsigned long long> generation_{0};
    std::atomic<size_t> next_{0}, busy_{0};
    const std::function<void(size_t, size_t)>* job_ = nullptr;
    size_t count_ = 0, chunk_ = 1;

    void work() {
        for (size_t begin; (begin = next_.fetch_add(chunk_, std::memory_order_relaxed)) < count_;)
            (*job_)(begin, std::min(begin + chunk_, count_));
    }

    void workerLoop() {
        unsigned long long seen = 0;
        for (;;) {
            auto spinUntil = std::chrono::steady_clock::now() + kSpinTime;
            for (int spin = 0; generation_.load(std::memory_order_acquire) == seen; ++spin) {
                if ((spin & 255) != 0 || std::chrono::steady_clock::now() < spinUntil) {
                    cpuRelax();
                    continue;
                }
                std::unique_lock lock(mutex_);
                ++sleeping_;
                wake_.wait(lock, [&] { return generation_.load(std::memory_order_acquire) != seen; });
                --sleeping_;
            }
            seen = generation_.load(std::memory_order_acquire);
            if (quit_.load(std::memory_order_relaxed)) return;
            work();
            busy_.fetch_sub(1, std::memory_order_release);
        }
    }
};

Pool& pool() {
    static Pool p;
    return p;
}

} // namespace

void parallelFor(size_t count, const std::function<void(size_t, size_t)>& fn, size_t minChunk) {
    if (count == 0) return;
    Pool& p = pool();
    if (count <= minChunk || p.threads() == 1) {
        fn(0, count);
        return;
    }
    // a few chunks per thread so uneven work (dense vs sparse regions) still balances
    size_t chunk = std::max(minChunk, (count + 4 * p.threads() - 1) / (4 * p.threads()));
    p.run(count, chunk, fn);
}

} // namespace pe
