#pragma once
#include <cstddef>
#include <functional>

namespace pe {

// Runs fn(begin, end) over [0, count) split into chunks on a persistent worker pool.
// Small ranges run inline on the calling thread. fn must be safe to run concurrently on disjoint ranges.
void parallelFor(size_t count, const std::function<void(size_t, size_t)>& fn, size_t minChunk = 128);

} // namespace pe
