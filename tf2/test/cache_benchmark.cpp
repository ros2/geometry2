// Copyright 2024, Open Source Robotics Foundation, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation, Inc. nor the
//      names of its contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <benchmark/benchmark.h>

#include <chrono>
#include <tuple>
#include <vector>

#include "tf2/time_cache.hpp"

// Simulates 5 transforms, 10s worth of data at 200 Hz in a buffer that is
// completely full.
static void benchmark_insertion(benchmark::State & state)
{
  constexpr tf2::Duration max_storage_time = std::chrono::seconds(10);
  const int num_tform = 5;
  const int freq_hz = 200;
  const tf2::Duration dt = std::chrono::nanoseconds(1'000'000'000 / freq_hz);

  // Distinct transforms.
  std::vector<tf2::TransformStorage> example_items{};
  for (int tform_index = 0; tform_index < num_tform; ++tform_index) {
    tf2::TransformStorage stor{};
    stor.child_frame_id_ = tform_index;
    stor.translation_.setValue(tform_index, 0.0, 0.0);
    stor.rotation_.setValue(0.0, 0.0, 0.0, 1.0);
    example_items.emplace_back(stor);
  }

  // Insert data to cache
  auto insert_data = [ =, &example_items](
    tf2::TimeCache & cache,
    tf2::TimePoint timestamp,
    int step,
    tf2::TimePoint target_timestamp) {
      while (timestamp < target_timestamp) {
        for (int tform_index = 0; tform_index < num_tform; ++tform_index) {
          example_items[tform_index].frame_id_ = step;
          example_items[tform_index].stamp_ = timestamp;
          cache.insertData(example_items[tform_index]);
        }
        timestamp += dt;
        ++step;
      }
      return std::make_tuple(timestamp, step);
    };

  // First, fill the cache with max storage amount (the limit).
  tf2::TimeCache fill_cache(max_storage_time);
  const auto [fill_timestamp, fill_timestep] = insert_data(
      fill_cache,
      tf2::TimePointZero,
      0,
      tf2::TimePointZero + max_storage_time);

  // Now profile adding new data to the copied cache.
  const tf2::TimePoint target_timestamp = fill_timestamp + max_storage_time;
  for (auto _ : state) {
    // Don't profile construction (copying) of the cache.
    state.PauseTiming();
    tf2::TimeCache cache(fill_cache);
    state.ResumeTiming();

    insert_data(
        cache,
        fill_timestamp,
        fill_timestep,
        target_timestamp);
  }
}

BENCHMARK(benchmark_insertion);
