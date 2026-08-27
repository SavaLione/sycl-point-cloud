/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Copyright (C) 2026 Savelii Pototskii (savalione.com)
 *
 * Author: Savelii Pototskii <savelii.pototskii@gmail.com>
 *
 * This file is part of sycl-point-cloud.
 *
 * sycl-point-cloud is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * sycl-point-cloud is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sycl-point-cloud. If not, see <https://www.gnu.org/licenses/>.
 */
/**
 * @file bm-compact-spatial-hashing.cpp
 * @brief Benchmark suite for evaluating the performance of compact spatial hashing and radius search execution on the CPU.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "bm-compact-spatial-hashing.h"

#include <sycl/sycl.hpp>
#include <chrono>

#include "point.h"
#include "dataset-generator.h"
#include "compact-spatial-hashing.h"
#include "spatial-hashing.h"
#include "io.h"

void bm_cpu_compact_spatial_hashing(benchmark::State &state)
{
    std::size_t const num_points = state.range(0);
    dataset_type const type      = static_cast<dataset_type>(state.range(1));

    auto const cloud      = generate_dataset(num_points, type);
    point_3d const query  = cloud[0]; // FIXME - The benchmark should pick N random points and then calculate median distribution of results
    float const radius    = 2.5f;
    float const cell_size = radius / 2.0f;

    for(auto _ : state)
    {
        auto start = std::chrono::high_resolution_clock::now(); // FIXME - All counters should be added to the benchmark results

        std::size_t const expected_capacity   = cloud.size() * 2;
        std::size_t const table_size          = get_next_prime(expected_capacity);
        compact_spatial_hash_table hash_table = build_compact_spatial_hash(cloud, cell_size, table_size);
        auto results                          = radius_search(query, radius, cloud, hash_table);

        auto end                                          = std::chrono::high_resolution_clock::now(); // FIXME - All counters should be added to the benchmark results
        std::chrono::duration<double, std::milli> elapsed = end - start;                               // FIXME - All counters should be added to the benchmark results

        benchmark::DoNotOptimize(results);
    }
    state.SetItemsProcessed(state.iterations() * num_points);
}

void bm_cpu_compact_spatial_hashing_file(benchmark::State &state)
{
    const char *env = std::getenv("BENCHMARK_POINT_CLOUD");
    if(!env)
    {
        state.SkipWithError("BENCHMARK_POINT_CLOUD environment variable is not set.");
        return;
    }

    std::size_t num_points      = state.range(0);
    std::vector<point_3d> cloud = load_point_cloud(env);
    point_3d const query        = {cloud[0]}; // FIXME - The benchmark should pick N random points and then calculate median distribution of results
    float const radius          = 2.5f;
    float const cell_size       = radius / 2.0f;

    // Make the cloud smaller
    if(num_points != 0)
    {
        if(cloud.size() >= num_points)
        {
            cloud.resize(num_points);
        }
        else
        {
            state.SkipWithMessage("Provided cloud size is smaller than the suggested testing cloud size.");
            return;
        }
    }
    else
    {
        num_points = cloud.size();
    }

    if(cloud.empty())
    {
        state.SkipWithError("Provided cloud is empty.");
        return;
    }

    for(auto _ : state)
    {
        auto start = std::chrono::high_resolution_clock::now(); // FIXME - All counters should be added to the benchmark results

        std::size_t const expected_capacity   = cloud.size() * 2;
        std::size_t const table_size          = get_next_prime(expected_capacity);
        compact_spatial_hash_table hash_table = build_compact_spatial_hash(cloud, cell_size, table_size);
        auto results                          = radius_search(query, radius, cloud, hash_table);

        auto end                                          = std::chrono::high_resolution_clock::now(); // FIXME - All counters should be added to the benchmark results
        std::chrono::duration<double, std::milli> elapsed = end - start;                               // FIXME - All counters should be added to the benchmark results

        benchmark::DoNotOptimize(results);
    }

    state.SetLabel("file=" + std::string(env));

    state.SetItemsProcessed(state.iterations() * num_points);
}
