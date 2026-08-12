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
 * @file bm-dispatcher.cpp
 * @brief The SYCL dispatcher benchmark
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bm-dispatcher.h"

#include "dispatcher.h"
#include "dataset-generator.h"
#include "io.h"
#include <cstdlib>

void bm_adaptive_dispatcher(benchmark::State &state)
{
    std::size_t const num_points = state.range(0);
    dataset_type const type      = static_cast<dataset_type>(state.range(1));

    // Generate point cloud matching parameter arguments
    auto const cloud = generate_dataset(num_points, type);

    spatial_characteristics characteristics {};
    execution_target decision {execution_target::cpu_multithreaded};

    for(auto _ : state)
    {
        characteristics = analyze_point_cloud_fast(cloud, 256);
        decision        = evaluate_dispatch_decision(characteristics);

        benchmark::DoNotOptimize(characteristics);
        benchmark::DoNotOptimize(decision);
    }

    // Assign structural characteristics to benchmark output counters
    state.counters["effective_density"]  = characteristics.effective_density;
    state.counters["normalized_entropy"] = characteristics.normalized_entropy;

    // Binary indicator for target execution pathway (1.0 = SYCL GPU, 0.0 = Multi-threaded CPU)
    state.counters["decision_is_gpu"] = (decision == execution_target::sycl_gpu) ? 1.0 : 0.0;

    state.counters["dataset_type"] = static_cast<double>(type);

    state.SetItemsProcessed(state.iterations() * num_points);
}

void bm_adaptive_dispatcher_file(benchmark::State &state)
{
    const char *env = std::getenv("BENCHMARK_POINT_CLOUD");
    if(!env)
    {
        state.SkipWithError("BENCHMARK_POINT_CLOUD environment variable is not set.");
        return;
    }

    std::vector<point_3d> const cloud = load_point_cloud(env);
    spatial_characteristics characteristics {};
    execution_target decision {execution_target::cpu_multithreaded};

    for(auto _ : state)
    {
        characteristics = analyze_point_cloud_fast(cloud, 256);
        decision        = evaluate_dispatch_decision(characteristics);

        benchmark::DoNotOptimize(characteristics);
        benchmark::DoNotOptimize(decision);
    }

    // Assign structural characteristics to benchmark output counters
    state.counters["effective_density"]  = characteristics.effective_density;
    state.counters["normalized_entropy"] = characteristics.normalized_entropy;

    // Binary indicator for target execution pathway (1.0 = SYCL GPU, 0.0 = Multi-threaded CPU)
    state.counters["decision_is_gpu"] = (decision == execution_target::sycl_gpu) ? 1.0 : 0.0;

    state.SetItemsProcessed(state.iterations() * cloud.size());
}