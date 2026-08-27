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
 * @brief Benchmark suite for evaluating the computational overhead and routing accuracy of the adaptive dispatcher.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "bm-dispatcher.h"

#include <cstdlib>

#include "dispatcher.h"
#include "dataset-generator.h"
#include "io.h"

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
        characteristics = probe_density<8>(cloud, 4096);
        decision        = evaluate_dispatch_decision(characteristics);

        benchmark::DoNotOptimize(characteristics);
        benchmark::DoNotOptimize(decision);
    }

    // Assign structural characteristics to benchmark output counters
    state.counters["effective_density"]  = characteristics.effective_density;
    state.counters["normalized_entropy"] = characteristics.normalized_entropy;
    state.counters["decision_is_gpu"]    = (decision == execution_target::sycl_gpu) ? 1.0 : 0.0; // 1 - GPU, 0 - CPUs
    state.counters["dataset_type"]       = static_cast<double>(type);
    state.counters["grid_dim"]           = 8;
    state.counters["sample_size"]        = 4096;

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

    std::size_t num_points      = state.range(0);
    std::vector<point_3d> cloud = load_point_cloud(env);
    spatial_characteristics characteristics {};
    execution_target decision {execution_target::cpu_multithreaded};

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
        characteristics = probe_density<8>(cloud, 4096);
        decision        = evaluate_dispatch_decision(characteristics);

        benchmark::DoNotOptimize(characteristics);
        benchmark::DoNotOptimize(decision);
    }

    // Assign structural characteristics to benchmark output counters
    state.counters["effective_density"]  = characteristics.effective_density;
    state.counters["normalized_entropy"] = characteristics.normalized_entropy;
    state.counters["decision_is_gpu"]    = (decision == execution_target::sycl_gpu) ? 1.0 : 0.0; // 1 - GPU, 0 - CPUs
    state.counters["grid_dim"]           = 8;
    state.counters["sample_size"]        = 4096;
    state.SetLabel("file=" + std::string(env));

    state.SetItemsProcessed(state.iterations() * num_points);
}
