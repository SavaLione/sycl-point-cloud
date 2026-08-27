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
 * @file bm-probe-density.cpp
 * @brief Benchmark declarations for evaluating structural resolution and normalized spatial entropy heuristics on point clouds.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "bm-probe-density.h"

#include "probe-density.h"
#include "dispatcher.h"
#include "dataset-generator.h"

void bm_probe_density_grid_sample(benchmark::State &state)
{
    std::size_t const num_points  = state.range(0);
    dataset_type const type       = static_cast<dataset_type>(state.range(1));
    std::size_t const grid_dim    = state.range(2);
    std::size_t const sample_size = state.range(3);

    auto const cloud = generate_dataset(num_points, type);
    spatial_characteristics characteristics {};
    execution_target decision {execution_target::cpu_multithreaded};

    for(auto _ : state)
    {
        switch(grid_dim)
        {
            case 4:
                if(sample_size == 256)
                {
                    characteristics = probe_density<4>(cloud, 256);
                }
                else if(sample_size == 1024)
                {
                    characteristics = probe_density<4>(cloud, 1024);
                }
                else if(sample_size == 4096)
                {
                    characteristics = probe_density<4>(cloud, 4096);
                }
                else
                {
                    state.SkipWithError("Unknown sample_size " + std::to_string(sample_size));
                }
                break;
            case 8:
                if(sample_size == 256)
                {
                    characteristics = probe_density<8>(cloud, 256);
                }
                else if(sample_size == 1024)
                {
                    characteristics = probe_density<8>(cloud, 1024);
                }
                else if(sample_size == 4096)
                {
                    characteristics = probe_density<8>(cloud, 4096);
                }
                else
                {
                    state.SkipWithError("Unknown sample_size " + std::to_string(sample_size));
                }
                break;
            case 16:
                if(sample_size == 256)
                {
                    characteristics = probe_density<16>(cloud, 256);
                }
                else if(sample_size == 1024)
                {
                    characteristics = probe_density<16>(cloud, 1024);
                }
                else if(sample_size == 4096)
                {
                    characteristics = probe_density<16>(cloud, 4096);
                }
                else
                {
                    state.SkipWithError("Unknown sample_size " + std::to_string(sample_size));
                }
                break;
            default:
                state.SkipWithError("Unknown grid_dim " + std::to_string(grid_dim));
                break;
        }
        decision = evaluate_dispatch_decision(characteristics);

        benchmark::DoNotOptimize(characteristics);
        benchmark::DoNotOptimize(decision);
    }

    // Assign structural characteristics to benchmark output counters
    state.counters["effective_density"]  = characteristics.effective_density;
    state.counters["normalized_entropy"] = characteristics.normalized_entropy;
    state.counters["decision_is_gpu"]    = (decision == execution_target::sycl_gpu) ? 1.0 : 0.0;
    state.counters["dataset_type"]       = static_cast<double>(type);
    state.counters["grid_dim"]           = grid_dim;
    state.counters["sample_size"]        = sample_size;

    state.SetItemsProcessed(state.iterations() * num_points);
}