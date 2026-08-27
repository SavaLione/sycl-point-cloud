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
 * @file bm-probe-density.h
 * @brief Benchmark declarations for evaluating structural resolution and normalized spatial entropy heuristics on point clouds.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef BM_PROBE_DENSITY_H
#define BM_PROBE_DENSITY_H

#include "bm-common.h"

/**
 * @brief Benchmarks the execution latency and metric accuracy of the spatial density probe across varying grid resolutions and sample sizes.
 * 
 * @param state Google Benchmark state object containing invocation parameters:
 *              - state.range(0): Total number of points in the dataset.
 *              - state.range(1): Dataset topology type (dataset_type enum).
 *              - state.range(2): Linear voxel grid resolution (grid_dim).
 *              - state.range(3): Sub-sample size for heuristic evaluation.
 */
void bm_probe_density_grid_sample(benchmark::State &state);

BENCHMARK(bm_probe_density_grid_sample)->Apply(default_args_probe_density_grid_sample);

#endif // BM_PROBE_DENSITY_H