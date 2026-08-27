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
 * @file bm-sycl-compact-spatial-hashing.h
 * @brief Benchmark suite for evaluating SYCL-accelerated compact spatial hashing across heterogeneous target devices.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef BM_SYCL_COMPACT_SPATIAL_HASHING_H
#define BM_SYCL_COMPACT_SPATIAL_HASHING_H

#include "bm-common.h"

/**
 * @brief Evaluates the computation latency of compact spatial hashing construction and radius search on a SYCL-compatible GPU target.
 * 
 * @param[in,out] state Google Benchmark state object containing invocation parameters:
 *                      - state.range(0): Total number of points in the dataset.
 *                      - state.range(1): Dataset topology type (dataset_type enum).
 */
void bm_sycl_gpu_compact_spatial_hashing(benchmark::State &state);

/**
 * @brief Evaluates the computation latency of compact spatial hashing construction and radius search on a SYCL-compatible CPU target.
 * 
 * @param[in,out] state Google Benchmark state object containing invocation parameters:
 *                      - state.range(0): Total number of points in the dataset.
 *                      - state.range(1): Dataset topology type (dataset_type enum).
 */
void bm_sycl_cpu_compact_spatial_hashing(benchmark::State &state);

/**
 * @brief Evaluates the computation latency of compact spatial hashing construction and radius search on a SYCL-compatible GPU target utilizing a realistic dataset retrieved from the file system.
 * 
 * @param[in,out] state Google Benchmark state object containing invocation parameters:
 *                      - state.range(0): Total number of points in the dataset.
 */
void bm_sycl_gpu_compact_spatial_hashing_file(benchmark::State &state);

/**
 * @brief Evaluates the computation latency of compact spatial hashing construction and radius search on a SYCL-compatible GPU target utilizing a realistic dataset retrieved from the file system.
 * 
 * @param[in,out] state Google Benchmark state object containing invocation parameters:
 *                      - state.range(0): Total number of points in the dataset.
 */
void bm_sycl_cpu_compact_spatial_hashing_file(benchmark::State &state);

BENCHMARK(bm_sycl_gpu_compact_spatial_hashing)->Apply(default_args);
BENCHMARK(bm_sycl_cpu_compact_spatial_hashing)->Apply(default_args);
BENCHMARK(bm_sycl_gpu_compact_spatial_hashing_file)->Apply(default_args_file);
BENCHMARK(bm_sycl_cpu_compact_spatial_hashing_file)->Apply(default_args_file);

#endif // BM_SYCL_COMPACT_SPATIAL_HASHING_H