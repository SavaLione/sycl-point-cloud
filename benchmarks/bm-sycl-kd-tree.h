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
 * @file bm-sycl-kd-tree.h
 * @brief Benchmark suite for evaluating the construction and search latencies of a K-D tree structure within the SYCL environment.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef BM_SYCL_KD_TREE_H
#define BM_SYCL_KD_TREE_H

#include "bm-common.h"

/**
 * @brief Evaluates the computation latency of K-D tree construction and radius search on a SYCL-compatible CPU target.
 * 
 * @param[in,out] state The Google Benchmark state object tracking execution iterations and timing counters.
 */
void bm_sycl_cpu_kd_tree(benchmark::State &state);

/**
 * @brief Evaluates the computation latency of the SYCL K-D tree utilizing a realistic dataset retrieved from the file system.
 * 
 * @param[in,out] state The Google Benchmark state object tracking execution iterations and timing counters.
 */
void bm_sycl_cpu_kd_tree_file(benchmark::State &state);

BENCHMARK(bm_sycl_cpu_kd_tree)->Apply(default_args);
BENCHMARK(bm_sycl_cpu_kd_tree_file)->Unit(benchmark::kMillisecond);

#endif // BM_SYCL_KD_TREE_H