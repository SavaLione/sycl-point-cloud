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
 * @file bm-common.cpp
 * @brief Provides common utilities, shared configuration, and default argument definitions for the benchmark suites.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "bm-common.h"

void default_args(benchmark::internal::Benchmark *b)
{
    // clang-format off
    b->ArgsProduct({
        {10000, 100000, 1000000, 10000000},
        {0, 1, 2, 3}
    })->Unit(benchmark::kMillisecond);
    // clang-format on
}

void default_args_file(benchmark::internal::Benchmark *b)
{
    b->Args({0}) // 0 means all points in a given point cloud
        ->Args({10000})
        ->Args({100000})
        ->Args({1000000})
        ->Args({10000000})
        ->Unit(benchmark::kMillisecond);
}

void default_args_probe_density_grid_sample(benchmark::internal::Benchmark *b)
{
    // clang-format off
    b->ArgsProduct({
        {10000, 100000, 1000000, 10000000, 20000000, 30000000, 40000000},
        {0, 1, 2, 3},
        {4, 8, 16},
        {256, 1024, 4096}
    })->Unit(benchmark::kMillisecond);
    // clang-format on
}

BENCHMARK_MAIN();
