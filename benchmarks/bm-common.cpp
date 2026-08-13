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
    b->Args({10000, 0})
        ->Args({100000, 0})
        ->Args({1000000, 0})
        ->Args({10000000, 0})
        ->Args({10000, 1})
        ->Args({100000, 1})
        ->Args({1000000, 1})
        ->Args({10000000, 1})
        ->Args({10000, 2})
        ->Args({100000, 2})
        ->Args({1000000, 2})
        ->Args({10000000, 2})
        ->Args({10000, 3})
        ->Args({100000, 3})
        ->Args({1000000, 3})
        ->Args({10000000, 3})
        ->Unit(benchmark::kMillisecond);
}

BENCHMARK_MAIN();
