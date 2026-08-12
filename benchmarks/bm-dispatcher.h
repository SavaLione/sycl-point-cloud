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
 * @file bm-dispatcher.h
 * @brief The SYCL dispatcher benchmark
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef BM_DISPATCHER_H
#define BM_DISPATCHER_H

#include "bm-common.h"

void bm_adaptive_dispatcher(benchmark::State &state);
void bm_adaptive_dispatcher_file(benchmark::State &state);

BENCHMARK(bm_adaptive_dispatcher)->Apply(default_args);
BENCHMARK(bm_adaptive_dispatcher_file)->Unit(benchmark::kMillisecond);

#endif // BM_DISPATCHER_H