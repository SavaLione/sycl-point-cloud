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
 * @file dispatcher.cpp
 * @brief Implements the adaptive dispatcher based on the topological transition model for optimal hardware routing of spatial queries.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "dispatcher.h"

#include <cmath>

execution_target evaluate_dispatch_decision(
    spatial_characteristics const &characteristics, std::size_t const compute_units, std::size_t work_group_size, std::size_t const min_waves_to_hide_latency)
{
    if(characteristics.total_points == 0)
    {
        return execution_target::cpu_multithreaded;
    }

    // Dynamic point saturation threshold
    const double n_saturate = static_cast<double>(compute_units * work_group_size * min_waves_to_hide_latency); // $N_{sat} = C \cdot W \cdot w$

    // Hardware-derived crossover resolution
    constexpr double spatial_dimensions = 3.0;
    const double crossover_threshold    = std::pow(n_saturate, 1.0 / spatial_dimensions); // $T = \sqrt[3]{N_{sat}}$

    // Workload effective resolution (modulated by entropy)
    const double n_points = static_cast<double>(characteristics.total_points);
    const double entropy  = std::max(static_cast<double>(characteristics.normalized_entropy), 0.01);

    const double effective_volume     = n_points * entropy;
    const double effective_resolution = std::pow(effective_volume, 1.0 / spatial_dimensions); // $\sqrt[3]{N \cdot E}$

    // $I = \sqrt[3]{N \cdot E} \ge T$
    if(effective_resolution >= crossover_threshold)
    {
        return execution_target::sycl_gpu;
    }

    return execution_target::cpu_multithreaded;
}