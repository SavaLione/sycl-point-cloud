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
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "dispatcher.h"

#include <cmath>

execution_target evaluate_dispatch_decision(spatial_characteristics const &characteristics)
{
    if(characteristics.total_points == 0)
    {
        return execution_target::cpu_multithreaded;
    }

    double const n_points = static_cast<double>(characteristics.total_points);
    double const entropy  = static_cast<double>(characteristics.normalized_entropy);

    // Topological constant (D)
    // '3.0' represents the dimensionality of the point cloud (3D spatial data)
    constexpr double spatial_dimensions = 3.0;

    // Structural resolution (R)
    // Represents the linear scaling factor of a volumetric entity encompassing N points.
    double const structural_resolution = std::pow(n_points, 1.0 / spatial_dimensions);

    // Effective parallel workload index
    // Penalizes the parallel execution potential by spatial imbalance (warp divergence proxy).
    double const effective_parallel_index = structural_resolution * entropy;

    // Volumetric expansion threshold
    // The natural exponential boundary where volumetric parallel computation dominates over
    // surface-area data transmission and synchronization penalties.
    double const crossover_threshold = std::exp(spatial_dimensions);

    // Dispatch decision.
    // Target the GPU only when the parallel index outscales the topological overhead.
    if(effective_parallel_index >= crossover_threshold)
    {
        return execution_target::sycl_gpu;
    }

    return execution_target::cpu_multithreaded;
}