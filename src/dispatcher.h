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
 * @file dispatcher.h
 * @brief Implements the adaptive dispatcher based on the topological transition model for optimal hardware routing of spatial queries.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "probe-density.h"
#include <cstddef>

/**
 * @brief Suggested execution target.
 */
enum class execution_target
{
    cpu_multithreaded, ///< SYCL-compatible CPU target.
    sycl_gpu           ///< A SYCL-compatible GPU target.
};

/**
 * @brief Evaluates execution target using the topological crossover model.
 * 
 * Computes a dimensionless asymptotic bound relying purely on spatial
 * complexity and distribution entropy. GPU offloading becomes mathematically
 * viable when the entropy-modulated structural resolution exceeds the 
 * volumetric expansion constant of the topological space.
 * 
 * @param[in] characteristics Output structural metrics from fast pre-analysis.
 * @param[in] compute_units Number of compute units reported by the target GPU device (e.g., 96 for Intel Arc A310 LP) (\f$C\f$).
 * @param[in] work_group_size Maximum work-group size supported by the target GPU device (e.g., 1024 for Intel Arc A310 LP) (\f$W\f$).
 * @param[in] min_waves_to_hide_latency Minimum concurrent waves needed to hide memory latency (typically 2-4) (default: 1) (\f$w\f$).
 * @return execution_target Mathematically optimal hardware execution path.
 * 
 * @code
 * sycl::device gpu_device {sycl::gpu_selector_v};
 * compute_units   = gpu_device.get_info<sycl::info::device::max_compute_units>();
 * work_group_size = gpu_device.get_info<sycl::info::device::max_work_group_size>();
 * @endcode
 */
execution_target evaluate_dispatch_decision(
    spatial_characteristics const &characteristics, std::size_t const compute_units = 96, std::size_t work_group_size = 1024, std::size_t const min_waves_to_hide_latency = 1);

#endif // DISPATCHER_H