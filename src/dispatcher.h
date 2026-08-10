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
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "probe-density.h"

enum class execution_target
{
    cpu_multithreaded,
    sycl_gpu
};

/**
 * @brief Evaluates execution target using the topological crossover model.
 * 
 * Computes a dimensionless asymptotic bound relying purely on spatial
 * complexity and distribution entropy. GPU offloading becomes mathematically
 * viable when the entropy-modulated structural resolution exceeds the 
 * volumetric expansion constant of the topological space.
 * 
 * @param characteristics Output structural metrics from fast pre-analysis.
 * @return execution_target Mathematically optimal hardware execution path.
 */
execution_target evaluate_dispatch_decision(spatial_characteristics const &characteristics);

#endif // DISPATCHER_H