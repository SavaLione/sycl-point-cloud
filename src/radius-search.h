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
 * @file radius-search.h
 * @brief Provides a dispatcher-aware execution wrapper for routing radius search queries to the optimal computational device.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef RADIUS_SEARCH_H
#define RADIUS_SEARCH_H

#include "point.h"
#include <vector>

/**
 * @brief Evaluates dataset characteristics and routes the radius search execution to the optimal computational device.
 * 
 * Integrates the adaptive topological transition model to compute an effective parallelization index.
 * The request is subsequently assigned either to the central processing unit or the SYCL-compatible hardware accelerator.
 * 
 * @param[in] point_cloud The input spatial dataset representing the environment.
 * @param[in] host_queries A vector containing the search queries.
 * @param[in] search_radius The radius of the search sphere.
 * @param[in] cell_size The linear dimension of the spatial hashing voxel.
 * @param[in] max_results_per_query The maximum allowed number of matching points to retrieve per individual query.
 * @return A two-dimensional vector where each inner vector contains the retrieved points corresponding to a specific query.
 */
std::vector<std::vector<point_3d>> radius_search_dispatcher(
    std::vector<point_3d> const &point_cloud, std::vector<point_3d> const &host_queries, float search_radius, float cell_size, std::size_t const max_results_per_query);

/**
 * @brief Outputs the result sets of spatial queries to the standard output stream for inspection.
 * 
 * @param[in] r The two-dimensional vector containing the matched points per query.
 */
void print_radius_search_results(std::vector<std::vector<point_3d>> const &r);

#endif // RADIUS_SEARCH_H