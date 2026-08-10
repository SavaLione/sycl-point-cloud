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
 * @file sycl-kd-tree.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef SYCL_KD_TREE_H
#define SYCL_KD_TREE_H

#include <sycl/sycl.hpp>
#include <cstddef>
#include <cstdint>
#include "point.h"

struct sycl_kd_node
{
    point_3d point;
    std::size_t original_index;
    std::int32_t left_child;  // Offset in flat array (-1 if null)
    std::int32_t right_child; // Offset in flat array (-1 if null)
    std::uint8_t axis;        // 0 = X, 1 = Y, 2 = Z
};

// Internal representation for tracking input spatial coordinates alongside host/device indices
struct indexed_point_sycl
{
    point_3d pt;
    std::size_t original_idx;
};

// Internal structure representing a sub-range partitioning task for breadth-first queueing
struct alignas(16) range_task
{
    std::size_t start;
    std::size_t end;
    std::int32_t node_idx;
    std::uint8_t depth;
};

/**
 * Builds an array-backed, cache-coherent K-D Tree in USM device memory using SYCL.
 * 
 * @param q              Active SYCL queue (instantiated with sycl::cpu_selector_v)
 * @param d_cloud        Device pointer to input point_3d data (USM allocation)
 * @param num_points     Number of elements in the point cloud
 * @param d_tree_nodes   Device output allocation for nodes (must be pre-allocated to size num_points)
 * @param d_root_idx     Output device pointer storing the computed root node offset
 */
void build_kd_tree_sycl(sycl::queue &q, point_3d const *d_cloud, std::size_t const num_points, sycl_kd_node *d_tree_nodes, std::int32_t *d_root_idx);

/**
 * Performs batched radius search queries on a SYCL K-D tree using non-recursive stack traversal.
 *
 * @param q                       Active SYCL queue
 * @param d_queries               Device pointer to query coordinates (USM allocation)
 * @param num_queries             Number of query points
 * @param radius                  Search radius
 * @param d_tree_nodes            Device pointer to constructed flat K-D tree nodes
 * @param root_idx                Root node offset inside d_tree_nodes array
 * @param d_results               Flattened output array (size = num_queries * max_results_per_query)
 * @param d_result_counts         Output array of size num_queries storing hit counts
 * @param max_results_per_query   Maximum hit capacity reserved per query
 */
void radius_search_kd_tree_sycl(
    sycl::queue &q,
    point_3d const *d_queries,
    std::size_t const num_queries,
    float const radius,
    sycl_kd_node const *d_tree_nodes,
    std::int32_t const root_idx,
    std::size_t *d_results,
    std::size_t *d_result_counts,
    std::size_t const max_results_per_query);

#endif // SYCL_KD_TREE_H