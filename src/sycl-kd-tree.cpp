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
 * @file sycl-kd-tree.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "sycl-kd-tree.h"
#include <algorithm>
#include <cmath>

// Device-side iterative quickselect partitioning routine to bypass recursive stack limits
inline std::size_t device_quickselect_median(indexed_point_sycl *data, std::size_t left, std::size_t right, std::uint8_t const axis)
{
    std::size_t const k = left + (right - left) / 2;
    right               = right - 1; // Convert to inclusive index

    while(left < right)
    {
        // Select pivot element
        indexed_point_sycl const pivot = data[k];

        // Swap pivot to right boundary
        std::swap(data[k], data[right]);

        std::size_t store_idx = left;
        for(std::size_t i = left; i < right; ++i)
        {
            bool is_less = false;
            if(axis == 0)
                is_less = (data[i].pt.x < pivot.pt.x);
            else if(axis == 1)
                is_less = (data[i].pt.y < pivot.pt.y);
            else
                is_less = (data[i].pt.z < pivot.pt.z);

            if(is_less)
            {
                std::swap(data[i], data[store_idx]);
                store_idx++;
            }
        }
        std::swap(data[store_idx], data[right]);

        if(store_idx == k)
        {
            return k;
        }
        else if(store_idx < k)
        {
            left = store_idx + 1;
        }
        else
        {
            right = store_idx - 1;
        }
    }
    return k;
}

void build_kd_tree_sycl(sycl::queue &q, point_3d const *d_cloud, std::size_t const num_points, sycl_kd_node *d_tree_nodes, std::int32_t *d_root_idx)
{
    if(num_points == 0)
    {
        int32_t const empty_root = -1;
        q.memcpy(d_root_idx, &empty_root, sizeof(std::int32_t)).wait();
        return;
    }

    // Allocate device memory for working index array
    indexed_point_sycl *d_indexed_pts = sycl::malloc_device<indexed_point_sycl>(num_points, q);

    // Initialize indexed structures via parallel execution
    q.submit(
         [&](sycl::handler &cgh)
         {
             cgh.parallel_for(
                 sycl::range<1>(num_points),
                 [=](sycl::id<1> idx)
                 {
                     std::size_t const i = idx[0];
                     d_indexed_pts[i]    = {d_cloud[i], i};
                 });
         })
        .wait();

    // Allocate USM queues for level-synchronous side-to-side building
    range_task *d_current_tasks = sycl::malloc_device<range_task>(num_points, q);
    range_task *d_next_tasks    = sycl::malloc_device<range_task>(num_points, q);

    // Initialize root task
    range_task root_task {0, num_points, 0, 0};
    q.memcpy(d_current_tasks, &root_task, sizeof(range_task)).wait();

    std::int32_t const root_val = 0;
    q.memcpy(d_root_idx, &root_val, sizeof(std::int32_t)).wait();

    std::size_t current_task_count    = 1;
    std::size_t node_allocator_offset = 1; // Root node takes index 0

    // Side-to-side level-synchronous construction
    while(current_task_count > 0)
    {
        // Allocate counter for tracking sub-tasks generated for the subsequent depth level
        std::size_t *d_next_task_count = sycl::malloc_shared<std::size_t>(1, q);
        *d_next_task_count             = 0;

        // Allocate counter for assigning array offsets to child nodes atomically
        std::size_t *d_node_counter = sycl::malloc_shared<std::size_t>(1, q);
        *d_node_counter             = node_allocator_offset;

        q.submit(
             [&](sycl::handler &cgh)
             {
                 cgh.parallel_for(
                     sycl::range<1>(current_task_count),
                     [=](sycl::id<1> idx)
                     {
                         std::size_t const task_idx = idx[0];
                         range_task const task      = d_current_tasks[task_idx];

                         std::size_t const start     = task.start;
                         std::size_t const end       = task.end;
                         std::int32_t const node_idx = task.node_idx;
                         std::uint8_t const depth    = task.depth;
                         std::uint8_t const axis     = depth % 3;

                         // Median index determination
                         std::size_t const mid = device_quickselect_median(d_indexed_pts, start, end, axis);

                         // Populate current node record
                         sycl_kd_node node {};
                         node.point          = d_indexed_pts[mid].pt;
                         node.original_index = d_indexed_pts[mid].original_idx;
                         node.left_child     = -1;
                         node.right_child    = -1;
                         node.axis           = axis;

                         // Evaluate left child sub-range
                         if(start < mid)
                         {
                             sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_node_counter(
                                 *d_node_counter);

                             std::size_t const left_child_idx = atomic_node_counter.fetch_add(1);
                             node.left_child                  = static_cast<std::int32_t>(left_child_idx);

                             sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_task_counter(
                                 *d_next_task_count);

                             std::size_t const t_slot = atomic_task_counter.fetch_add(1);
                             d_next_tasks[t_slot]     = range_task {start, mid, node.left_child, static_cast<std::uint8_t>(depth + 1)};
                         }

                         // Evaluate right child sub-range
                         if(mid + 1 < end)
                         {
                             sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_node_counter(
                                 *d_node_counter);

                             std::size_t const right_child_idx = atomic_node_counter.fetch_add(1);
                             node.right_child                  = static_cast<std::int32_t>(right_child_idx);

                             sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_task_counter(
                                 *d_next_task_count);

                             std::size_t const t_slot = atomic_task_counter.fetch_add(1);
                             d_next_tasks[t_slot]     = range_task {mid + 1, end, node.right_child, static_cast<std::uint8_t>(depth + 1)};
                         }

                         // Commit node to global memory buffer
                         d_tree_nodes[node_idx] = node;
                     });
             })
            .wait();

        // Advance counters and swap task queue allocations
        current_task_count    = *d_next_task_count;
        node_allocator_offset = *d_node_counter;

        std::swap(d_current_tasks, d_next_tasks);

        sycl::free(d_next_task_count, q);
        sycl::free(d_node_counter, q);
    }

    // Memory deallocation
    sycl::free(d_indexed_pts, q);
    sycl::free(d_current_tasks, q);
    sycl::free(d_next_tasks, q);
}

void radius_search_kd_tree_sycl(
    sycl::queue &q,
    point_3d const *d_queries,
    std::size_t const num_queries,
    float const radius,
    sycl_kd_node const *d_tree_nodes,
    std::int32_t const root_idx,
    std::size_t *d_results,
    std::size_t *d_result_counts,
    std::size_t const max_results_per_query)
{
    if(num_queries == 0 || root_idx == -1)
    {
        return;
    }

    float const radius_sq = radius * radius;

    q.submit(
         [&](sycl::handler &cgh)
         {
             cgh.parallel_for(
                 sycl::range<1>(num_queries),
                 [=](sycl::id<1> idx)
                 {
                     std::size_t const q_idx = idx[0];
                     point_3d const query    = d_queries[q_idx];

                     std::size_t match_count           = 0;
                     std::size_t const output_base_idx = q_idx * max_results_per_query;

                     // Fixed-size stack for depth traversal (depth 64 supports up to 2^64 nodes)
                     constexpr std::size_t max_stack_depth = 64;
                     std::int32_t node_stack[max_stack_depth];
                     std::size_t stack_ptr = 0;

                     // Push root node
                     node_stack[stack_ptr++] = root_idx;

                     while(stack_ptr > 0)
                     {
                         // Pop node from explicit stack
                         std::int32_t const current_node_idx = node_stack[--stack_ptr];
                         sycl_kd_node const &node            = d_tree_nodes[current_node_idx];

                         // Distance calculation
                         float const dx      = query.x - node.point.x;
                         float const dy      = query.y - node.point.y;
                         float const dz      = query.z - node.point.z;
                         float const dist_sq = (dx * dx) + (dy * dy) + (dz * dz);

                         if(dist_sq <= radius_sq)
                         {
                             if(match_count < max_results_per_query)
                             {
                                 d_results[output_base_idx + match_count] = node.original_index;
                                 match_count++;
                             }
                         }

                         // Hyperplane distance check
                         float axis_delta = 0.0f;
                         if(node.axis == 0)
                             axis_delta = query.x - node.point.x;
                         else if(node.axis == 1)
                             axis_delta = query.y - node.point.y;
                         else
                             axis_delta = query.z - node.point.z;

                         float const axis_delta_sq = axis_delta * axis_delta;

                         std::int32_t const primary_child   = (axis_delta <= 0.0f) ? node.left_child : node.right_child;
                         std::int32_t const secondary_child = (axis_delta <= 0.0f) ? node.right_child : node.left_child;

                         // Push secondary child if bounding plane intersects search sphere
                         if(axis_delta_sq <= radius_sq && secondary_child != -1)
                         {
                             if(stack_ptr < max_stack_depth)
                             {
                                 node_stack[stack_ptr++] = secondary_child;
                             }
                         }

                         // Push primary child
                         if(primary_child != -1)
                         {
                             if(stack_ptr < max_stack_depth)
                             {
                                 node_stack[stack_ptr++] = primary_child;
                             }
                         }
                     }

                     d_result_counts[q_idx] = match_count;
                 });
         })
        .wait();
}
