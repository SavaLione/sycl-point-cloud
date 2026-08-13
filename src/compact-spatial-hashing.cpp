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
 * @file compact-spatial-hashing.cpp
 * @brief Defines structures and algorithms for compact spatial hashing, utilizing prefix sums to optimize memory access patterns in heterogeneous environments.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "compact-spatial-hashing.h"

#include <cmath>

#include "spatial-hashing.h"

// Constructs the compact spatial hash using a counting and prefix-sum methodology
compact_spatial_hash_table build_compact_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size)
{
    compact_spatial_hash_table hash_table;
    hash_table.cell_size  = cell_size;
    hash_table.table_size = table_size;

    hash_table.bucket_offsets.resize(table_size + 1, 0);
    hash_table.point_indices.resize(cloud.size(), 0);

    // Compute the spatial hash for each point and count bucket occurrences
    std::vector<std::size_t> point_hashes(cloud.size(), 0);
    for(std::size_t i = 0; i < cloud.size(); ++i)
    {
        int const voxel_x = static_cast<int>(std::floor(cloud[i].x / cell_size));
        int const voxel_y = static_cast<int>(std::floor(cloud[i].y / cell_size));
        int const voxel_z = static_cast<int>(std::floor(cloud[i].z / cell_size));

        std::size_t const hash_index = compute_hash_index(voxel_x, voxel_y, voxel_z, table_size);
        point_hashes[i]              = hash_index;

        hash_table.bucket_offsets[hash_index]++;
    }

    // Compute the exclusive prefix sum (scan) to determine memory offsets
    std::size_t cumulative_sum = 0;
    for(std::size_t i = 0; i < table_size; ++i)
    {
        std::size_t const current_count = hash_table.bucket_offsets[i];
        hash_table.bucket_offsets[i]    = cumulative_sum;
        cumulative_sum += current_count;
    }

    // Set the final boundary element
    hash_table.bucket_offsets[table_size] = cumulative_sum;

    // Populate the dense point index array
    // A volatile copy of the offsets is utilized to track insertion positions
    std::vector<std::size_t> insertion_offsets = hash_table.bucket_offsets;
    for(std::size_t i = 0; i < cloud.size(); ++i)
    {
        std::size_t const hash_index      = point_hashes[i];
        std::size_t const insertion_index = insertion_offsets[hash_index];

        hash_table.point_indices[insertion_index] = i;

        // Increment the tracking offset, so the next element in this bucket is placed adjacently
        insertion_offsets[hash_index]++;
    }

    return hash_table;
}

std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, compact_spatial_hash_table const &hash_table)
{
    std::vector<std::size_t> results;

    // Utilize a flat boolean array to prevent processing the same point multiple times
    std::vector<bool> visited(cloud.size(), false);

    int const min_x = static_cast<int>(std::floor((query.x - radius) / hash_table.cell_size));
    int const max_x = static_cast<int>(std::floor((query.x + radius) / hash_table.cell_size));
    int const min_y = static_cast<int>(std::floor((query.y - radius) / hash_table.cell_size));
    int const max_y = static_cast<int>(std::floor((query.y + radius) / hash_table.cell_size));
    int const min_z = static_cast<int>(std::floor((query.z - radius) / hash_table.cell_size));
    int const max_z = static_cast<int>(std::floor((query.z + radius) / hash_table.cell_size));

    // Compute squared radius once to bypass std::sqrt overhead and precision loss
    float const radius_sq = radius * radius;

    for(int ix = min_x; ix <= max_x; ++ix)
    {
        for(int iy = min_y; iy <= max_y; ++iy)
        {
            for(int iz = min_z; iz <= max_z; ++iz)
            {
                std::size_t const hash_index = compute_hash_index(ix, iy, iz, hash_table.table_size);

                // Extract the memory boundaries for the active bucket
                std::size_t const start_idx = hash_table.bucket_offsets[hash_index];
                std::size_t const end_idx   = hash_table.bucket_offsets[hash_index + 1];

                for(std::size_t i = start_idx; i < end_idx; ++i)
                {
                    std::size_t const point_idx = hash_table.point_indices[i];

                    // Bypass if this point was already evaluated during an overlapping hash collision
                    if(visited[point_idx])
                    {
                        continue;
                    }

                    visited[point_idx] = true;

                    point_3d const &target = cloud[point_idx];

                    float const dx = query.x - target.x;
                    float const dy = query.y - target.y;
                    float const dz = query.z - target.z;

                    float const distance_sq = (dx * dx) + (dy * dy) + (dz * dz);

                    if(distance_sq <= radius_sq)
                    {
                        results.push_back(point_idx);
                    }
                }
            }
        }
    }

    return results;
}

// Reusable device-side hash function
inline std::size_t sycl_compute_hash_index(int const x, int const y, int const z, std::size_t const table_size)
{
    long long const p1 = 73856093;
    long long const p2 = 19349663;
    long long const p3 = 83492791;

    long long hash_value = (static_cast<long long>(x) * p1) ^ (static_cast<long long>(y) * p2) ^ (static_cast<long long>(z) * p3);

    if(hash_value < 0)
        hash_value = -hash_value;
    return static_cast<std::size_t>(hash_value) % table_size;
}

// Builds the hash table arrays directly in USM memory
void build_spatial_hash_sycl(
    sycl::queue &q,
    point_3d const *d_cloud,
    std::size_t const num_points,
    float const cell_size,
    std::size_t const table_size,
    std::size_t *d_bucket_offsets,
    std::size_t *d_point_indices)
{
    // Initialize bucket offsets to zero
    q.fill(d_bucket_offsets, std::size_t {0}, table_size + 1).wait();

    std::size_t *d_point_hashes = sycl::malloc_device<std::size_t>(num_points, q);

    // Histogram counting
    q.submit(
         [&](sycl::handler &cgh)
         {
             cgh.parallel_for(
                 sycl::range<1>(num_points),
                 [=](sycl::id<1> idx)
                 {
                     point_3d const p  = d_cloud[idx];
                     int const voxel_x = static_cast<int>(sycl::floor(p.x / cell_size));
                     int const voxel_y = static_cast<int>(sycl::floor(p.y / cell_size));
                     int const voxel_z = static_cast<int>(sycl::floor(p.z / cell_size));

                     std::size_t const hash_idx = sycl_compute_hash_index(voxel_x, voxel_y, voxel_z, table_size);
                     d_point_hashes[idx]        = hash_idx;

                     // Atomic increment for thread-safe counting
                     sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_bucket(
                         d_bucket_offsets[hash_idx]);
                     atomic_bucket.fetch_add(1);
                 });
         })
        .wait();

    // Exclusive prefix sum
    // TODO: It's probably better to use 'std::exclusive_scan' or 'acpp::algorithms::exclusive_scan' (for device-side execution)
    //      See: https://en.cppreference.com/cpp/algorithm/exclusive_scan
    //      See: https://github.com/AdaptiveCpp/AdaptiveCpp/blob/develop/doc/algorithms.md
    std::size_t *h_bucket_offsets = sycl::malloc_host<std::size_t>(table_size + 1, q);
    q.memcpy(h_bucket_offsets, d_bucket_offsets, (table_size + 1) * sizeof(std::size_t)).wait();

    std::size_t cumulative_sum = 0;
    for(std::size_t i = 0; i < table_size; ++i)
    {
        std::size_t const current_count = h_bucket_offsets[i];
        h_bucket_offsets[i]             = cumulative_sum;
        cumulative_sum += current_count;
    }
    h_bucket_offsets[table_size] = cumulative_sum;

    q.memcpy(d_bucket_offsets, h_bucket_offsets, (table_size + 1) * sizeof(std::size_t)).wait();
    sycl::free(h_bucket_offsets, q);

    // Scatter
    std::size_t *d_insertion_offsets = sycl::malloc_device<std::size_t>(table_size + 1, q);
    q.memcpy(d_insertion_offsets, d_bucket_offsets, (table_size + 1) * sizeof(std::size_t)).wait();

    q.submit(
         [&](sycl::handler &cgh)
         {
             cgh.parallel_for(
                 sycl::range<1>(num_points),
                 [=](sycl::id<1> idx)
                 {
                     std::size_t const hash_idx = d_point_hashes[idx];

                     sycl::atomic_ref<std::size_t, sycl::memory_order::relaxed, sycl::memory_scope::device, sycl::access::address_space::global_space> atomic_offset(
                         d_insertion_offsets[hash_idx]);

                     // Atomically retrieve current insertion index and increment
                     std::size_t const insert_idx = atomic_offset.fetch_add(1);
                     d_point_indices[insert_idx]  = idx;
                 });
         })
        .wait();

    sycl::free(d_insertion_offsets, q);
    sycl::free(d_point_hashes, q);
}

// Batched radius search in SYCL
void radius_search_sycl(
    sycl::queue &q,
    point_3d const *d_queries,
    std::size_t const num_queries,
    float const radius,
    point_3d const *d_cloud,
    std::size_t const num_points,
    float const cell_size,
    std::size_t const table_size,
    std::size_t const *d_bucket_offsets,
    std::size_t const *d_point_indices,
    std::size_t *d_results,       // Flattened output array: size = num_queries * max_results_per_query
    std::size_t *d_result_counts, // Array of size num_queries to store hit counts
    std::size_t const max_results_per_query)
{
    float const radius_sq = radius * radius;

    q.submit(
         [&](sycl::handler &cgh)
         {
             cgh.parallel_for(
                 sycl::range<1>(num_queries),
                 [=](sycl::id<1> q_idx)
                 {
                     std::size_t const query_id = q_idx[0];
                     point_3d const query       = d_queries[query_id];

                     std::size_t match_count           = 0;
                     std::size_t const output_base_idx = query_id * max_results_per_query;

                     int const min_x = static_cast<int>(sycl::floor((query.x - radius) / cell_size));
                     int const max_x = static_cast<int>(sycl::floor((query.x + radius) / cell_size));
                     int const min_y = static_cast<int>(sycl::floor((query.y - radius) / cell_size));
                     int const max_y = static_cast<int>(sycl::floor((query.y + radius) / cell_size));
                     int const min_z = static_cast<int>(sycl::floor((query.z - radius) / cell_size));
                     int const max_z = static_cast<int>(sycl::floor((query.z + radius) / cell_size));

                     for(int ix = min_x; ix <= max_x; ++ix)
                     {
                         for(int iy = min_y; iy <= max_y; ++iy)
                         {
                             for(int iz = min_z; iz <= max_z; ++iz)
                             {
                                 std::size_t const hash_idx  = sycl_compute_hash_index(ix, iy, iz, table_size);
                                 std::size_t const start_idx = d_bucket_offsets[hash_idx];
                                 std::size_t const end_idx   = d_bucket_offsets[hash_idx + 1];

                                 for(std::size_t i = start_idx; i < end_idx; ++i)
                                 {
                                     std::size_t const point_idx = d_point_indices[i];
                                     point_3d const target       = d_cloud[point_idx];

                                     // Resolve hash collisions without memory allocation
                                     int const t_vx = static_cast<int>(sycl::floor(target.x / cell_size));
                                     int const t_vy = static_cast<int>(sycl::floor(target.y / cell_size));
                                     int const t_vz = static_cast<int>(sycl::floor(target.z / cell_size));
                                     if(t_vx != ix || t_vy != iy || t_vz != iz)
                                         continue;

                                     float const dx      = query.x - target.x;
                                     float const dy      = query.y - target.y;
                                     float const dz      = query.z - target.z;
                                     float const dist_sq = (dx * dx) + (dy * dy) + (dz * dz);

                                     if(dist_sq <= radius_sq)
                                     {
                                         if(match_count < max_results_per_query)
                                         {
                                             d_results[output_base_idx + match_count] = point_idx;
                                             match_count++;
                                         }
                                     }
                                 }
                             }
                         }
                     }
                     d_result_counts[query_id] = match_count;
                 });
         })
        .wait();
}
