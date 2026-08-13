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
 * @file compact-spatial-hashing.h
 * @brief Defines structures and algorithms for compact spatial hashing, utilizing prefix sums to optimize memory access patterns in heterogeneous environments.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef COMPACT_SPATIAL_HASHING_H
#define COMPACT_SPATIAL_HASHING_H

#include <vector>
#include <sycl/sycl.hpp>

#include "point.h"

/**
 * @brief Continuous memory representation of a compact spatial hash table.
 */
struct compact_spatial_hash_table
{
    float cell_size;
    std::size_t table_size;

    // Size: table_size + 1. Stores the start index for a given bucket.
    // The number of elements in bucket [i] is computed as: bucket_offsets[i+1] - bucket_offsets[i].
    std::vector<std::size_t> bucket_offsets;

    // Size: N. A densely packed array containing the indices of the points.
    std::vector<std::size_t> point_indices;

    friend std::ostream &operator<<(std::ostream &os, compact_spatial_hash_table const &sht)
    {
        os << "Spatial hash table (table size: " << sht.table_size << ", cell size: " << sht.cell_size << ")\n";

        // Iterate through all buckets up to table_size
        for(std::size_t i = 0; i < sht.table_size; ++i)
        {
            std::size_t const start_idx = sht.bucket_offsets[i];
            std::size_t const end_idx   = sht.bucket_offsets[i + 1];

            // If the start and end indices are identical, the bucket contains zero elements.
            if(start_idx < end_idx)
            {
                os << "  Bucket [" << i << "] -> point indices: ";
                for(std::size_t j = start_idx; j < end_idx; ++j)
                {
                    os << sht.point_indices[j] << " ";
                }
                os << "\n";
            }
        }
        return os;
    }
};

/**
 * @brief Constructs a compact spatial hash table using a counting and prefix-sum methodology.
 * 
 * @param[in] cloud The input spatial dataset representing the point cloud.
 * @param[in] cell_size The linear dimension of a single discretization voxel.
 * @param[in] table_size The total allocated capacity of the hash table (should be a prime number).
 * @return A constructed compact spatial hash table containing continuous memory layouts.
 */
compact_spatial_hash_table build_compact_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size);

/**
 * @brief Executes a radius search query using the CPU implementation of compact spatial hashing.
 * 
 * @param[in] query The three-dimensional spatial coordinates of the search center.
 * @param[in] radius The search radius defining the spherical boundary.
 * @param[in] cloud The input spatial dataset.
 * @param[in] hash_table The pre-constructed compact spatial hash table.
 * @return A standard vector containing the original indices of all points residing within the specified radius.
 */
std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, compact_spatial_hash_table const &hash_table);

/**
 * @brief Executes a batched radius search query in the heterogeneous SYCL environment.
 * 
 * @param[in] q The active SYCL execution queue.
 * @param[in] d_queries Device pointer to the array of query coordinates.
 * @param[in] num_queries The total number of independent search queries.
 * @param[in] radius The search radius defining the spherical boundary.
 * @param[in] d_cloud Device pointer to the input spatial dataset.
 * @param[in] num_points The total number of spatial coordinates in the dataset.
 * @param[in] cell_size The linear dimension of a single discretization voxel.
 * @param[in] table_size The total allocated capacity of the hash table.
 * @param[in] d_bucket_offsets Device pointer to the prefix-sum bucket offsets array.
 * @param[in] d_point_indices Device pointer to the densely packed array of point indices.
 * @param[out] d_results Device pointer to the flattened output array storing search results.
 * @param[out] d_result_counts Device pointer to the output array storing the number of matches per query.
 * @param[in] max_results_per_query The maximum allowed number of matching points to store per query.
 */
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
    std::size_t const max_results_per_query);

/**
 * @brief Constructs the compact spatial hash table directly within USM via SYCL.
 * 
 * @param[in] q The active SYCL execution queue.
 * @param[in] d_cloud Device pointer to the input spatial dataset.
 * @param[in] num_points The total number of spatial coordinates in the dataset.
 * @param[in] cell_size The linear dimension of a single discretization voxel.
 * @param[in] table_size The total allocated capacity of the hash table.
 * @param[out] d_bucket_offsets Device pointer to the array receiving prefix-sum bucket offsets.
 * @param[out] d_point_indices Device pointer to the array receiving sorted point indices.
 */
void build_spatial_hash_sycl(
    sycl::queue &q,
    point_3d const *d_cloud,
    std::size_t const num_points,
    float const cell_size,
    std::size_t const table_size,
    std::size_t *d_bucket_offsets,
    std::size_t *d_point_indices);

#endif // COMPACT_SPATIAL_HASHING_H