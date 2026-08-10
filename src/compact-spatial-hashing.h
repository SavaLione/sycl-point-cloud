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
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef COMPACT_SPATIAL_HASHING_H
#define COMPACT_SPATIAL_HASHING_H

#include "point.h"
#include <vector>

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

// Constructs the compact spatial hash using a counting and prefix-sum methodology.
compact_spatial_hash_table build_compact_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size);

// Radius search on compact spatial hashing
std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, compact_spatial_hash_table const &hash_table);

#include <sycl/sycl.hpp>

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
    std::size_t const max_results_per_query);

// Builds the hash table arrays directly in USM memory
void build_spatial_hash_sycl(
    sycl::queue &q,
    point_3d const *d_cloud,
    std::size_t const num_points,
    float const cell_size,
    std::size_t const table_size,
    std::size_t *d_bucket_offsets,
    std::size_t *d_point_indices);

#endif // COMPACT_SPATIAL_HASHING_H