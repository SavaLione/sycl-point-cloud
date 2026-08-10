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
 * @file spatial-hashing.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef SPATIAL_HASHING_H
#define SPATIAL_HASHING_H

#include "point.h"
#include <vector>

// The index of the outer vector is the hash value
// The inner vector contains the indices of the points in the cloud
struct spatial_hash_table
{
    float cell_size;
    std::size_t table_size;
    std::vector<std::vector<std::size_t>> buckets;

    friend std::ostream &operator<<(std::ostream &os, spatial_hash_table const &sht)
    {
        os << "Spatial hash table (table size: " << sht.table_size << ", cell size: " << sht.cell_size << ")\n";

        for(std::size_t i = 0; i < sht.buckets.size(); ++i)
        {
            if(!sht.buckets[i].empty())
            {
                os << "  Bucket [" << i << "] -> point indices: ";
                for(std::size_t const point_idx : sht.buckets[i])
                {
                    os << point_idx << " ";
                }
                os << "\n";
            }
        }
        return os;
    }
};

// Computes a 1D hash index from 3D discrete voxel coordinates
// Utilizes large prime numbers to distribute the coordinates
std::size_t compute_hash_index(int const x, int const y, int const z, std::size_t const table_size);

// Constructs the spatial hash table from the point cloud
spatial_hash_table build_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size);

// Performs a radius search using the spatial hash table
// Because multiple distinct voxels may hash to the same bucket (collisions), the strict Euclidean distance check eliminates false positives
std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, spatial_hash_table const &hash_table);

// To achieve an optimal balance between memory consumption and collision probability, the hash table size must scale linearly with the number of points (O(N))
// A standard heuristic in computational geometry is to allocate a table size that is a prime number strictly greater than a multiple of the point cloud size
// A load factor constraint (e.g., table size ~~= 2N) is typically employed
//
// The use of a prime number prevents harmonic resonance between the discrete spatial coordinates and the modulo operator, thereby minimizing structural hash collisions
//
// Evaluates whether a given integer is a prime number.
bool is_prime(std::size_t const n);

// Computes the next prime number greater than or equal to n.
std::size_t get_next_prime(std::size_t n);
// Recommended usage in the main function:
// std::size_t const expected_capacity = point_cloud.size() * 2;
// std::size_t const table_size = get_next_prime(expected_capacity);

#endif // SPATIAL_HASHING_H