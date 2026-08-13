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
 * @brief Defines the standard, non-compact spatial hashing structures and search algorithms for the CPU.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef SPATIAL_HASHING_H
#define SPATIAL_HASHING_H

#include "point.h"
#include <vector>

/**
 * @brief Holds a spatial hash table.
 * 
 * The index of the outer vector is the hash value.
 * The inner vector contains the indices of the points in the cloud.
 */
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

/**
 * @brief Computes a one-dimensional hash index from three-dimensional discrete voxel coordinates.
 * 
 * Utilizes large prime numbers to distribute spatial coordinates uniformly, minimizing harmonic resonance and structural collisions.
 * 
 * @param[in] x The discrete voxel coordinate along the X-axis.
 * @param[in] y The discrete voxel coordinate along the Y-axis.
 * @param[in] z The discrete voxel coordinate along the Z-axis.
 * @param[in] table_size The total allocated capacity of the hash table.
 * @return The computed scalar hash index.
 */
std::size_t compute_hash_index(int const x, int const y, int const z, std::size_t const table_size);

/**
 * @brief Constructs a standard non-compact spatial hash table from the given spatial dataset.
 * 
 * @param[in] cloud The input spatial dataset representing the point cloud.
 * @param[in] cell_size The linear dimension of a single discretization voxel.
 * @param[in] table_size The total allocated capacity of the hash table.
 * @return A constructed spatial hash table containing dynamically allocated buckets.
 */
spatial_hash_table build_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size);

// Performs a radius search using the spatial hash table
// Because multiple distinct voxels may hash to the same bucket (collisions), the strict Euclidean distance check eliminates false positives
std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, spatial_hash_table const &hash_table);

/**
 * @brief Evaluates whether a given unsigned integer is a prime number.
 * 
 * @param[in] n The integer to evaluate.
 * @return True if the integer is a prime number, false otherwise.
 */
bool is_prime(std::size_t const n);

/**
 * @brief Computes the next prime number strictly greater than or equal to the specified boundary.
 * 
 * Applied to guarantee a prime table size, ensuring optimal load factors and reducing collision probability.
 * 
 * @param[in] n The numerical lower bound.
 * @return The nearest prime number greater than or equal to the lower bound.
 */
std::size_t get_next_prime(std::size_t n);

#endif // SPATIAL_HASHING_H