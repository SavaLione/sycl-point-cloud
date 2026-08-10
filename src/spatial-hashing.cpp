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
 * @file spatial-hashing.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "spatial-hashing.h"

#include <cmath>

spatial_hash_table build_spatial_hash(std::vector<point_3d> const &cloud, float const cell_size, std::size_t const table_size)
{
    spatial_hash_table hash_table;
    hash_table.cell_size  = cell_size;
    hash_table.table_size = table_size;
    hash_table.buckets.resize(table_size);

    for(std::size_t i = 0; i < cloud.size(); ++i)
    {
        int const voxel_x = static_cast<int>(std::floor(cloud[i].x / cell_size));
        int const voxel_y = static_cast<int>(std::floor(cloud[i].y / cell_size));
        int const voxel_z = static_cast<int>(std::floor(cloud[i].z / cell_size));

        std::size_t const hash_index = compute_hash_index(voxel_x, voxel_y, voxel_z, table_size);

        hash_table.buckets[hash_index].push_back(i);
    }

    return hash_table;
}

std::size_t compute_hash_index(int const x, int const y, int const z, std::size_t const table_size)
{
    long long const p1 = 73856093;
    long long const p2 = 19349663;
    long long const p3 = 83492791;

    long long hash_value = (static_cast<long long>(x) * p1) ^ (static_cast<long long>(y) * p2) ^ (static_cast<long long>(z) * p3);

    // Ensure the hash value is strictly positive before the modulo operation
    if(hash_value < 0)
    {
        hash_value = -hash_value;
    }

    return static_cast<std::size_t>(hash_value) % table_size;
}

bool is_prime(std::size_t const n)
{
    if(n <= 1)
        return false;
    if(n <= 3)
        return true;
    if(n % 2 == 0 || n % 3 == 0)
        return false;

    for(std::size_t i = 5; i * i <= n; i += 6)
    {
        if(n % i == 0 || n % (i + 2) == 0)
        {
            return false;
        }
    }
    return true;
}

std::size_t get_next_prime(std::size_t n)
{
    // Ensure the number is odd to halve the search space
    if(n % 2 == 0)
    {
        n++;
    }
    while(!is_prime(n))
    {
        n += 2;
    }
    return n;
}

std::vector<std::size_t> radius_search(point_3d const &query, float const radius, std::vector<point_3d> const &cloud, spatial_hash_table const &hash_table)
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
                std::size_t const hash_index           = compute_hash_index(ix, iy, iz, hash_table.table_size);
                std::vector<std::size_t> const &bucket = hash_table.buckets[hash_index];

                for(std::size_t const point_idx : bucket)
                {
                    // Skip if this point was already evaluated via a hash collision
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
