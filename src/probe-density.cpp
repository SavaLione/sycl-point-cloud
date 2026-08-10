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
 * @file probe-density.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "probe-density.h"

#include <cmath>

spatial_characteristics analyze_point_cloud_fast(std::vector<point_3d> const &cloud, std::size_t sample_size)
{
    std::size_t const n = cloud.size();
    if(n == 0)
    {
        return {0, 0.0f, 0.0f};
    }

    std::size_t const m      = std::min(sample_size, n);
    std::size_t const stride = std::max<std::size_t>(1, n / m);

    // Calculate mean and standard deviation over sample M
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for(std::size_t i = 0; i < m; ++i)
    {
        point_3d const &p = cloud[i * stride];
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
    }

    double const m_inv = 1.0 / static_cast<double>(m);
    float const mean_x = static_cast<float>(sum_x * m_inv);
    float const mean_y = static_cast<float>(sum_y * m_inv);
    float const mean_z = static_cast<float>(sum_z * m_inv);

    double var_x = 0.0, var_y = 0.0, var_z = 0.0;
    for(std::size_t i = 0; i < m; ++i)
    {
        point_3d const &p = cloud[i * stride];
        float const dx    = p.x - mean_x;
        float const dy    = p.y - mean_y;
        float const dz    = p.z - mean_z;
        var_x += dx * dx;
        var_y += dy * dy;
        var_z += dz * dz;
    }

    float const std_x = std::sqrt(static_cast<float>(var_x * m_inv)) + 1e-5f;
    float const std_y = std::sqrt(static_cast<float>(var_y * m_inv)) + 1e-5f;
    float const std_z = std::sqrt(static_cast<float>(var_z * m_inv)) + 1e-5f;

    // Effective volume defined by 4 standard deviations (covers ~95% of distribution)
    float const effective_volume  = 64.0f * std_x * std_y * std_z;
    float const effective_density = static_cast<float>(n) / effective_volume;

    // Coarse voxel entropy over an 8x8x8 grid (512 total cells)
    constexpr std::size_t grid_dim    = 8;
    constexpr std::size_t total_cells = grid_dim * grid_dim * grid_dim;
    std::array<std::size_t, total_cells> voxel_counts {};

    float const min_x   = mean_x - 2.0f * std_x;
    float const min_y   = mean_y - 2.0f * std_y;
    float const min_z   = mean_z - 2.0f * std_z;
    float const scale_x = static_cast<float>(grid_dim) / (4.0f * std_x);
    float const scale_y = static_cast<float>(grid_dim) / (4.0f * std_y);
    float const scale_z = static_cast<float>(grid_dim) / (4.0f * std_z);

    for(std::size_t i = 0; i < m; ++i)
    {
        point_3d const &p = cloud[i * stride];

        int gx = static_cast<int>((p.x - min_x) * scale_x);
        int gy = static_cast<int>((p.y - min_y) * scale_y);
        int gz = static_cast<int>((p.z - min_z) * scale_z);

        gx = std::clamp(gx, 0, static_cast<int>(grid_dim - 1));
        gy = std::clamp(gy, 0, static_cast<int>(grid_dim - 1));
        gz = std::clamp(gz, 0, static_cast<int>(grid_dim - 1));

        std::size_t const cell_idx = gx + grid_dim * (gy + grid_dim * gz);
        voxel_counts[cell_idx]++;
    }

    // Compute Shannon Entropy H
    double entropy = 0.0;
    for(std::size_t count : voxel_counts)
    {
        if(count > 0)
        {
            double const p = static_cast<double>(count) * m_inv;
            entropy -= p * std::log2(p);
        }
    }

    // Normalize entropy to [0.0, 1.0] range relative to H_max = log2(512) = 9.0
    constexpr double max_entropy   = 9.0;
    float const normalized_entropy = static_cast<float>(entropy / max_entropy);

    return {n, effective_density, normalized_entropy};
}