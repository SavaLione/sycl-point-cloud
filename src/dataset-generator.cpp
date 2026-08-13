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
 * @file dataset-generator.cpp
 * @brief Provides deterministic algorithms for generating synthetic point cloud datasets with specific spatial entropy and structural characteristics.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "dataset-generator.h"

std::vector<point_3d> generate_uniform_noise(std::size_t num_points, float extent, std::mt19937 &gen)
{
    std::vector<point_3d> points(num_points);
    std::uniform_real_distribution<float> dist(-extent, extent);

    for(auto &p : points)
    {
        p = {dist(gen), dist(gen), dist(gen)};
    }

    return points;
}

std::vector<point_3d> generate_gaussian_clusters(std::size_t num_points, std::size_t num_clusters, float extent, float std_dev, std::mt19937 &gen)
{
    std::vector<point_3d> points(num_points);
    std::uniform_real_distribution<float> center_dist(-extent, extent);
    std::normal_distribution<float> noise_dist(0.0f, std_dev);

    // Pre-calculate cluster centers
    std::vector<point_3d> centers(num_clusters);
    for(auto &c : centers)
    {
        c = {center_dist(gen), center_dist(gen), center_dist(gen)};
    }

    std::size_t points_per_cluster = num_points / num_clusters;

    for(std::size_t i = 0; i < num_points; ++i)
    {
        // Determine the applicable cluster index
        // The remainder is handled
        std::size_t cluster_idx = std::min(i / points_per_cluster, num_clusters - 1);

        // clang-format off
        points[i] = {
            centers[cluster_idx].x + noise_dist(gen),
            centers[cluster_idx].y + noise_dist(gen),
            centers[cluster_idx].z + noise_dist(gen)
        };
        // clang-format on
    }

    return points;
}

std::vector<point_3d> generate_lidar_simulation(std::size_t num_points, std::size_t num_rings, float max_radius, std::mt19937 &gen)
{
    std::vector<point_3d> points(num_points);

    // Constant for the pi number
    const float pi = std::acos(-1.0f);

    std::uniform_real_distribution<float> azimuth_dist(0.0f, 2.0f * pi);
    std::uniform_real_distribution<float> distance_dist(2.0f, max_radius);

    std::size_t points_per_ring = num_points / num_rings;

    for(std::size_t i = 0; i < num_points; ++i)
    {
        std::size_t ring_idx = std::min(i / points_per_ring, num_rings - 1);

        // Simulating an elevation range from -15.0 degrees upward
        float elevation_deg = -15.0f + (static_cast<float>(ring_idx) * 2.0f);
        float elevation_rad = elevation_deg * (pi / 180.0f);

        float azimuth  = azimuth_dist(gen);
        float distance = distance_dist(gen);

        // Spherical to Cartesian conversion
        // See: https://www.mathworks.com/help/simulink/ref_extras/sphericaltocartesian.html
        // clang-format off
        points[i] = {
            distance * std::cos(elevation_rad) * std::cos(azimuth),
            distance * std::cos(elevation_rad) * std::sin(azimuth),
            distance * std::sin(elevation_rad)
        };
        // clang-format on
    }

    return points;
}

std::vector<point_3d> generate_dataset(std::size_t num_points, dataset_type type)
{
    // A fixed seed guarantees deterministic output across separate execution cycles
    std::mt19937 gen(42);

    switch(type)
    {
        case dataset_type::uniform:
            return generate_uniform_noise(num_points, 100.0f, gen);

        case dataset_type::gaussian_clusters:
            return generate_gaussian_clusters(num_points, 50, 100.0f, 2.0f, gen);

        case dataset_type::sparse_lidar:
            return generate_lidar_simulation(num_points, 16, 150.0f, gen);

        case dataset_type::hyper_clustered:
            return generate_hyper_clustered_noise(num_points, 5, 100.0f, 0.05f, gen); // 5 micro-clusters with extremely high density (std_dev = 0.05)

        default:
            return {};
    }
}

std::vector<point_3d> generate_hyper_clustered_noise(std::size_t num_points, std::size_t num_clusters, float extent, float tight_std_dev, std::mt19937 &gen)
{
    std::vector<point_3d> points(num_points);
    std::uniform_real_distribution<float> center_dist(-extent, extent);
    std::normal_distribution<float> noise_dist(0.0f, tight_std_dev);

    // Pre-calculate sparse cluster centers far apart in space
    std::vector<point_3d> centers(num_clusters);
    for(auto &c : centers)
    {
        c = {center_dist(gen), center_dist(gen), center_dist(gen)};
    }

    std::size_t const points_per_cluster = num_points / num_clusters;

    for(std::size_t i = 0; i < num_points; ++i)
    {
        std::size_t const cluster_idx = std::min(i / points_per_cluster, num_clusters - 1);

        // clang-format off
        points[i] = {
            centers[cluster_idx].x + noise_dist(gen),
            centers[cluster_idx].y + noise_dist(gen),
            centers[cluster_idx].z + noise_dist(gen)
        };
        // clang-format on
    }

    return points;
}