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
 * @file dataset-generator.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef DATASET_GENERATOR_H
#define DATASET_GENERATOR_H

#include "point.h"
#include <vector>
#include <random>

// Synthetic dataset generator
enum class dataset_type
{
    uniform,
    gaussian_clusters,
    sparse_lidar,
    hyper_clustered ///< (very low entropy)
};

/**
 * @brief Generates uniformly distributed points within a cubic bounding volume.
 * 
 * @param num_points The total number of points to generate.
 * @param extent The symmetrical boundary of the cube [-extent, extent].
 * @param gen A reference to a seeded Mersenne Twister pseudo-random number generator.
 * @return std::vector<point_3d> An array containing the generated point cloud.
 */
std::vector<point_3d> generate_uniform_noise(std::size_t num_points, float extent, std::mt19937 &gen);

/**
 * @brief Generates highly dense, isolated spatial clusters utilizing normal distributions.
 * 
 * First, random cluster centers are generated uniformly within the specified extent.
 * Next, points are distributed around these centers applying standard deviation.
 * 
 * @param num_points The total number of points to generate.
 * @param num_clusters The total number of isolated clusters.
 * @param extent  The boundary parameter for placing cluster centers.
 * @param std_dev The standard deviation defining the spatial density of each cluster.
 * @param gen A reference to a seeded Mersenne Twister pseudo-random number generator.
 * @return std::vector<point_3d> An array containing the clustered point cloud.
 */
std::vector<point_3d> generate_gaussian_clusters(std::size_t num_points, std::size_t num_clusters, float extent, float std_dev, std::mt19937 &gen);

/**
 * @brief Simulates a sparse street LiDAR scan characterized by high variance.
 * 
 * The algorithm models laser emission rings by varying elevation angles and 
 * generating points in spherical coordinates before converting them to Cartesian coordinates.
 * 
 * @param num_points The total number of points to generate.
 * @param num_rings The number of discrete vertical elevation channels (e.g., 16, 32, 64).
 * @param max_radius The maximum operational range of the simulated LiDAR sensor.
 * @param gen A reference to a seeded Mersenne Twister pseudo-random number generator.
 * @return std::vector<point_3d> An array containing the simulated LiDAR point cloud.
 */
std::vector<point_3d> generate_lidar_simulation(std::size_t num_points, std::size_t num_rings, float max_radius, std::mt19937 &gen);


// Usage example:
// std::vector<point_3d> point_cloud_uniform_10kk = generate_dataset(1000*1000*10, dataset_type::uniform);
// std::vector<point_3d> point_cloud_gaussian_clusters_10kk = generate_dataset(1000*1000*10, dataset_type::gaussian_clusters);
// std::vector<point_3d> point_cloud_sparse_lidar_10kk = generate_dataset(1000*1000*10, dataset_type::sparse_lidar);
// std::vector<point_3d> point_cloud_hyper_clustered_10kk = generate_dataset(1000*1000*10, dataset_type::hyper_clustered);
/**
 * @brief Unified interface for generating synthetic point cloud datasets.
 * 
 * Instantiates a deterministic random number generator to ensure experimental reproducibility.
 * 
 * @param num_points The total number of elements required in the dataset.
 * @param type The desired spatial distribution classification.
 * @return std::vector<point_3d> An array containing the generated point cloud.
 */
std::vector<point_3d> generate_dataset(std::size_t num_points, dataset_type type);

/**
 * @brief Generates hyper-clustered point clouds targeting low normalized entropy.
 * 
 * Concentrates points into tightly bound spatial micro-clusters, designed
 * to stress test spatial hash collisions and GPU thread divergence.
 * 
 * @param num_points Total number of points to generate.
 * @param num_clusters Total number of micro-cluster centers.
 * @param extent Boundary parameter for placing cluster centers.
 * @param tight_std_dev Tight standard deviation (e.g., 0.05f).
 * @param gen Seeded Mersenne Twister engine.
 * @return std::vector<point_3d> Dense micro-clustered dataset.
 */
std::vector<point_3d> generate_hyper_clustered_noise(std::size_t num_points, std::size_t num_clusters, float extent, float tight_std_dev, std::mt19937 &gen);

#endif // DATASET_GENERATOR_H