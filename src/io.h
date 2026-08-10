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
 * @file io.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef IO_H
#define IO_H

#include "point.h"
#include <string>
#include <vector>

// Read dataset into a vector
// Each dataset has a header representing the number of points in the dataset
std::vector<point_3d> load_point_cloud(std::string const &filename);

// Save point cloud
void save_point_cloud(std::string const &filename, std::vector<point_3d> const &cloud);

// Usage example:
// // Save dataset
// save_point_cloud_ply("datasets/point_cloud_uniform_10kk_result.ply", point_cloud_uniform_10kk);
// save_point_cloud_ply("datasets/point_cloud_gaussian_clusters_10kk_result.ply", point_cloud_gaussian_clusters_10kk);
// save_point_cloud_ply("datasets/point_cloud_sparse_lidar_10kk_result.ply", point_cloud_sparse_lidar_10kk);
// save_point_cloud_ply("datasets/point_cloud_hyper_clustered_10kk_result.ply", point_cloud_hyper_clustered_10kk);
// Save point cloud in the PLY
void save_point_cloud_ply(std::string const &filename, std::vector<point_3d> const &cloud);

#endif // IO_H