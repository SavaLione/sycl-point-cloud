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
 * @brief Provides input and output routines for reading and writing point cloud datasets in different formats.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef IO_H
#define IO_H

#include <string>
#include <vector>

#include "point.h"

/**
 * @brief Loads a binary point cloud dataset from the specified file into a standard vector.
 * 
 * The binary format assumes a specific header denoting the total number of points, followed by the raw spatial coordinate data.
 * 
 * @param[in] filename The absolute or relative path to the binary dataset file.
 * @return A standard vector populated with the extracted three-dimensional points.
 * @throws std::runtime_error If the file cannot be accessed or the content is structurally invalid.
 */
std::vector<point_3d> load_point_cloud(std::string const &filename);

/**
 * @brief Serializes a point cloud dataset to disk using a custom binary format.
 * 
 * The format: [number of points][point 0][point 1][point 2] ... [point N]
 * 
 * @param[in] filename The intended path for the output binary file.
 * @param[in] cloud The spatial dataset to serialize.
 * @throws std::runtime_error If the specified file path cannot be opened for writing operations.
 */
void save_point_cloud(std::string const &filename, std::vector<point_3d> const &cloud);

/**
 * @brief Serializes a point cloud dataset to disk in the Polygon File Format (Stanford PLY).
 * 
 * The output utilizes the `binary_little_endian 1.0` format encoding.
 * 
 * @param[in] filename The intended path for the output PLY file.
 * @param[in] cloud The spatial dataset to serialize.
 * @throws std::runtime_error If the specified file path cannot be opened for writing operations.
 */
void save_point_cloud_ply(std::string const &filename, std::vector<point_3d> const &cloud);

#endif // IO_H