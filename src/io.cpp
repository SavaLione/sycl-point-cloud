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
 * @file io.h
 * @brief Provides input and output routines for reading and writing point cloud datasets in different formats.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "io.h"

#include <iostream>
#include <fstream>

std::vector<point_3d> load_point_cloud(std::string const &filename)
{
    std::ifstream file(filename, std::ios::binary);
    if(!file.is_open())
    {
        throw std::runtime_error("Failed to open dataset file: " + filename);
    }

    // Read the dataset header (the number of points in the dataset)
    uint64_t num_points = 0;
    file.read(reinterpret_cast<char *>(&num_points), sizeof(uint64_t));

    if(!file || num_points == 0)
    {
        throw std::runtime_error("Invalid or empty point cloud file: " + filename);
    }

    std::vector<point_3d> cloud(num_points);
    file.read(reinterpret_cast<char *>(cloud.data()), num_points * sizeof(point_3d));

    if(!file)
    {
        throw std::runtime_error("Failed to read point data from: " + filename);
    }

    file.close();

    return cloud;
}

void save_point_cloud(std::string const &filename, std::vector<point_3d> const &cloud)
{
    std::ofstream file(filename, std::ios::binary);
    if(!file.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    uint64_t const num_points = cloud.size();
    file.write(reinterpret_cast<char const *>(&num_points), sizeof(uint64_t));
    file.write(reinterpret_cast<char const *>(cloud.data()), num_points * sizeof(point_3d));

    file.close();
}

void save_point_cloud_ply(std::string const &filename, std::vector<point_3d> const &cloud)
{
    std::ofstream file(filename, std::ios::binary);
    if(!file.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // PLY header
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    file.write(reinterpret_cast<char const *>(cloud.data()), cloud.size() * sizeof(point_3d));

    file.close();
}
