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
 * @file sycl-point-cloud-convert.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef SYCL_POINT_CLOUD_CONVERT_H
#define SYCL_POINT_CLOUD_CONVERT_H

#include <string>
#include <vector>

#include "point.h"


enum file_type
{
    BINARY            = 0,
    PLY               = 1,
    XYZ_INTENSITY_RGB = 2,
    UNKNOWN           = 3,
};

/**
 * @brief Converts a string to a file type
 * @param s String to convert
 * @return The file type (UNKNOWN if the file type is not known)
 */
file_type string_to_file_type(std::string s);

/**
 * @brief Converts a file type to a string
 * @param t File type to convert
 * @return The string
 */
std::string file_type_to_string(file_type t);

/**
 * @struct configuration
 * @brief Holds the application's configuration settings, parsed from command-line arguments.
 */
struct configuration
{
    std::string file_source      = "";                 ///< Path to the file that has to be converted
    std::string file_destination = "";                 ///< Path to the conversion result
    file_type source_type        = file_type::UNKNOWN; ///< Source file type
    file_type destination_type   = file_type::UNKNOWN; ///< Destination file type
};

/**
 * @brief Parses command-line arguments and populates a configuration struct
 * @param argc Argument count from `main`
 * @param argv Argument vector from `main`
 * @return A populated `configuration` struct
 * @throws std::runtime_error on parsing failure or invalid arguments
 */
configuration parse_arguments(int argc, char **argv);

/**
 * @brief Prints help information that is invoked by `-h` or `--help`
 */
void print_help();

/**
 * @brief Prints version information that is invoked by `-v` or `--version`
 */
void print_version();

/**
 * @brief Reads a point cloud from an ASCII text file formatted as 'xyz_intensity_rgb'.
 * @param[in] filename Path to the input file.
 * @return std::vector<point_3d> Vector that contains the extracted 3D spatial coordinates.
 * @throws std::runtime_error On the file that cannot be opened, is empty, or contains invalid formatted data.
 */
std::vector<point_3d> load_point_cloud_xyz_intensity_rgb(std::string const &filename);

void convert(std::string source, std::string destination, file_type source_type, file_type destination_type);

#endif // SYCL_POINT_CLOUD_CONVERT_H