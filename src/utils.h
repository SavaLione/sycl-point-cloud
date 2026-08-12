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
 * @file utils.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

#include "point.h"

/**
 * @struct configuration
 * @brief Holds the application's configuration settings, parsed from command-line arguments.
 */
struct configuration
{
    std::string cloud                    = "";                   ///< Path to the point cloud file
    std::string device                   = "";                   ///< Specific SYCL device
    float radius                         = 2.5f;                 ///< Radius for the radius search
    float cell_size                      = radius / 2.0f;        ///< Compact spatial hashing, cell size
    std::vector<point_3d> search_queries = {{0.0f, 0.0f, 0.0f}}; ///< Radius search - radius queries
    std::size_t batch_size               = 500;                  ///< Batch size per a radius search query
    bool dispatcher_prediction           = false;                ///< Out of a given point cloud, print a prediction
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
 * @brief Parse a string with search queries (used for radius search)
*/
std::vector<point_3d> parse_search_queries(std::string s);

/**
 * @brief Prints a list of all available SYCL devices and platforms to the console
 * 
 * @note This implementation is mostly adapted from an open-source example
 * @see https://github.com/SavaLione/sycl-acpp-example/blob/main/src/sycl-acpp-example.cpp
*/
void print_all_sycl_devices();

/**
 * @brief Print the dispatcher's prediction for a given dataset
 * @param c Configuration (user in order to find the dataset path)
 */
void print_dispatcher_prediction(configuration const &c);

#endif // UTILS_H