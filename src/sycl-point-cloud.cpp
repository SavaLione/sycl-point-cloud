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
 * @file sycl-point-cloud.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <sycl/sycl.hpp>

#include "point.h"
#include "io.h"
#include "utils.h"
#include "dispatcher.h"
#include "radius-search.h"

int main(int argc, char **argv)
{
    configuration config;

    // Parse config
    try
    {
        config = parse_arguments(argc, argv);

        if(config.print_device_list)
        {
            print_all_sycl_devices();
            return EXIT_SUCCESS;
        }

        if(config.cloud == "")
            throw std::runtime_error("the point cloud path can't be empty.");

        if(config.batch_size <= 0)
            throw std::runtime_error("the batch size can't be negative or zero.");
    }
    catch(std::exception const &e)
    {
        std::stringstream ss;
        ss << "sycl-point-cloud: " << e.what() << std::endl;
        std::cerr << ss.str();
        return EXIT_FAILURE;
    }

    // Load point cloud
    std::vector<point_3d> point_cloud;
    try
    {
        point_cloud = load_point_cloud(config.cloud);

        std::cout << "Point cloud at " + config.cloud + " was successfully loaded" << std::endl;
        std::cout << "It has " << point_cloud.size() << " points" << std::endl;
    }
    catch(std::exception const &e)
    {
        std::stringstream ss;
        ss << "sycl-point-cloud: " << e.what() << std::endl;
        std::cerr << ss.str();
        return EXIT_FAILURE;
    }

    // Dispatcher decision
    spatial_characteristics characteristics = analyze_point_cloud_fast(point_cloud, 256);
    execution_target decision = evaluate_dispatch_decision(characteristics);
    std::cout << "Dispatcher effective density: " << std::to_string(characteristics.effective_density) << std::endl;
    std::cout << "Dispatcher normalized entropy: " << std::to_string(characteristics.normalized_entropy) << std::endl;
    std::cout << "Dispatcher decision: " << ((decision == execution_target::sycl_gpu) ? "sycl_gpu" : "cpu_multithreaded") << std::endl;

    // Perform radius search (dispatcher aware)
    try
    {
        auto r = radius_search_dispatcher(point_cloud, config.search_queries, config.radius, config.cell_size, config.batch_size);
        print_radius_search_results(r);
    }
    catch (sycl::exception const& e)
    {
        std::cerr << "sycl-point-cloud: SYCL exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(std::exception const &e)
    {
        std::stringstream ss;
        ss << "sycl-point-cloud: " << e.what() << std::endl;
        std::cerr << ss.str();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
