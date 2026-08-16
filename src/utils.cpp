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
 * @file utils.cpp
 * @brief Provides auxiliary routines for command-line argument parsing, environment inspection, and application configuration.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "utils.h"

#include <sycl/sycl.hpp>
#include <stdexcept>
#include <string>
#include <iostream>

#include "xgetopt/xgetopt.h"
#include "config.h"
#include "dispatcher.h"
#include "io.h"

configuration parse_arguments(int argc, char **argv)
{
    if(argc == 1)
    {
        print_help();
        exit(EXIT_SUCCESS);
    }

    configuration result;

    // Accepted parameters
    std::string const short_opts = "c:ld:r:s:q:b:phv";

    // clang-format off
    std::array<xoption, 11> long_options =
        {{
            {"cloud",                 xrequired_argument, nullptr, 'c'},
            {"device-list",           xno_argument,       nullptr, 'l'},
            {"device",                xrequired_argument, nullptr, 'd'},
            {"radius",                xrequired_argument, nullptr, 'r'},
            {"cell-size",             xrequired_argument, nullptr, 's'},
            {"search-queries",        xrequired_argument, nullptr, 'q'},
            {"batch-size",            xrequired_argument, nullptr, 'b'},
            {"dispatcher-prediction", xno_argument,       nullptr, 'p'},
            {"help",                  xno_argument,       nullptr, 'h'},
            {"version",               xno_argument,       nullptr, 'v'},
            {0, 0, 0, 0} // Sentinel
        }};
    // clang-format on

    while(true)
    {
        auto const opt = xgetopt_long(argc, argv, short_opts.c_str(), long_options.data(), nullptr);

        if(opt == -1)
            break;

        // clang-format off
        switch(opt)
        {
            case 'c': result.cloud = xoptarg; break;
            case 'l': print_all_sycl_devices(); exit(EXIT_SUCCESS); break;
            case 'd': result.device = xoptarg; break;
            case 'r': result.radius = std::stof(xoptarg); break;
            case 's': result.cell_size = std::stof(xoptarg); break;
            case 'q': result.search_queries =  parse_search_queries(xoptarg); break;
            case 'b': result.batch_size =  std::stoi(xoptarg); break; // FIXME: stoi -> size_t aware
            case 'p': result.dispatcher_prediction = true; break;
            case 'h': print_help(); exit(EXIT_SUCCESS); break;
            case 'v': print_version(); exit(EXIT_SUCCESS); break;
            default: throw std::runtime_error("could not parse parameters, use --help for usage.");
        }
        // clang-format on
    }

    if(result.cloud == "")
        throw std::runtime_error("the point cloud path can't be empty.");

    if(result.batch_size <= 0)
        throw std::runtime_error("the batch size can't be negative or zero.");

    if(result.dispatcher_prediction)
    {
        print_dispatcher_prediction(result);
        exit(EXIT_SUCCESS);
    }

    return result;
}

void print_help()
{
    std::string const help =
        R"(sycl-point-cloud: Accelerated 3D point cloud radius search with dynamic execution target routing.

Usage:
  sycl-point-cloud -c <path> [options...]
  sycl-point-cloud -l
  sycl-point-cloud -h | -v

Options:
  -c, --cloud <path>                  Required (for computation). Path to the input point cloud file.
  -l, --device-list                   Enumerate and display all available SYCL platforms and devices.
  -d, --device <name>                 Explicitly select a target SYCL device by name substring.
  -r, --radius <float>                Search radius for neighborhood queries. (default: 2.5)
  -s, --cell-size <float>             Spatial hashing cell dimension. (default: radius / 2.0)
  -q, --search-queries <coords>       Comma-separated query coordinates (x1,y1,z1,x2,y2,z2,...).
                                      (default: 0.0,0.0,0.0)
  -b, --batch-size <uint>             Number of query points processed per batch. (default: 500)
  -p, --dispatcher-prediction         Perform spatial analysis and output the execution target prediction
                                      without executing the radius search.
  -h, --help                          Display this help text and exit.
  -v, --version                       Display application version and license information and exit.

Examples:
  sycl-point-cloud --device-list
  sycl-point-cloud -c dataset.bin -r 1.5 -s 0.75
  sycl-point-cloud -c dataset.bin -q 10.0,5.0,-2.5,0.0,0.0,0.0 -b 1000
  sycl-point-cloud -c dataset.bin --dispatcher-prediction
)";

    std::cout << help << std::endl;
}

void print_version()
{
    std::string version = "sycl-point-cloud " PROJECT_VERSION "\n"
                          "Copyright (C) 2026 " PROJECT_AUTHOR "\n"
                          "License " PROJECT_LICENSE ": GNU GPL version 3 or later <https://gnu.org/licenses/gpl.html>.\n"
                          "This is free software: you are free to change and redistribute it.\n"
                          "There is NO WARRANTY, to the extent permitted by law.\n\n"
                          "Homepage: " PROJECT_HOMEPAGE_URL "\n";

    std::cout << version << std::endl;
}

std::vector<point_3d> parse_search_queries(std::string s)
{
    std::vector<point_3d> result;

    std::stringstream ss(s);
    std::string item;
    std::vector<float> items;

    while(std::getline(ss, item, ','))
    {
        items.push_back(std::stof(item));
    }

    if(items.size() % 3 != 0)
    {
        throw std::runtime_error("the search-queries must consist of elements that are a multiple of 3.");
    }

    result.reserve(items.size());
    for(std::size_t i = 0; i < items.size(); i += 3)
    {
        result.push_back({items[i], items[i + 1], items[i + 2]});
    }

    return result;
}

// The code taken mostly from: https://github.com/SavaLione/sycl-acpp-example/blob/main/src/sycl-acpp-example.cpp
void print_all_sycl_devices()
{
    // Retrieve all available SYCL platforms
    std::vector<sycl::platform> platforms = sycl::platform::get_platforms();

    if(platforms.empty())
    {
        std::cout << "No SYCL platforms found on this system." << std::endl;
        return;
    }

    for(const auto &platform : platforms)
    {
        std::cout << "Platform: " << platform.get_info<sycl::info::platform::name>() << std::endl;
        std::cout << "Vendor:   " << platform.get_info<sycl::info::platform::vendor>() << std::endl;
        std::cout << "Version:  " << platform.get_info<sycl::info::platform::version>() << std::endl;

        // Get devices associated with the current platform
        std::vector<sycl::device> devices = platform.get_devices();

        for(const auto &device : devices)
        {
            std::cout << "  Device: " << device.get_info<sycl::info::device::name>() << std::endl;

            // Determine the device type
            std::cout << "  Type:   ";
            if(device.is_gpu())
            {
                std::cout << "GPU" << std::endl;
            }
            else if(device.is_cpu())
            {
                std::cout << "CPU" << (device.is_host() ? " (host)" : "") << std::endl;
            }
            else if(device.is_accelerator())
            {
                std::cout << "Accelerator" << std::endl;
            }
            else
            {
                std::cout << "Unknown" << std::endl;
            }

            std::cout << "  Driver: " << device.get_info<sycl::info::device::driver_version>() << std::endl;
        }
    }
}

void print_dispatcher_prediction(configuration const &c)
{
    std::vector<point_3d> point_cloud = load_point_cloud(c.cloud);

    spatial_characteristics characteristics = analyze_point_cloud_fast(point_cloud, 256);
    execution_target decision               = evaluate_dispatch_decision(characteristics);
    std::cout << "Number of points: " << std::to_string(point_cloud.size()) << std::endl;
    std::cout << "Dispatcher effective density: " << std::to_string(characteristics.effective_density) << std::endl;
    std::cout << "Dispatcher normalized entropy: " << std::to_string(characteristics.normalized_entropy) << std::endl;
    std::cout << "Dispatcher decision: " << ((decision == execution_target::sycl_gpu) ? "sycl_gpu" : "cpu_multithreaded") << std::endl;
}
