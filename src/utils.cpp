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
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "utils.h"

#include <stdexcept>
#include <string>
#include <iostream>
#include <sycl/sycl.hpp>

#include "xgetopt/xgetopt.h"
#include "config.h"

configuration parse_arguments(int argc, char **argv)
{
    if(argc == 1)
    {
        print_help();
        exit(EXIT_SUCCESS);
    }

    configuration result;

    // Accepted parameters
    std::string const short_opts = "c:ld:r:s:q:b:hv";

    // clang-format off
    std::array<xoption, 10> long_options =
        {{
            {"cloud",               xrequired_argument, nullptr, 'c'},
            {"device-list",         xno_argument,       nullptr, 'l'},
            {"device",              xrequired_argument, nullptr, 'd'},
            {"radius",              xrequired_argument, nullptr, 'r'},
            {"cell-size",           xrequired_argument, nullptr, 's'},
            {"search-queries",      xrequired_argument, nullptr, 'q'},
            {"batch-size",          xrequired_argument, nullptr, 'b'},
            {"help",                xno_argument,       nullptr, 'h'},
            {"version",             xno_argument,       nullptr, 'v'},
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
            case 'l': result.print_device_list = true; break;
            case 'd': result.device = xoptarg; break;
            case 'r': result.radius = std::stof(xoptarg); break;
            case 's': result.cell_size = std::stof(xoptarg); break;
            case 'q': result.search_queries =  parse_search_queries(xoptarg); break;
            case 'b': result.batch_size =  std::stoi(xoptarg); break; // FIXME: stoi -> size_t aware
            case 'h': print_help(); exit(EXIT_SUCCESS); break;
            case 'v': print_version(); exit(EXIT_SUCCESS); break;
            default: throw std::runtime_error("could not parse parameters, use --help for usage.");
        }
        // clang-format on
    }

    return result;
}

void print_help()
{
    std::string help =
        R"(
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

    for (const auto& platform : platforms)
    {
        std::cout << "Platform: " << platform.get_info<sycl::info::platform::name>() << std::endl;
        std::cout << "Vendor:   " << platform.get_info<sycl::info::platform::vendor>() << std::endl;
        std::cout << "Version:  " << platform.get_info<sycl::info::platform::version>() << std::endl;

        // Get devices associated with the current platform
        std::vector<sycl::device> devices = platform.get_devices();

        for (const auto& device : devices)
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
