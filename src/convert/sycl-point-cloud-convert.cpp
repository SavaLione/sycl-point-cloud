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
 * @brief Provides a standalone command-line utility for converting point cloud datasets between supported file formats.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "sycl-point-cloud-convert.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>

#include "config.h"
#include "io.h"
#include "xgetopt/xgetopt.h"

int main(int argc, char **argv)
{
    configuration config;

    try
    {
        config = parse_arguments(argc, argv);

        convert(config.file_source, config.file_destination, config.source_type, config.destination_type);
    }
    catch(std::exception const &e)
    {
        std::stringstream ss;
        ss << "sycl-point-cloud-convert: " << e.what() << std::endl;
        std::cerr << ss.str();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

configuration parse_arguments(int argc, char **argv)
{
    if(argc == 1)
    {
        print_help();
        exit(EXIT_SUCCESS);
    }

    configuration result;

    // Accepted parameters
    std::string const short_opts = "s:d:i:o:hv";

    // clang-format off
    std::array<xoption, 7> long_options =
        {{
            {"file-source",         xrequired_argument, nullptr, 's'},
            {"file-destination",    xrequired_argument, nullptr, 'd'},
            {"source-type",         xrequired_argument, nullptr, 'i'},
            {"destination-type",    xrequired_argument, nullptr, 'o'},
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
            case 's': result.file_source = xoptarg; break;
            case 'd': result.file_destination = xoptarg; break;
            case 'i': result.source_type = string_to_file_type(xoptarg); break;
            case 'o': result.destination_type = string_to_file_type(xoptarg); break;
            case 'h': print_help(); exit(EXIT_SUCCESS); break;
            case 'v': print_version(); exit(EXIT_SUCCESS); break;
            default: throw std::runtime_error("could not parse parameters, use --help for usage.");
        }
        // clang-format on
    }

    if(result.file_source == "")
        throw std::runtime_error("the file source path can't be empty.");

    if(result.file_destination == "")
        throw std::runtime_error("the file destination path can't be empty.");

    if(result.source_type == file_type::UNKNOWN)
        throw std::runtime_error("the source file type in unknown.");

    if(result.destination_type == file_type::UNKNOWN)
        throw std::runtime_error("the destination file type in unknown.");

    return result;
}

void print_help()
{
    std::string help =
        R"(sycl-point-cloud-convert: A command-line utility for point cloud file format conversion.

usage: sycl-point-cloud-convert -s <path> -d <path> -i <type> -o <type> [options...]

Options:
  -s, --file-source <path>        Required. Path to the input point cloud file.
  -d, --file-destination <path>   Required. Path to the output point cloud file.
  -i, --source-type <type>        Required. Format of the source file.
  -o, --destination-type <type>   Required. Format of the destination file.
  -h, --help                      Print this help message and exit.
  -v, --version                   Print version information and exit.

Supported File Types:
  binary                          Custom binary point cloud format.
  ply                             Polygon file format (Stanford PLY).
  xyz_intensity_rgb               ASCII text format containing X, Y, Z coordinates, intensity, and RGB color channels.

Examples:
  sycl-point-cloud-convert -s input.ply -d output.bin -i ply -o binary
  sycl-point-cloud-convert --file-source in.txt --file-destination out.ply --source-type xyz_intensity_rgb --destination-type ply
)";

    std::cout << help << std::endl;
}

void print_version()
{
    std::string version = "sycl-point-cloud-convert " PROJECT_VERSION "\n"
                          "Copyright (C) 2026 " PROJECT_AUTHOR "\n"
                          "License " PROJECT_LICENSE ": GNU GPL version 3 or later <https://gnu.org/licenses/gpl.html>.\n"
                          "This is free software: you are free to change and redistribute it.\n"
                          "There is NO WARRANTY, to the extent permitted by law.\n\n"
                          "Homepage: " PROJECT_HOMEPAGE_URL "\n";

    std::cout << version << std::endl;
}

file_type string_to_file_type(std::string s)
{
    file_type result = file_type::UNKNOWN;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if(s == "binary")
        result = file_type::BINARY;

    if(s == "ply")
        result = file_type::PLY;

    if(s == "xyz_intensity_rgb")
        result = file_type::XYZ_INTENSITY_RGB;

    return result;
}

std::string file_type_to_string(file_type t)
{
    if(t == file_type::BINARY)
        return "BINARY";

    if(t == file_type::PLY)
        return "PLY";

    if(t == file_type::XYZ_INTENSITY_RGB)
        return "XYZ_INTENSITY_RGB";

    return "UNKNOWN";
}

std::vector<point_3d> load_point_cloud_xyz_intensity_rgb(std::string const &filename)
{
    std::ifstream file(filename);
    if(!file.is_open())
    {
        throw std::runtime_error("Failed to open dataset file: " + filename);
    }

    std::vector<point_3d> cloud;

    point_3d point;
    double intensity;
    int r = 0, g = 0, b = 0;

    while(file >> point.x >> point.y >> point.z >> intensity >> r >> g >> b)
    {
        cloud.push_back(point);
    }

    if(!file.eof() && file.fail())
    {
        throw std::runtime_error("Parsing failure encountered while reading file: " + filename);
    }

    if(cloud.empty())
    {
        throw std::runtime_error("Invalid or empty point cloud file: " + filename);
    }

    file.close();

    return cloud;
}

void convert(std::string source, std::string destination, file_type source_type, file_type destination_type)
{
    bool converted = false;

    if(source_type == file_type::XYZ_INTENSITY_RGB && destination_type == file_type::BINARY)
    {
        save_point_cloud(destination, load_point_cloud_xyz_intensity_rgb(source));
        converted = true;
    }

    if(source_type == file_type::XYZ_INTENSITY_RGB && destination_type == file_type::PLY)
    {
        save_point_cloud_ply(destination, load_point_cloud_xyz_intensity_rgb(source));
        converted = true;
    }

    if(source_type == file_type::BINARY && destination_type == file_type::PLY)
    {
        save_point_cloud_ply(destination, load_point_cloud(source));
        converted = true;
    }

    if(!converted)
        throw std::runtime_error("Conversion from " + file_type_to_string(source_type) + " to " + file_type_to_string(destination_type) + " is not supported.");
}