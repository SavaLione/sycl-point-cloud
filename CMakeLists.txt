#######################################################################
# Copyright (C) 2026 Savelii Pototskii (savalione.com)
# 
# Author: Savelii Pototskii <savelii.pototskii@gmail.com>
# 
# This file is part of sycl-point-cloud.
# 
# sycl-point-cloud is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation, either version 3
# of the License, or (at your option) any later version.
# 
# sycl-point-cloud is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with sycl-point-cloud. If not, see <https://www.gnu.org/licenses/>.
#######################################################################
cmake_minimum_required (VERSION 3.28)

project(sycl-point-cloud
    VERSION "1.0.7"
    DESCRIPTION "An O(1) dynamic task routing mechanism for 3D point cloud radius search operations"
    HOMEPAGE_URL "https://github.com/SavaLione/sycl-point-cloud"
    LANGUAGES C CXX
)

set(PROJECT_AUTHOR "Savelii Pototskii (savalione.com)")
set(PROJECT_LICENSE "GPL-3.0-or-later")

# Project configuration
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BUILD_SHARED_LIBS OFF)

# Provide compile commands for tools like clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Suppress 'argument unused during compilation' warnings from AdaptiveCpp/Clang
add_compile_options(-Qunused-arguments)

# Set a default build type if none is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (Release, Debug, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

# Dependencies
find_package(AdaptiveCpp CONFIG REQUIRED)
find_package(benchmark)

# Configuration file generation
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config.h"
)

# Sources and executable definition
set(SPC_SRC
    src/io.cpp
    src/point.cpp
    src/spatial-hashing.cpp
    src/compact-spatial-hashing.cpp
    src/sycl-kd-tree.cpp
    src/dataset-generator.cpp
    src/probe-density.cpp
    src/radius-search.cpp
    src/dispatcher.cpp
    src/utils.cpp
    src/xgetopt/xgetopt.c
)

set(SPC_EXECUTABLE_SRC
    src/sycl-point-cloud.cpp
    ${SPC_SRC}
)

add_executable(${PROJECT_NAME} ${SPC_EXECUTABLE_SRC})

# Define a SYCL AdaptiveCpp project
add_sycl_to_target(
    TARGET ${PROJECT_NAME}
    SOURCES ${SPC_EXECUTABLE_SRC}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    "${PROJECT_SOURCE_DIR}/src"
)

# Benchmark executable
set(BENCHMARK_SRC
    benchmarks/bm-common.cpp
    benchmarks/bm-compact-spatial-hashing.cpp
    benchmarks/bm-dispatcher.cpp
    benchmarks/bm-probe-density.cpp
    benchmarks/bm-sycl-compact-spatial-hashing.cpp
    benchmarks/bm-sycl-kd-tree.cpp
    ${SPC_SRC}
)

add_executable(sycl-point-cloud-benchmarks ${BENCHMARK_SRC})

add_sycl_to_target(
    TARGET sycl-point-cloud-benchmarks
    SOURCES ${BENCHMARK_SRC}
)

target_include_directories(sycl-point-cloud-benchmarks PUBLIC "${PROJECT_SOURCE_DIR}/src")
target_include_directories(sycl-point-cloud-benchmarks PUBLIC "${PROJECT_SOURCE_DIR}/benchmarks")
target_link_libraries(sycl-point-cloud-benchmarks PRIVATE benchmark::benchmark)

# Convert executable
set(SPC_CONVERT_SRC
    src/convert/sycl-point-cloud-convert.cpp
    src/io.cpp
    src/point.cpp
    src/xgetopt/xgetopt.c
)

add_executable(sycl-point-cloud-convert ${SPC_CONVERT_SRC})

target_include_directories(sycl-point-cloud-convert PUBLIC "${PROJECT_SOURCE_DIR}/src")
target_include_directories(sycl-point-cloud-convert PUBLIC "${PROJECT_SOURCE_DIR}/src/convert")
