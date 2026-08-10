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
 * @file radius-search.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "radius-search.h"

#include <hipSYCL/sycl/device_selector.hpp>
#include <sycl/sycl.hpp>
#include "dispatcher.h"
#include "spatial-hashing.h"
#include "compact-spatial-hashing.h"

std::vector<std::vector<point_3d>> radius_search_dispatcher(
    std::vector<point_3d> const &point_cloud, std::vector<point_3d> const &host_queries, float search_radius, float cell_size, std::size_t const max_results_per_query)
{
    spatial_characteristics characteristics = analyze_point_cloud_fast(point_cloud, 256);
    execution_target decision               = evaluate_dispatch_decision(characteristics);
    // std::cout << "Dispatcher effective density: " << std::to_string(characteristics.effective_density) << std::endl;
    // std::cout << "Dispatcher normalized entropy: " << std::to_string(characteristics.normalized_entropy) << std::endl;
    // std::cout << "Dispatcher decision: " << ((decision == execution_target::sycl_gpu) ? "sycl_gpu" : "cpu_multithreaded") << std::endl;
    std::vector<std::vector<point_3d>> result;

    if(decision == execution_target::sycl_gpu)
    {
        std::size_t const expected_capacity = point_cloud.size() * 2;
        std::size_t const table_size        = get_next_prime(expected_capacity);
        std::size_t const num_queries       = host_queries.size(); // Basically points to search (see query_point)

        sycl::queue q(sycl::gpu_selector_v); // GPU // TODO: Write a custom selector
        // std::cout << "Execution target device: " << q.get_device().get_info<sycl::info::device::name>() << std::endl;

        // USM device memory allocation
        // Allocate memory explicitly on the device to guarantee PCIe transfer boundaries
        point_3d *d_cloud   = sycl::malloc_device<point_3d>(point_cloud.size(), q);
        point_3d *d_queries = sycl::malloc_device<point_3d>(num_queries, q);

        std::size_t *d_bucket_offsets = sycl::malloc_device<std::size_t>(table_size + 1, q);
        std::size_t *d_point_indices  = sycl::malloc_device<std::size_t>(point_cloud.size(), q);

        std::size_t *d_results       = sycl::malloc_device<std::size_t>(num_queries * max_results_per_query, q);
        std::size_t *d_result_counts = sycl::malloc_device<std::size_t>(num_queries, q);

        // Host-to-device data transfer
        q.memcpy(d_cloud, point_cloud.data(), point_cloud.size() * sizeof(point_3d)).wait();
        q.memcpy(d_queries, host_queries.data(), num_queries * sizeof(point_3d)).wait();

        // Kernel execution
        build_spatial_hash_sycl(q, d_cloud, point_cloud.size(), cell_size, table_size, d_bucket_offsets, d_point_indices);
        radius_search_sycl(
            q,
            d_queries,
            num_queries,
            search_radius,
            d_cloud,
            point_cloud.size(),
            cell_size,
            table_size,
            d_bucket_offsets,
            d_point_indices,
            d_results,
            d_result_counts,
            max_results_per_query);

        // Device-to-host result retrieval
        std::vector<std::size_t> host_result_counts(num_queries);
        std::vector<std::size_t> host_results(num_queries * max_results_per_query);

        q.memcpy(host_result_counts.data(), d_result_counts, num_queries * sizeof(std::size_t)).wait();
        q.memcpy(host_results.data(), d_results, (num_queries * max_results_per_query) * sizeof(std::size_t)).wait();

        // Results
        for(std::size_t i = 0; i < num_queries; ++i)
        {
            std::size_t const match_count = host_result_counts[i];

            std::vector<point_3d> res;

            std::size_t const base_idx = i * max_results_per_query;
            for(std::size_t j = 0; j < match_count; ++j)
            {
                std::size_t const point_idx = host_results[base_idx + j];
                // Access original coordinate via host_cloud[point_idx] if needed.
                res.push_back(point_cloud[point_idx]);
            }

            result.push_back(res);
        }

        // Memory deallocation
        sycl::free(d_cloud, q);
        sycl::free(d_queries, q);
        sycl::free(d_bucket_offsets, q);
        sycl::free(d_point_indices, q);
        sycl::free(d_results, q);
        sycl::free(d_result_counts, q);

        return result;
    }
    else
    {
        std::size_t const expected_capacity = point_cloud.size() * 2;
        std::size_t const table_size        = get_next_prime(expected_capacity);

        compact_spatial_hash_table const compact_hash_table = build_compact_spatial_hash(point_cloud, cell_size, table_size);

        for(point_3d const &query_point : host_queries)
        {
            std::vector<std::size_t> const found_indices = radius_search(query_point, search_radius, point_cloud, compact_hash_table);

            std::size_t const count = std::min(found_indices.size(), max_results_per_query);

            std::vector<point_3d> query_result;
            query_result.reserve(count);

            for(std::size_t i = 0; i < count; ++i)
            {
                query_result.push_back(point_cloud[found_indices[i]]);
            }

            result.push_back(query_result);
        }

        return result;
    }
}

#include <iostream>

void print_radius_search_results(std::vector<std::vector<point_3d>> const &r)
{
    std::cout << "Number of queries: " << std::to_string(r.size()) << std::endl;
    for(std::size_t i = 0; i < r.size(); i++)
    {
        std::cout << "  [" << std::to_string(i) << "] results (" << std::to_string(r[i].size()) << "):" << std::endl;
        for(auto const &p : r[i])
        {
            std::cout << "    " << p << std::endl;
        }
    }
}
