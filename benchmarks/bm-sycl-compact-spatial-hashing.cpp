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
 * @file bm-sycl-compact-spatial-hashing.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bm-sycl-compact-spatial-hashing.h"

#include <sycl/sycl.hpp>
#include <chrono>

#include "point.h"
#include "dataset-generator.h"
#include "compact-spatial-hashing.h"
#include "spatial-hashing.h"

// -----------------------------------------------------------------------------
// Benchmark: SYCL GPU Compact Spatial Hashing (Isolated Latencies) and Radius Search
// -----------------------------------------------------------------------------
void bm_sycl_gpu_compact_spatial_hashing(benchmark::State &state)
{
    std::size_t const num_points = state.range(0);
    dataset_type const type      = static_cast<dataset_type>(state.range(1));

    auto const cloud                         = generate_dataset(num_points, type);
    std::vector<point_3d> const host_queries = {{0.0f, 0.0f, 0.0f}};

    float const radius                      = 2.5f;
    float const cell_size                   = radius / 2.0f;
    std::size_t const num_queries           = host_queries.size();
    std::size_t const max_results_per_query = 500;
    std::size_t const table_size            = get_next_prime(cloud.size() * 2);

    sycl::queue q(sycl::default_selector_v);

    for(auto _ : state)
    {
        // Allocation
        point_3d *d_cloud             = sycl::malloc_device<point_3d>(num_points, q);
        point_3d *d_queries           = sycl::malloc_device<point_3d>(num_queries, q);
        std::size_t *d_bucket_offsets = sycl::malloc_device<std::size_t>(table_size + 1, q);
        std::size_t *d_point_indices  = sycl::malloc_device<std::size_t>(num_points, q);
        std::size_t *d_results        = sycl::malloc_device<std::size_t>(num_queries * max_results_per_query, q);
        std::size_t *d_result_counts  = sycl::malloc_device<std::size_t>(num_queries, q);

        // Measure PCIe host-to-device copy
        auto t_transfer_start = std::chrono::high_resolution_clock::now();
        q.memcpy(d_cloud, cloud.data(), num_points * sizeof(point_3d));
        q.memcpy(d_queries, host_queries.data(), num_queries * sizeof(point_3d));
        q.wait();
        auto t_transfer_mid = std::chrono::high_resolution_clock::now();

        // Measure GPU compute execution
        auto t_compute_start = std::chrono::high_resolution_clock::now();
        build_spatial_hash_sycl(q, d_cloud, num_points, cell_size, table_size, d_bucket_offsets, d_point_indices);
        radius_search_sycl(
            q, d_queries, num_queries, radius, d_cloud, num_points, cell_size, table_size, d_bucket_offsets, d_point_indices, d_results, d_result_counts, max_results_per_query);
        q.wait();
        auto t_compute_end = std::chrono::high_resolution_clock::now();

        // Measure PCIe device-to-host copy
        std::vector<std::size_t> host_result_counts(num_queries);
        std::vector<std::size_t> host_results(num_queries * max_results_per_query);
        q.memcpy(host_result_counts.data(), d_result_counts, num_queries * sizeof(std::size_t));
        q.memcpy(host_results.data(), d_results, num_queries * max_results_per_query * sizeof(std::size_t));
        q.wait();
        auto t_transfer_end = std::chrono::high_resolution_clock::now();

        // Calculate isolated metrics
        double pcie_h2d_ms = std::chrono::duration<double, std::milli>(t_transfer_mid - t_transfer_start).count();
        double compute_ms  = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();
        double pcie_d2h_ms = std::chrono::duration<double, std::milli>(t_transfer_end - t_compute_end).count();

        state.counters["pcie_transfer_ms"] = pcie_h2d_ms + pcie_d2h_ms;
        state.counters["compute_ms"]       = compute_ms;
        state.counters["total_gpu_ms"]     = pcie_h2d_ms + compute_ms + pcie_d2h_ms;

        // Memory deallocation
        sycl::free(d_cloud, q);
        sycl::free(d_queries, q);
        sycl::free(d_bucket_offsets, q);
        sycl::free(d_point_indices, q);
        sycl::free(d_results, q);
        sycl::free(d_result_counts, q);
    }
}

// -----------------------------------------------------------------------------
// Benchmark: SYCL CPU Compact Spatial Hashing (Isolated Latencies) and Radius Search
// -----------------------------------------------------------------------------

void bm_sycl_cpu_compact_spatial_hashing(benchmark::State &state)
{
    std::size_t const num_points = state.range(0);
    dataset_type const type      = static_cast<dataset_type>(state.range(1));

    auto const cloud                         = generate_dataset(num_points, type);
    std::vector<point_3d> const host_queries = {{0.0f, 0.0f, 0.0f}};

    float const radius                      = 2.5f;
    float const cell_size                   = radius / 2.0f;
    std::size_t const num_queries           = host_queries.size();
    std::size_t const max_results_per_query = 500;
    std::size_t const table_size            = get_next_prime(cloud.size() * 2);

    sycl::queue q(sycl::cpu_selector_v);

    for(auto _ : state)
    {
        // Allocation
        point_3d *d_cloud             = sycl::malloc_device<point_3d>(num_points, q);
        point_3d *d_queries           = sycl::malloc_device<point_3d>(num_queries, q);
        std::size_t *d_bucket_offsets = sycl::malloc_device<std::size_t>(table_size + 1, q);
        std::size_t *d_point_indices  = sycl::malloc_device<std::size_t>(num_points, q);
        std::size_t *d_results        = sycl::malloc_device<std::size_t>(num_queries * max_results_per_query, q);
        std::size_t *d_result_counts  = sycl::malloc_device<std::size_t>(num_queries, q);

        // Measure PCIe host-to-device copy
        auto t_transfer_start = std::chrono::high_resolution_clock::now();
        q.memcpy(d_cloud, cloud.data(), num_points * sizeof(point_3d));
        q.memcpy(d_queries, host_queries.data(), num_queries * sizeof(point_3d));
        q.wait();
        auto t_transfer_mid = std::chrono::high_resolution_clock::now();

        // Measure GPU compute execution
        auto t_compute_start = std::chrono::high_resolution_clock::now();
        build_spatial_hash_sycl(q, d_cloud, num_points, cell_size, table_size, d_bucket_offsets, d_point_indices);
        radius_search_sycl(
            q, d_queries, num_queries, radius, d_cloud, num_points, cell_size, table_size, d_bucket_offsets, d_point_indices, d_results, d_result_counts, max_results_per_query);
        q.wait();
        auto t_compute_end = std::chrono::high_resolution_clock::now();

        // Measure PCIe device-to-host copy
        std::vector<std::size_t> host_result_counts(num_queries);
        std::vector<std::size_t> host_results(num_queries * max_results_per_query);
        q.memcpy(host_result_counts.data(), d_result_counts, num_queries * sizeof(std::size_t));
        q.memcpy(host_results.data(), d_results, num_queries * max_results_per_query * sizeof(std::size_t));
        q.wait();
        auto t_transfer_end = std::chrono::high_resolution_clock::now();

        // Calculate isolated metrics
        double pcie_h2d_ms = std::chrono::duration<double, std::milli>(t_transfer_mid - t_transfer_start).count();
        double compute_ms  = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();
        double pcie_d2h_ms = std::chrono::duration<double, std::milli>(t_transfer_end - t_compute_end).count();

        state.counters["pcie_transfer_ms"] = pcie_h2d_ms + pcie_d2h_ms;
        state.counters["compute_ms"]       = compute_ms;
        state.counters["total_cpu_ms"]     = pcie_h2d_ms + compute_ms + pcie_d2h_ms;

        // Memory deallocation
        sycl::free(d_cloud, q);
        sycl::free(d_queries, q);
        sycl::free(d_bucket_offsets, q);
        sycl::free(d_point_indices, q);
        sycl::free(d_results, q);
        sycl::free(d_result_counts, q);
    }
}
