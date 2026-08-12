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
 * @file bm-sycl-kd-tree.cpp
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bm-sycl-kd-tree.h"

#include <sycl/sycl.hpp>
#include <chrono>

#include "sycl-kd-tree.h"
#include "point.h"
#include "dataset-generator.h"
#include "io.h"

// -----------------------------------------------------------------------------
// Benchmark: SYCL CPU K-D Tree Construction and Radius Search
// -----------------------------------------------------------------------------
void bm_sycl_cpu_kd_tree(benchmark::State &state)
{
    std::size_t const num_points = state.range(0);
    dataset_type const type      = static_cast<dataset_type>(state.range(1));

    auto const cloud                         = generate_dataset(num_points, type);
    std::vector<point_3d> const host_queries = {{0.0f, 0.0f, 0.0f}};

    float const radius                      = 2.5f;
    std::size_t const num_queries           = host_queries.size();
    std::size_t const max_results_per_query = 500;

    // Explicitly target the host CPU execution device via SYCL CPU selector
    sycl::queue q(sycl::cpu_selector_v);

    for(auto _ : state)
    {
        // Allocation of USM buffers
        point_3d *d_cloud            = sycl::malloc_device<point_3d>(num_points, q);
        point_3d *d_queries          = sycl::malloc_device<point_3d>(num_queries, q);
        sycl_kd_node *d_tree_nodes   = sycl::malloc_device<sycl_kd_node>(num_points, q);
        std::int32_t *d_root_idx     = sycl::malloc_device<std::int32_t>(1, q);
        std::size_t *d_results       = sycl::malloc_device<std::size_t>(num_queries * max_results_per_query, q);
        std::size_t *d_result_counts = sycl::malloc_device<std::size_t>(num_queries, q);

        // Measure host-to-device transfer latency
        auto t_transfer_start = std::chrono::high_resolution_clock::now();
        q.memcpy(d_cloud, cloud.data(), num_points * sizeof(point_3d));
        q.memcpy(d_queries, host_queries.data(), num_queries * sizeof(point_3d));
        q.wait();
        auto t_transfer_mid = std::chrono::high_resolution_clock::now();

        // Measure computation execution (tree construction + radius search)
        auto t_compute_start = std::chrono::high_resolution_clock::now();

        build_kd_tree_sycl(q, d_cloud, num_points, d_tree_nodes, d_root_idx);

        std::int32_t host_root_idx = -1;
        q.memcpy(&host_root_idx, d_root_idx, sizeof(std::int32_t)).wait();

        radius_search_kd_tree_sycl(q, d_queries, num_queries, radius, d_tree_nodes, host_root_idx, d_results, d_result_counts, max_results_per_query);
        q.wait();
        auto t_compute_end = std::chrono::high_resolution_clock::now();

        // Measure device-to-host transfer latency
        std::vector<std::size_t> host_result_counts(num_queries);
        std::vector<std::size_t> host_results(num_queries * max_results_per_query);
        q.memcpy(host_result_counts.data(), d_result_counts, num_queries * sizeof(std::size_t));
        q.memcpy(host_results.data(), d_results, num_queries * max_results_per_query * sizeof(std::size_t));
        q.wait();
        auto t_transfer_end = std::chrono::high_resolution_clock::now();

        // Calculate timing metrics
        double h2d_transfer_ms = std::chrono::duration<double, std::milli>(t_transfer_mid - t_transfer_start).count();
        double compute_ms      = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();
        double d2h_transfer_ms = std::chrono::duration<double, std::milli>(t_transfer_end - t_compute_end).count();

        state.counters["transfer_h2d_ms"]   = h2d_transfer_ms;
        state.counters["compute_ms"]        = compute_ms;
        state.counters["transfer_d2h_ms"]   = d2h_transfer_ms;
        state.counters["total_sycl_cpu_ms"] = h2d_transfer_ms + compute_ms + d2h_transfer_ms;

        benchmark::DoNotOptimize(host_results);
        benchmark::DoNotOptimize(host_result_counts);

        // Memory deallocation
        sycl::free(d_cloud, q);
        sycl::free(d_queries, q);
        sycl::free(d_tree_nodes, q);
        sycl::free(d_root_idx, q);
        sycl::free(d_results, q);
        sycl::free(d_result_counts, q);
    }
    state.SetItemsProcessed(state.iterations() * num_points);
}

void bm_sycl_cpu_kd_tree_file(benchmark::State &state)
{
    const char *env = std::getenv("BENCHMARK_POINT_CLOUD");
    if(!env)
    {
        state.SkipWithError("BENCHMARK_POINT_CLOUD environment variable is not set.");
        return;
    }

    std::vector<point_3d> const cloud        = load_point_cloud(env);
    std::vector<point_3d> const host_queries = {{0.0f, 0.0f, 0.0f}};
    float const radius                       = 2.5f;
    std::size_t const num_queries            = host_queries.size();
    std::size_t const max_results_per_query  = 500;

    // Explicitly target the host CPU execution device via SYCL CPU selector
    sycl::queue q(sycl::cpu_selector_v);

    for(auto _ : state)
    {
        // Allocation of USM buffers
        point_3d *d_cloud            = sycl::malloc_device<point_3d>(cloud.size(), q);
        point_3d *d_queries          = sycl::malloc_device<point_3d>(num_queries, q);
        sycl_kd_node *d_tree_nodes   = sycl::malloc_device<sycl_kd_node>(cloud.size(), q);
        std::int32_t *d_root_idx     = sycl::malloc_device<std::int32_t>(1, q);
        std::size_t *d_results       = sycl::malloc_device<std::size_t>(num_queries * max_results_per_query, q);
        std::size_t *d_result_counts = sycl::malloc_device<std::size_t>(num_queries, q);

        // Measure host-to-device transfer latency
        auto t_transfer_start = std::chrono::high_resolution_clock::now();
        q.memcpy(d_cloud, cloud.data(), cloud.size() * sizeof(point_3d));
        q.memcpy(d_queries, host_queries.data(), num_queries * sizeof(point_3d));
        q.wait();
        auto t_transfer_mid = std::chrono::high_resolution_clock::now();

        // Measure computation execution (tree construction + radius search)
        auto t_compute_start = std::chrono::high_resolution_clock::now();

        build_kd_tree_sycl(q, d_cloud, cloud.size(), d_tree_nodes, d_root_idx);

        std::int32_t host_root_idx = -1;
        q.memcpy(&host_root_idx, d_root_idx, sizeof(std::int32_t)).wait();

        radius_search_kd_tree_sycl(q, d_queries, num_queries, radius, d_tree_nodes, host_root_idx, d_results, d_result_counts, max_results_per_query);
        q.wait();
        auto t_compute_end = std::chrono::high_resolution_clock::now();

        // Measure device-to-host transfer latency
        std::vector<std::size_t> host_result_counts(num_queries);
        std::vector<std::size_t> host_results(num_queries * max_results_per_query);
        q.memcpy(host_result_counts.data(), d_result_counts, num_queries * sizeof(std::size_t));
        q.memcpy(host_results.data(), d_results, num_queries * max_results_per_query * sizeof(std::size_t));
        q.wait();
        auto t_transfer_end = std::chrono::high_resolution_clock::now();

        // Calculate timing metrics
        double h2d_transfer_ms = std::chrono::duration<double, std::milli>(t_transfer_mid - t_transfer_start).count();
        double compute_ms      = std::chrono::duration<double, std::milli>(t_compute_end - t_compute_start).count();
        double d2h_transfer_ms = std::chrono::duration<double, std::milli>(t_transfer_end - t_compute_end).count();

        state.counters["transfer_h2d_ms"]   = h2d_transfer_ms;
        state.counters["compute_ms"]        = compute_ms;
        state.counters["transfer_d2h_ms"]   = d2h_transfer_ms;
        state.counters["total_sycl_cpu_ms"] = h2d_transfer_ms + compute_ms + d2h_transfer_ms;

        benchmark::DoNotOptimize(host_results);
        benchmark::DoNotOptimize(host_result_counts);

        // Memory deallocation
        sycl::free(d_cloud, q);
        sycl::free(d_queries, q);
        sycl::free(d_tree_nodes, q);
        sycl::free(d_root_idx, q);
        sycl::free(d_results, q);
        sycl::free(d_result_counts, q);
    }
    state.SetItemsProcessed(state.iterations() * cloud.size());
}
