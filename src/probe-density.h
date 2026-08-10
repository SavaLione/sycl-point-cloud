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
 * @file probe-density.h
 * @brief 
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef PROBE_DENSITY_H
#define PROBE_DENSITY_H

#include "point.h"
#include <vector>

struct spatial_characteristics
{
    std::size_t total_points;
    float effective_density;
    float normalized_entropy; // [0.0, 1.0]: 1.0 = Uniform distribution, lower = High clustering
};

/**
 * @brief Computes spatial characteristics in O(M) time using a fixed sub-sample.
 * 
 * @param cloud Full input point cloud buffer.
 * @param sample_size Constant number of points to evaluate (default: 256).
 * @return spatial_characteristics Calculated metrics.
 */
spatial_characteristics analyze_point_cloud_fast(std::vector<point_3d> const &cloud, std::size_t sample_size = 256);

#endif // PROBE_DENSITY_H