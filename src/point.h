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
 * @file point.h
 * @brief Defines the fundamental three-dimensional point structure and memory layout requirements.
 * @author Savelii Pototskii
 * @copyright Copyright (C) 2026 Savelii Pototskii (savalione.com)
 * @copyright SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef POINT_H
#define POINT_H

#include <iostream>

/**
 * @brief Representation of a 3D point in space.
 */
struct point_3d
{
    float x, y, z;

    friend std::ostream &operator<<(std::ostream &os, point_3d const &p)
    {
        os << "x = " << std::to_string(p.x) << " y = " << std::to_string(p.y) << " z = " << std::to_string(p.z);

        return os;
    }
};

static_assert(sizeof(point_3d) == 3 * sizeof(float), "point_3d layout must match binary standard layout.");

#endif // POINT_H