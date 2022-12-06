/*
 * UVIO: Ultra Wide-Band aided Visual-Inertial Odometry
 * Copyright (C) 2020-2022 Alessandro Fornasier
 * Copyright (C) 2018-2022 UVIO Contributors
 *
 * Control of Networked Systems, University of Klagenfurt, Austria (alessandro.fornasier@aau.at)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UVIO_SENSOR_DATA_H
#define UVIO_SENSOR_DATA_H

#include <unordered_map>

namespace uvio
{

/**
 * @brief Struct for a single uwb measurement (time, anchor_id (0,1,2,3,...), distance measurement)
 */
struct UwbData {

    /// Timestamp of the reading
    double timestamp;

    /// anchor id, distance measurement
    /// unordered_map is used since the IDs are not ordered in any sense
    std::unordered_map<size_t, double> uwb_ranges;

    /// Sort function to allow for using of STL containers
    bool operator<(const UwbData& other) const {
        return timestamp < other.timestamp;
    }
};

}

#endif // UVIO_SENSOR_DATA_H
