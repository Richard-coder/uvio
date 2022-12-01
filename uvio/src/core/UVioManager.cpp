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

#include <core/UVioManager.h>

using namespace uvio;

void UVioManager::feed_measurement_uwb(const UwbData &message)
{
  std::cout << "RECEIVED THE FOLLOWING UWB MEASUREMENT: " << std::endl;
  for (const auto& it : message.uwb_ranges)
  {
    std::cout << "Anchor: " << it.first << "\nRange: " << it.second << std::endl;
  }
//  // If we do not have VIO initialization or if we are on the ground, then return
//  // Note: UWB is poor on the ground, we do not use the measurement if the pose is
//  // within a 50cm radius ball from the initial pose
//  double dist = Eigen::Vector3d(state->_imu->pos() - state->_imu0->pos()).norm();
//  if(!is_initialized_vio || dist < 0.5 || !are_initialized_anchors) {
//    return;
//  }
}
