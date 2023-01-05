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

#ifndef UVIO_MANAGER_H
#define UVIO_MANAGER_H

#include <core/UVioManagerOptions.h>
#include <core/VioManager.h>
#include <state/UVioState.h>
#include <utils/uvio_sensor_data.h>

namespace uvio {

class UVioManager : public ov_msckf::VioManager {

public:
  /**
   * @brief Constructor
   */
  UVioManager(UVioManagerOptions &params);

  /**
   * @brief Feed function for a uwb set of measurements
   * @param uwb measurements
   */
  void feed_measurement_uwb(const UwbData &message);

private:
  /// UVIO state
  std::shared_ptr<UVioState> uvio_state;
};

} // namespace uvio

#endif // UVIO_MANAGER_H
