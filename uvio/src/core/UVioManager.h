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

#include "core/UVioManagerOptions.h"
#include "core/VioManager.h"
#include "state/UVioState.h"
#include "update/UpdaterUWB.h"

namespace uvio {

class UVioManager : public ov_msckf::VioManager {

public:
  /**
   * @brief Constructor
   */
  UVioManager(UVioManagerOptions &params_);

  /**
   * @brief Feed function for a uwb set of measurements
   * @param uwb measurements
   */
  void feed_measurement_uwb(const UwbData &message);

  /// Accessor to get the current uvio state
  inline std::shared_ptr<UVioState> get_uvio_state() { return state; }

private:
  /// Manager parameters
  UVioManagerOptions params;

  /// UVIO state
  std::shared_ptr<UVioState> state;

  /**
   * @brief This function will initialize UWB anchors into the state.
   */
  void initialize_uwb_anchors(const std::vector<AnchorData> &anchors);

  /**
  * @brief This will do the propagation and uwb update to the state
  * @param Reference to pointer to uwb range measurements
  */
  void do_uwb_propagate_update(const std::shared_ptr<UwbData>& message);

  /// Our uwb updater
  std::unique_ptr<UpdaterUWB> updaterUWB;

  /// Boolean if uwb anchors are initialized or not
  bool are_initialized_anchors = false;
};

} // namespace uvio

#endif // UVIO_MANAGER_H
