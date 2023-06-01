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
#include "feat/FeatureDatabase.h"
#include "state/UVioState.h"
#include "update/UpdaterUWB.h"
#include "update/UpdaterZeroVelocity.h"

#include "state/UVioPropagator.h"

namespace uvio {

class UVioManager : public ov_msckf::VioManager {

public:
  /**
   * @brief Constructor
   */
  UVioManager(UVioManagerOptions &params_);

  /**
   * @brief Feed function for camera measurements
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_measurement_camera(const ov_core::CameraData &message) { track_image_and_update(message); }

  /**
   * @brief Feed function for a uwb set of measurements
   * @param uwb measurements
   */
  void feed_measurement_uwb(const UwbData &message);

  /**
   * @brief Initialize uwb anchors
   * @param anchors
   */
  void try_to_initialize_uwb_anchors(const std::vector<AnchorData> &anchors);

  /**
   * @brief Get number of fixed anchors
   */
  int get_n_anchors_to_fix() { return params.n_anchors_to_fix; };

  /**
   * @brief Get if anchors are initialized
   */
  bool get_are_initialized_anchors() { return are_initialized_anchors; };

  /// Accessor to get the current state
  std::shared_ptr<UVioState> get_uvio_state() { return state; }

private:
  /// Manager parameters
  UVioManagerOptions params;

  /// Our UVioState (state->_state for State)
  std::shared_ptr<UVioState> state;

  /// Propagator of our state
  std::shared_ptr<UVioPropagator> propagator;

  /**
   * @brief Given a new set of camera images, this will track them.
   *
   * If we are having stereo tracking, we should call stereo tracking functions.
   * Otherwise we will try to track on each of the images passed.
   *
   * @param message Contains our timestamp, images, and camera ids
   */
  void track_image_and_update(const ov_core::CameraData &message);

  /**
   * @brief This function will initialize all UWB anchors stored in params.uwb_anchors into the state.
   */
  void initialize_uwb_anchors();

  /**
   * @brief This function will initialize a new single UWB anchor provided as argument into the state.
   * @param anchor
   */
  void initialize_new_uwb_anchor(const AnchorData &anchor);

  /**
   * @brief This will do the propagation and uwb update to the state
   * @param Reference to pointer to uwb range measurements
   */
  void do_uwb_propagate_update(const std::shared_ptr<UwbData> &message);

  /// Our uwb updater
  std::unique_ptr<UpdaterUWB> updaterUWB;

  /// Our uwb measurements buffer
  std::map<double, std::shared_ptr<UwbData>> past_measurements;

  /// Boolean if uwb anchors are initialized or not
  bool are_initialized_anchors = false;
};

} // namespace uvio

#endif // UVIO_MANAGER_H
