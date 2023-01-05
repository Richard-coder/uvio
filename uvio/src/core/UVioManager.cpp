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

UVioManager::UVioManager(UVioManagerOptions &params) : ov_msckf::VioManager::VioManager(params) {

  // [COMMENT] VioManager (the parent class) initialize the state and holds a pointer to the state.
  // The uvio state is defined as a pointer and it can be initialized calling the copy constructor of the base class
  uvio_state = std::make_shared<UVioState>(*this->get_state(), params.uvio_state_options);

  // [DEBUG] If correct than cam intrinsic should have the correct inizialized value from VioManager
  std::cout << "\n\nCam intrinsic: " << uvio_state->_cam_intrinsics.at(0)->value() << "\n\n" << std::endl;

  // [COMMENT] Tested it works so at this point uvio state has the "State" part already initialized and
  // the only thing to do is to initialize (set value and fej) of _calib_UWBtoIMU
}

void UVioManager::feed_measurement_uwb(const UwbData &message) {
  //  // If we do not have VIO initialization or if we are on the ground, then return
  //  // Note: UWB is poor on the ground, we do not use the measurement if the pose is
  //  // within a 50cm radius ball from the initial pose
  //  double dist = Eigen::Vector3d(state->_imu->pos() - state->_imu0->pos()).norm();
  //  if(!is_initialized_vio || dist < 0.5 || !are_initialized_anchors) {
  //    return;
  //  }

  for (const auto &it : message.uwb_ranges) {
    PRINT_DEBUG(GREEN "[UWB]: anchor[%d] range = %.3f m\n" RESET, it.first, it.second);
  }
}
