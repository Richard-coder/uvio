/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "UVioState.h"

using namespace uvio;

UVioState::UVioState(const ov_msckf::State &state, UVioStateOptions &options) : ov_msckf::State(state), _uvio_options(options) {
  /// If enabled add to state variables calibration UWB tag - IMU
  /// NOTE! This initialization can also be done later in UVioManager.cpp (as for the anchors)
  if (_uvio_options.do_calib_uwb_position) {
    std::vector<std::shared_ptr<ov_type::Type>> H_order;
    Eigen::MatrixXd H_R;
    Eigen::Matrix3d H_L = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * _uvio_options.prior_uwb_imu_cov;
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    ov_msckf::StateHelper::initialize_invertible(shared_from_this(), _calib_UWBtoIMU, H_order, H_R, H_L, R, res);
  }
}
