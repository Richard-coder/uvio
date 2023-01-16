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

UVioManager::UVioManager(UVioManagerOptions &params_) : ov_msckf::VioManager::VioManager(params_), params(params_) {

  // [COMMENT] VioManager (the parent class) initialize the state and holds a pointer to the state.
  // The uvio state is defined as a pointer and it can be initialized calling the copy constructor of the base class
  state = std::make_shared<UVioState>(*this->get_state(), params.uvio_state_options);

  // [COMMENT] Tested it works so at this point uvio state has the "State" part already initialized and
  // the only thing to do is to initialize (set value and fej) of _calib_UWBtoIMU
  if (params.uvio_state_options.do_calib_uwb_position) {
    std::vector<std::shared_ptr<ov_type::Type>> H_order;
    Eigen::MatrixXd H_R = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d H_L = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * params.uvio_state_options.prior_uwb_imu_cov;
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    ov_msckf::StateHelper::initialize_invertible(state, state->_calib_UWBtoIMU, H_order, H_R, H_L, R, res);
  }

  // Our UWB sensor extrinsic transform
  state->_calib_UWBtoIMU->set_value(params.uwb_extrinsics);
  state->_calib_UWBtoIMU->set_fej(params.uwb_extrinsics);

  // [TMP DEBUG] Vector of anchors
  for (size_t i = 0; i < 4; i++) {
    AnchorData tmp;
    tmp.id = i + 110;
    tmp.fix = (i < 2) ? true : false;
    tmp.p_AinG = Eigen::Vector3d::Constant(i);
    tmp.const_bias = i + 0.5;
    tmp.dist_bias = i + 0.1;
    tmp.cov = Eigen::MatrixXd::Identity(5, 5) * 0.017;
    params.uwb_anchor_extrinsics.push_back(tmp);
  }

  // Initialize anchors
  if (!params.uwb_anchor_extrinsics.empty()) {
    initialize_uwb_anchors(params.uwb_anchor_extrinsics);
  }

  // [TMP DEBUG] print covariance of claibration uwb-imu
  std::cout << "UWB calibration covariance: " << ov_msckf::StateHelper::get_marginal_covariance(state, {state->_calib_UWBtoIMU}) << std::endl;

  // Make the updater!
  updaterUWB = std::make_unique<UpdaterUWB>(params.uwb_options);
}

void UVioManager::feed_measurement_uwb(const UwbData &message) {
  //  // If we do not have VIO initialization or if we are on the ground, then return
  //  // Note: UWB is poor on the ground, we do not use the measurement if the pose is
  //  // within a 50cm radius ball from the initial pose

  // TODO: there is no more _imu0
//  double dist = Eigen::Vector3d(state->_imu->pos() - state->_imu0->pos()).norm();
  if(!is_initialized_vio ||/* dist < 0.5 ||*/ !are_initialized_anchors) {
    return;
  }

  // [DEBUG]
  for (const auto &it : message.uwb_ranges) {
    PRINT_DEBUG(GREEN "[UWB]: anchor[%d] range = %.3f m\n" RESET, it.first, it.second);
  }
}

void UVioManager::initialize_uwb_anchors(const std::vector<AnchorData> &anchors)
{
  for (const auto &it : anchors) {
    std::shared_ptr<UWB_anchor> anchor = std::make_shared<UWB_anchor>(it);
    state->_calib_GLOBALtoANCHORS.insert({it.id, anchor});

    // Initialize state variable if option enabled and anchor not fixed
    if (params.uvio_state_options.do_calib_uwb_anchors && !it.fix) {
      // Initialize state variables
      std::vector<std::shared_ptr<ov_type::Type>> H_order;
      Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(5, 5);
      Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(5, 5);
      Eigen::MatrixXd R = it.cov;
      Eigen::VectorXd res = Eigen::VectorXd::Zero(5);
      ov_msckf::StateHelper::initialize_invertible(
            state, state->_calib_GLOBALtoANCHORS.at(it.id), H_order, H_R, H_L, R, res);
      PRINT_DEBUG("Added anchor[%d] to state.\n", it.id);
    }
  }
  are_initialized_anchors = true;
  PRINT_DEBUG("Anchors initialized.\n");
}
