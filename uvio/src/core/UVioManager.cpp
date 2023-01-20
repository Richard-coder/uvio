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

  /// Initialize uvio state and propagator
  state = std::make_shared<UVioState>(params.uvio_state_options, this->get_state());
  propagator = std::make_shared<UVioPropagator>(params.imu_noises, params.gravity_mag, this->get_propagator());

  // Initialize p_UinI (calibration uwb-imu)
  if (params.uvio_state_options.do_calib_uwb_position) {    
    std::vector<std::shared_ptr<ov_type::Type>> H_order;

    // Need to fake it...
    H_order.push_back(state->_state->_imu->q());

    Eigen::Matrix3d H_R = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d H_L = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * params.uvio_state_options.prior_uwb_imu_cov;
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    ov_msckf::StateHelper::initialize_invertible(state->_state, state->_calib_UWBtoIMU, H_order, H_R, H_L, R, res);
    PRINT_DEBUG("Calibration uwb-imu initialized.\n");
  }

  // Our UWB sensor extrinsic transform
  state->_calib_UWBtoIMU->set_value(params.uwb_extrinsics);
  state->_calib_UWBtoIMU->set_fej(params.uwb_extrinsics);

  // Initialize anchors
  if (!params.do_anchors_mapping && !params.uwb_anchor_extrinsics.empty()) {
    initialize_uwb_anchors(params.uwb_anchor_extrinsics);
  }

  // Make the updater!
  updaterUWB = std::make_unique<UpdaterUWB>(params.uwb_options);
}

void UVioManager::feed_measurement_uwb(const UwbData &message) {

  // Check if vio is initialized, anchors are initialized and distace traveled is
  // greater than 1 meter, else return
  if(!(is_initialized_vio && are_initialized_anchors && distance > 0.5)) {
    return;
  }

  // Return if the uwb measurement is out of order otherwise feed our bar measuremnts
  if(state->_state->_timestamp >= message.timestamp) {
    PRINT_INFO(YELLOW "UWB measurements received out of order (prop dt = %3f)\n" RESET, (message.timestamp-state->_state->_timestamp));
    return;
  } else {
    do_uwb_propagate_update(std::make_shared<UwbData>(message));
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

      // Need to fake it...
      H_order.push_back(state->_state->_imu->q());

      Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(5, 3);
      Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(5, 5);
      Eigen::MatrixXd R = Eigen::MatrixXd::Identity(5, 5);
      Eigen::VectorXd res = Eigen::VectorXd::Zero(5);
      ov_msckf::StateHelper::initialize_invertible(
            state->_state, state->_calib_GLOBALtoANCHORS.at(it.id), H_order, H_R, H_L, R, res);

      H_order.clear();
      H_order.push_back(state->_calib_GLOBALtoANCHORS.at(it.id));

      ov_msckf::StateHelper::set_initial_covariance(state->_state, it.cov, H_order);

      PRINT_DEBUG("Added anchor[%d] to state.\n", it.id);
    }
  }
  are_initialized_anchors = true;
  PRINT_DEBUG("Anchors initialized.\n");
}

void UVioManager::do_uwb_propagate_update(const std::shared_ptr<UwbData> &message)
{
  // Propagate the state forward to the current update time
  propagator->propagate(state, message->timestamp);

  // Return if we where unable to propagate
  if(state->_state->_timestamp != message->timestamp) {
    printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,message->timestamp-state->_state->_timestamp);
    return;
  }

  // EKF Update with UWB measurement
  // updaterUWB->update(state, message);

  // Debug, print our current state
  PRINT_DEBUG("calib_UWtoIMU = [%.3f,%.3f,%.3f]\n", state->_calib_UWBtoIMU->value()(0), state->_calib_UWBtoIMU->value()(1), state->_calib_UWBtoIMU->value()(2));
  for (const auto &it : state->_calib_GLOBALtoANCHORS) {
    if (!it.second->fixed()) {
      PRINT_DEBUG("anchor[%d]: p_AinG = [%.3f, %.3f, %.3f] | const_bias = %.4f | dist_bias = %.4f\n", it.first,
                  it.second->p_AinG()->value()(0), it.second->p_AinG()->value()(1), it.second->p_AinG()->value()(2),
                  it.second->const_bias()->value()(0), it.second->dist_bias()->value()(0));
    }
  }
}
