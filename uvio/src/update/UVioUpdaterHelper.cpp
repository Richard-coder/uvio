/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2020 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria (alessandro.fornasier@aau.at)
 * Copyright (C) 2020 OpenVINS Contributors
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

#include "UVioUpdaterHelper.h"

using namespace uvio;

void UVioUpdaterHelper::get_uwb_jacobian_full(std::shared_ptr<UVioState> state, std::shared_ptr<UwbData> measurement, Eigen::MatrixXd &H_x,
                                              Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order) {

  // Compute the size of the states involved with this feature
  int total_hx = 0;
  std::vector<size_t> state_anc_id;
  std::unordered_map<std::shared_ptr<ov_type::Type>, size_t> map_hx;

  // Add state clone
  std::shared_ptr<ov_type::PoseJPL> clone_I = state->_state->_imu->pose();
  map_hx.insert({clone_I, total_hx});
  x_order.push_back(clone_I);
  total_hx += clone_I->size();

  // Add extrinsics
  std::shared_ptr<ov_type::Vec> calibration = state->_calib_UWBtoIMU;
  if (state->_options.do_calib_uwb_extrinsics) {
    map_hx.insert({calibration, total_hx});
    x_order.push_back(calibration);
    total_hx += calibration->size();
  }

  for (const auto &it : state->_calib_GLOBALtoANCHORS) {
    if (!it.second->fixed()) {
      state_anc_id.push_back(it.second->anchor_id());
      map_hx.insert({it.second, total_hx});
      x_order.push_back(it.second);
      total_hx += it.second->size();
    }
  }

  //=========================================================================
  //=========================================================================

  // Retrive what we need for the Jacobian
  Eigen::Matrix3d R_GtoI = clone_I->Rot();
  Eigen::Vector3d p_IinG = clone_I->pos();
  Eigen::Vector3d p_IinU = calibration->value();

  // Allocate our residual, corrected measurement and Jacobians and predicted measurment (H_x_y = derivative of x wrt y)
  Eigen::MatrixXd H_n = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd H_z_I = Eigen::MatrixXd::Zero(3, 6);
  Eigen::MatrixXd H_I = Eigen::MatrixXd::Zero(measurement->uwb_ranges.size(), 6);
  Eigen::MatrixXd H_z_cal = Eigen::MatrixXd::Zero(measurement->uwb_ranges.size(), 3);
  Eigen::MatrixXd H_z_anc = Eigen::MatrixXd::Zero(measurement->uwb_ranges.size(), 5 * state_anc_id.size());
  res = Eigen::VectorXd::Zero(measurement->uwb_ranges.size());

  //=========================================================================
  //=========================================================================

  // Allocate Jacobians
  H_x = Eigen::MatrixXd::Zero(measurement->uwb_ranges.size(), total_hx);

  // Init index counter
  int idx = 0;

  // Iterate through every measurement
  for (const auto &it_range : measurement->uwb_ranges) {

    // Check there exist a correspondence in Id between measurment and anchors
    AnchorData anchor;
    try {
      anchor = state->_calib_GLOBALtoANCHORS.at(it_range.first)->anchor();
    } catch (const std::out_of_range &oor) {
      PRINT_DEBUG(RED "[UWB Update] No anchor found for the given measurement ID %d" RESET, it_range.first);
      continue;
    }

    // Compute the residual
    // Alessandro 2023: here anchor.dist_bias is alpha and (1 + anchor.dist_bias) is beta!!! Pay attention to it!
    res(idx) = it_range.second -
               ((1 + anchor.dist_bias) * ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm()) + anchor.const_bias);

    // DEBUG
    PRINT_DEBUG(YELLOW "Range measurement from anchor %d = %lf\n" RESET, anchor.id, it_range.second);
    PRINT_DEBUG(YELLOW "Predicted measurement from anchor %d = %lf\n" RESET, anchor.id,
                ((1 + anchor.dist_bias) * ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm()) + anchor.const_bias));
    PRINT_DEBUG(YELLOW "Residual for anchor %d = %lf\n" RESET, anchor.id, res(idx));

    // Compute Jacobian blocks (keep the order of variables defined in state, rotation first then translation)
    H_n.noalias() = ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).transpose()) /
                    (anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();
    H_z_I.block(0, 0, 3, 3).noalias() = R_GtoI.transpose() * ov_core::skew_x(-p_IinU);
    H_z_I.block(0, 3, 3, 3).noalias() = -Eigen::Matrix3d::Identity();
    H_I.block(idx, 0, 1, 6).noalias() = (1 + anchor.dist_bias) * H_n * H_z_I;

    // Compute Jacobian wrt calibration state
    if (state->_options.do_calib_uwb_extrinsics) {
      H_z_cal.block(idx, 0, 1, 3).noalias() = (1 + anchor.dist_bias) * H_n * R_GtoI.transpose();
    }

    // Compute Jacobian wrt anchor (p_AinU, const_bias, dist_bias)
    auto it = std::find(state_anc_id.begin(), state_anc_id.end(), anchor.id);
    if (it != state_anc_id.end()) {
      size_t i = std::distance(state_anc_id.begin(), it);
      H_z_anc.block(idx, i * 5, 1, 3).noalias() = (1 + anchor.dist_bias) * H_n * R_GtoI.transpose();
      H_z_anc(idx, i * 5 + 3) = 1;
      H_z_anc(idx, i * 5 + 4) = (anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();
    }

    // Increment the counter
    ++idx;
  }

  // CHAINRULE: get state Jacobian
  H_x.block(0, map_hx[clone_I], measurement->uwb_ranges.size(), clone_I->size()).noalias() = H_I;

  if (state->_options.do_calib_uwb_extrinsics) {
    H_x.block(0, map_hx[calibration], measurement->uwb_ranges.size(), calibration->size()).noalias() = H_z_cal;
  }

  for (const auto &it : state->_calib_GLOBALtoANCHORS) {
    if (!it.second->fixed()) {
      auto index = std::find(state_anc_id.begin(), state_anc_id.end(), it.second->anchor_id());
      size_t i = std::distance(state_anc_id.begin(), index);
      H_x.block(0, map_hx[it.second], measurement->uwb_ranges.size(), it.second->size()).noalias() = H_z_anc.block(0, i * 5, 4, 5);
    }
  }
}

void UVioUpdaterHelper::get_uwb_jacobian_single(std::shared_ptr<UVioState> state, const double timestamp, const size_t anchor_id, const double range,
                                                Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type> > &x_order) {

  // Check there exist a correspondence in Id between measurment and anchors
  std::shared_ptr<UWB_anchor> anchor_ptr;
  try {
    anchor_ptr = state->_calib_GLOBALtoANCHORS.at(anchor_id);
  } catch (const std::out_of_range &oor) {
    PRINT_DEBUG(RED "[UWB Update] No anchor found for the given measurement ID %d" RESET, anchor_id);
    return;
  }

  // Compute the size of the states involved with this feature
  int total_hx = 0;
  std::unordered_map<std::shared_ptr<ov_type::Type>, size_t> map_hx;

  // Add state clone
  std::shared_ptr<ov_type::PoseJPL> clone_I = state->_state->_imu->pose();
  map_hx.insert({clone_I, total_hx});
  x_order.push_back(clone_I);
  total_hx += clone_I->size();

  // Add extrinsics
  std::shared_ptr<ov_type::Vec> calibration = state->_calib_UWBtoIMU;
  if (state->_options.do_calib_uwb_extrinsics) {
    map_hx.insert({calibration, total_hx});
    x_order.push_back(calibration);
    total_hx += calibration->size();
  }

  // Add anchor
  if (!anchor_ptr->fixed()) {
    map_hx.insert({anchor_ptr, total_hx});
    x_order.push_back(anchor_ptr);
    total_hx += anchor_ptr->size();
  }

  //=========================================================================
  //=========================================================================

  // Retrive what we need for the Jacobian
  Eigen::Matrix3d R_GtoI = clone_I->Rot();
  Eigen::Vector3d p_IinG = clone_I->pos();
  Eigen::Vector3d p_IinU = calibration->value();
  AnchorData anchor = anchor_ptr->anchor();

  // Allocate our residual, corrected measurement and Jacobians and predicted measurment (H_x_y = derivative of x wrt y)
  Eigen::MatrixXd H_n = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd H_z_I = Eigen::MatrixXd::Zero(3, 6);
  Eigen::MatrixXd H_I = Eigen::MatrixXd::Zero(1, 6);
  Eigen::MatrixXd H_z_cal = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd H_z_anc = Eigen::MatrixXd::Zero(1, 5);
  res = Eigen::VectorXd::Zero(1);

  //=========================================================================
  //=========================================================================

  // Allocate Jacobians
  H_x = Eigen::MatrixXd::Zero(1, total_hx);

  // Compute the residual
  // Alessandro 2023: here anchor.dist_bias is alpha and (1 + anchor.dist_bias) is beta!!! Pay attention to it!
  res(0) = range -
      ((1 + anchor.dist_bias) * ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm()) + anchor.const_bias);

  // DEBUG
  PRINT_DEBUG(YELLOW "Range measurement from anchor %d = %lf\n" RESET, anchor.id, range);
  PRINT_DEBUG(YELLOW "Predicted measurement from anchor %d = %lf\n" RESET, anchor.id,
              ((1 + anchor.dist_bias) * ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm()) + anchor.const_bias));
  PRINT_DEBUG(YELLOW "Residual for anchor %d = %lf\n" RESET, anchor.id, res(0));

  // Compute Jacobian blocks (keep the order of variables defined in state, rotation first then translation)
  H_n.noalias() = ((anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).transpose()) /
      (anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();
  H_z_I.block(0, 0, 3, 3).noalias() = R_GtoI.transpose() * ov_core::skew_x(-p_IinU);
  H_z_I.block(0, 3, 3, 3).noalias() = -Eigen::Matrix3d::Identity();
  H_I.noalias() = (1 + anchor.dist_bias) * H_n * H_z_I;

  // CHAINRULE: get state Jacobian
  H_x.block(0, map_hx[clone_I], 1, clone_I->size()).noalias() = H_I;

  // Compute Jacobian wrt calibration state
  if (state->_options.do_calib_uwb_extrinsics) {
    H_z_cal.noalias() = (1 + anchor.dist_bias) * H_n * R_GtoI.transpose();
    H_x.block(0, map_hx[calibration], 1, calibration->size()).noalias() = H_z_cal;
  }

  // Compute Jacobian wrt anchor (p_AinU, const_bias, dist_bias)
  if (!anchor_ptr->fixed()) {
    H_z_anc.block(0, 0, 1, 3).noalias() = (1 + anchor.dist_bias) * H_n * R_GtoI.transpose();
    H_z_anc(0, 3) = 1;
    H_z_anc(0, 4) = (anchor.p_AinG - (R_GtoI.transpose() * (-p_IinU) + p_IinG)).norm();
    H_x.block(0, map_hx[anchor_ptr], 1, anchor_ptr->size()).noalias() = H_z_anc;
  }
}
