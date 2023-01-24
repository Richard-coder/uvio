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

#include "UVioPropagator.h"

using namespace uvio;

void UVioPropagator::propagate(std::shared_ptr<UVioState> state, double timestamp) {

  // If the difference between the current update time and state is zero
  // We should crash, as this means we would have two clones at the same time!!!!
  if (state->_state->_timestamp == timestamp) {
    PRINT_ERROR(RED "UVioPropagator::propagate(): Propagation called again at same timestep at last update timestep!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // We should crash if we are trying to propagate backwards
  if (state->_state->_timestamp > timestamp) {
    PRINT_ERROR(RED "UVioPropagator::propagate(): Propagation called trying to propagate backwards in time!!!!\n" RESET);
    PRINT_ERROR(RED "UVioPropagator::propagate(): desired propagation = %.4f\n" RESET, (timestamp - state->_state->_timestamp));
    std::exit(EXIT_FAILURE);
  }

  // First lets construct an IMU vector of measurements we need
  double time0 = state->_state->_timestamp + propagator_->last_prop_time_offset;
  double time1 = timestamp;
  std::vector<ov_core::ImuData> prop_data;
  {
    std::lock_guard<std::mutex> lck(propagator_->imu_data_mtx);
    prop_data = propagator_->select_imu_readings(propagator_->imu_data, time0, time1);
  }

  // We are going to sum up all the state transition matrices, so we can do a single large multiplication at the end
  // Phi_summed = Phi_i*Phi_summed
  // Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
  // After summing we can multiple the total phi to get the updated covariance
  // We will then add the noise to the IMU portion of the state
  Eigen::Matrix<double, 15, 15> Phi_summed = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> Qd_summed = Eigen::Matrix<double, 15, 15>::Zero();
  double dt_summed = 0;

  // Loop through all IMU messages, and use them to move the state forward in time
  // This uses the zero'th order quat, and then constant acceleration discrete
  if (prop_data.size() > 1) {
    for (size_t i = 0; i < prop_data.size() - 1; i++) {

      // Get the next state Jacobian and noise Jacobian for this IMU reading
      Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
      Eigen::Matrix<double, 15, 15> Qdi = Eigen::Matrix<double, 15, 15>::Zero();
      propagator_->predict_and_compute(state->_state, prop_data.at(i), prop_data.at(i + 1), F, Qdi);

      // Next we should propagate our IMU covariance
      // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
      // Pci' = F*Pci and Pic' = Pic*F.transpose()
      // NOTE: Here we are summing the state transition F so we can do a single mutiplication later
      // NOTE: Phi_summed = Phi_i*Phi_summed
      // NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
      Phi_summed = F * Phi_summed;
      Qd_summed = F * Qd_summed * F.transpose() + Qdi;
      Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());
      dt_summed += prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;
    }
  }
  assert(std::abs((time1 - time0) - dt_summed) < 1e-4);

  // Last angular velocity (used for cloning when estimating time offset)
  Eigen::Matrix<double, 3, 1> last_w = Eigen::Matrix<double, 3, 1>::Zero();
  if (prop_data.size() > 1)
    last_w = prop_data.at(prop_data.size() - 2).wm - state->_state->_imu->bias_g();
  else if (!prop_data.empty())
    last_w = prop_data.at(prop_data.size() - 1).wm - state->_state->_imu->bias_g();

  // Do the update to the covariance with our "summed" state transition and IMU noise addition...
  std::vector<std::shared_ptr<ov_type::Type>> Phi_order;
  Phi_order.push_back(state->_state->_imu);
  ov_msckf::StateHelper::EKFPropagation(state->_state, Phi_order, Phi_order, Phi_summed, Qd_summed);

  // Set timestamp data
  state->_state->_timestamp = timestamp;
}
