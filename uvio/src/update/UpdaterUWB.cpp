/*
 * Copyright (C) 2020 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * You can contact the author at <alessandro.fornasier@aau.at>
 */

#include "UpdaterUWB.h"

using namespace uvio;

void UpdaterUWB::update(std::shared_ptr<UVioState> state, const std::shared_ptr<UwbData>& message) {

  // Convert uwb measurement into our current format keeping only valid measuremetns
  UVioUpdaterHelper::UpdaterHelperUWB meas;
  meas.uwbs = *message;

  // Copy Reference on the meas structure dereferenciating the pointer
  for (const auto &it : state->_calib_GLOBALtoANCHORS) {
    meas.anchors.insert({it.first, it.second.anchor()});
  }

  // Measurement noise for the uwb update
  // Alessandro: fixed value
  Eigen::MatrixXd R = pow(_options.uwb_sigma_range,2)*Eigen::MatrixXd::Identity(message->uwb_ranges.size(),message->uwb_ranges.size());

  // Our return values (state jacobian, residual, and order of state jacobian)
  Eigen::MatrixXd H_x;
  Eigen::VectorXd res;
  std::vector<std::shared_ptr<ov_type::Type>> Hx_order;

  // Get the Jacobian for the uwb update
  UVioUpdaterHelper::get_uwb_jacobian_full(state, meas, H_x, res, Hx_order);

  // Chi2 distance check
  Eigen::MatrixXd P_marg = ov_msckf::StateHelper::get_marginal_covariance(std::static_pointer_cast<ov_msckf::State>(state), Hx_order);
  Eigen::MatrixXd S = H_x*P_marg*H_x.transpose() + R; //Innovation matrix
  double chi2 = res.dot(S.llt().solve(res)); //r^T(S^-1)r

  // Get our threshold (we precompute up to 500 but handle the case that it is more)
  double chi2_check = _chi_squared_table[res.rows()];

  // Check if we should update or not
  // Alessandro: [check] CHI2 test
  if(chi2 > _options.chi2_multipler*chi2_check) {
//      std::cout << "UWB update chi2 = " << chi2 << " > " << _options_uwb.chi2_multipler*chi2_check << std::endl;
      return;
  }

  // We are good! Perform measurement compression
  UVioUpdaterHelper::measurement_compress_inplace(H_x, res);
  if(H_x.rows() < 1) {
    return;
  }

  // Update the state
  ov_msckf::StateHelper::EKFUpdate(std::static_pointer_cast<ov_msckf::State>(state), Hx_order, H_x, res, R);

}
