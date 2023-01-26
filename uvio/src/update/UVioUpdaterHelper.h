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

#ifndef UVIO_UPDATER_HELPER_H
#define UVIO_UPDATER_HELPER_H

#include "update/UpdaterHelper.h"
#include "utils/uvio_sensor_data.h"
#include "state/UVioState.h"

namespace uvio {

/**
 * @brief Class that has helper functions for our updaters.
 *
 * Can compute the Jacobian for a single feature representation.
 * This will create the Jacobian based on what representation our state is in.
 * If we are using the anchor representation then we also have additional Jacobians in respect to the anchor state.
 * Also has functions such as nullspace projection and full jacobian construction.
 * For derivations look at @ref update-feat page which has detailed equations.
 *
 */
class UVioUpdaterHelper : public ov_msckf::UpdaterHelper {

public:

  /**
   * @brief Will construct the "stacked" Jacobians for uwb measurements
   *
   * @param[in] state State of the filter system
   * @param[in] uwb measurements
   * @param[out] H_x Extra Jacobians in respect to the state
   * @param[out] res Measurement residual for this feature
   * @param[out] x_order Extra variables our extra Jacobian has
   */
  static void get_uwb_jacobian_full(std::shared_ptr<UVioState> state, std::shared_ptr<UwbData> measurement, Eigen::MatrixXd &H_x,
                                    Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order);

  /**
   * @brief Will construct the "stacked" Jacobians for a single uwb measurement
   *
   * @param[in] state State of the filter system
   * @param[in] timestamp Timestamp of the measurement
   * @param[in] anchor_id ID of the anchor that made the measurement
   * @param[in] range Distance measurement
   * @param[out] H_x Extra Jacobians in respect to the state
   * @param[out] res Measurement residual for this feature
   * @param[out] x_order Extra variables our extra Jacobian has
   *
   */
  static void get_uwb_jacobian_single(std::shared_ptr<UVioState> state, const double timestamp, const size_t anchor_id, const double range,
                                      Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order);

};

}

#endif //UVIO_UPDATER_HELPER_H
