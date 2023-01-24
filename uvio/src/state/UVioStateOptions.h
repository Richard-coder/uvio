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

#ifndef UVIO_STATE_OPTIONS_H
#define UVIO_STATE_OPTIONS_H

#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

namespace uvio {

/**
 * @brief Struct which stores uvio specific options
 */
struct UVioStateOptions {

  /// Bool to determine whether or not to calibrate imu-to-uwb module position
  bool do_calib_uwb_position = false;

  /// Bool to determine whether or not to calibrate uwb anchors global position
  bool do_calib_uwb_anchors = false;

  /// Prior covariances
  double prior_uwb_imu_cov = 0.1;

  /// Nice print function of what parameters we have loaded
  void print_and_load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {

    if (parser != nullptr) {
      parser->parse_external("config_uwb", "tag0", "calib_uwb_extrinsics", do_calib_uwb_position);
      parser->parse_external("config_uwb", "tag0", "calib_uwb_anchors", do_calib_uwb_anchors);
      parser->parse_external("config_uwb", "tag0", "prior_uwb_imu_cov", prior_uwb_imu_cov);
    }
    PRINT_DEBUG("    - calib_uwb_extrinsics: %s\n", do_calib_uwb_position ? "true" : "false");
    PRINT_DEBUG("    - calib_uwb_anchors: %s\n", do_calib_uwb_anchors ? "true" : "false");
    PRINT_DEBUG("    - prior_uwb_imu_cov: %.4f\n", prior_uwb_imu_cov);
  }
};

} // namespace uvio

#endif // UVIO_STATE_OPTIONS_H
