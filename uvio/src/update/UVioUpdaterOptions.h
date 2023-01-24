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

#ifndef UVIO_UPDATER_OPTIONS_H
#define UVIO_UPDATER_OPTIONS_H

#include <memory>
#include "utils/opencv_yaml_parse.h"
#include "update/UpdaterOptions.h"

namespace uvio {

/**
     * @brief Struct which stores general updater options
     */
struct UVioUpdaterOptions : public ov_msckf::UpdaterOptions {

  /// Noise sigma for our uwb range measurement
  double uwb_sigma_range = 1.0;

  /// Uwb chi2 multiplier
  double uwb_chi2_multipler = 1.0;

  /// Nice print function of what parameters we have loaded
  void print_and_load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_external("config_uwb", "tag0", "uwb_sigma_range", uwb_sigma_range);
      parser->parse_external("config_uwb", "tag0", "uwb_chi2_multipler", uwb_chi2_multipler);
    }
    PRINT_DEBUG("  - uwb_sigma_range: %.2f\n", uwb_sigma_range);
    PRINT_DEBUG("  - uwb_chi2_multipler: %.2f\n\n", uwb_chi2_multipler);
  }

};


}

#endif //UVIO_UPDATER_OPTIONS_H
