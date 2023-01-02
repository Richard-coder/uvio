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

#ifndef UVIO_STATE_H
#define UVIO_STATE_H

#include "state/State.h"
#include "state/StateHelper.h"
#include "UVioStateOptions.h"
#include "types/UWB_anchor.h"

namespace uvio {

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
 * We additionally have more parameters for online estimation of calibration and SLAM features.
 * We also have the covariance of the system, which should be managed using the StateHelper class.
 */
class UVioState : public ov_msckf::State, public std::enable_shared_from_this<UVioState>
{

public:

  UVioState(UVioStateOptions &options);

  ~UVioState() {}

  /// Struct containing filter options
  UVioStateOptions _options;

  /// Calibration position for the uwb sensor (p_IinU)
  std::shared_ptr<ov_type::Vec> _calib_UWBtoIMU = std::make_shared<ov_type::Vec>(3);

  /// Positions of the uwb anchors (id, UWB_anchor)
  std::unordered_map<size_t, std::shared_ptr<UWB_anchor>> _calib_GLOBALtoANCHORS;

private:

  /// Size of state for initialization
  int _size = -1;

  /// Initialize new states
  void initialize();

};

} // namespace uvio

#endif // UVIO_STATE_H
