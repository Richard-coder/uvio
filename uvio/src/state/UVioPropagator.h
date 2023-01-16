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

#ifndef UVIO_STATE_PROPAGATOR_H
#define UVIO_STATE_PROPAGATOR_H

#include "state/Propagator.h"
#include "state/UVioState.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace uvio {

/**
 * @brief Performs the state covariance and mean propagation using imu measurements
 *
 * We will first select what measurements we need to propagate with.
 * We then compute the state transition matrix at each step and update the state and covariance.
 * For derivations look at @ref propagation page which has detailed equations.
 */
class UVioPropagator : public ov_msckf::Propagator {

public:

  using ov_msckf::Propagator::Propagator;

  ~UVioPropagator() {}

  /**
   * @brief Propagate state up to given timestamp
   *
   * This will first collect all imu readings that occured between the
   * *current* state time and the new time we want the state to be at.
   * If we don't have any imu readings we will try to extrapolate into the future.
   *
   * @param state Pointer to state
   * @param timestamp Time
   */
  void propagate(std::shared_ptr<UVioState> state, double timestamp);

};

}

#endif // UVIO_STATE_PROPAGATOR_H
