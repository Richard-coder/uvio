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

  /// Initialize uvio state and downcast propagator
  state = std::make_shared<UVioState>(params.uvio_state_options, this->get_state());
  propagator = std::static_pointer_cast<UVioPropagator>(this->get_propagator());

  // Initialize p_UinI (calibration uwb-imu)
  if (params.uvio_state_options.do_calib_uwb_extrinsics) {
    std::vector<std::shared_ptr<ov_type::Type>> H_order;

    // Need to fake it...
    H_order.push_back(state->_state->_imu->q());

    Eigen::Matrix3d H_R = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d H_L = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * params.uvio_state_options.prior_uwb_imu_cov;
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    ov_msckf::StateHelper::initialize_invertible(state->_state, state->_calib_UWBtoIMU, H_order, H_R, H_L, R, res);
    PRINT_INFO("Calibration uwb-imu initialized\n");
    PRINT_INFO("calib_UWtoIMU = [%.3f,%.3f,%.3f]\n", state->_calib_UWBtoIMU->value()(0), state->_calib_UWBtoIMU->value()(1),
               state->_calib_UWBtoIMU->value()(2));
  }

  // Our UWB sensor extrinsic transform
  state->_calib_UWBtoIMU->set_value(params.uwb_extrinsics);
  state->_calib_UWBtoIMU->set_fej(params.uwb_extrinsics);

  // Initialize anchors (if provided in config file)
  try_to_initialize_uwb_anchors(params.uwb_anchors);

  // Make the updater!
  updaterUWB = std::make_unique<UpdaterUWB>(params.uwb_options);
}

void UVioManager::feed_measurement_uwb(const UwbData &message) {

  // Check if vio is initialized, anchors are initialized and distace traveled is
  // greater than 0.5 meter, else return
  if (!(is_initialized_vio && are_initialized_anchors && distance > 0.5)) {
    return;
  }

  // Return if the uwb measurement is out of order otherwise feed our bar measuremnts
  if (state->_state->_timestamp >= message.timestamp) {
    PRINT_INFO(YELLOW "UWB measurements received out of order (prop dt = %3f)\n" RESET, (message.timestamp - state->_state->_timestamp));
    return;
  }

  past_measurements.insert({message.timestamp, std::make_shared<UwbData>(message)});
}

void UVioManager::try_to_initialize_uwb_anchors(const std::vector<AnchorData> &anchors) {

  // Check if argument is empty
  if (anchors.empty()) {
    PRINT_INFO("UWB anchors not initialized (anchors not provided)\n");
    return;
  }

  // Check if anchors are not initialized yet and initialize them
  if (!are_initialized_anchors) {
    params.uwb_anchors = anchors;
    params.n_anchors = anchors.size();
    int n_fixed_anchors = std::count_if(anchors.begin(), anchors.end(), [](const AnchorData &a) { return a.fix; });
    params.n_anchors_to_fix = std::max(0, params.n_anchors_to_fix - n_fixed_anchors);
    initialize_uwb_anchors();
    return;
  }

  // Check if we have new anchors to initialize
  PRINT_INFO("UWB anchors already initialized. Checking for new anchors to initialize\n");
  for (const auto &it : anchors) {
    // If anchor is not in the state, initialize it
    if (state->_calib_GLOBALtoANCHORS.find(it.id) == state->_calib_GLOBALtoANCHORS.end()) {
      PRINT_INFO(GREEN "New anchor found. Initializing anchor[%d]\n" RESET, it.id);
      params.uwb_anchors.push_back(it);
      params.n_anchors++;
      if (it.fix) {
        params.n_anchors_to_fix = std::max(0, params.n_anchors_to_fix - 1);
      }
      initialize_new_uwb_anchor(it);
    }
  }

  return;
}

void UVioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);
  if (is_initialized_vio) {
    trackDATABASE->append_new_measurements(trackFEATS->get_feature_database());
  }

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
    trackDATABASE->append_new_measurements(trackARUCO->get_feature_database());
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state->_state, message.timestamp);
    }
    if (did_zupt_update) {
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // Retrive other measurement from last update up to vision measurement time
  if (!past_measurements.empty()) {
    for (auto it = past_measurements.begin(); it != past_measurements.lower_bound(message.timestamp); it++) {
      if (it->first < message.timestamp) {
        do_uwb_propagate_update(it->second);
      }
    }

    // Clear all the past measurement and throw away also measurement which have the same timestamp of the vision measurment even if not
    // processed
    past_measurements.erase(past_measurements.begin(), past_measurements.upper_bound(message.timestamp));
  }

  // Print our current uwb state
  if (params.uvio_state_options.do_calib_uwb_extrinsics) {
    PRINT_INFO(YELLOW "calib_UWtoIMU = [%.3f,%.3f,%.3f]\n" RESET, state->_calib_UWBtoIMU->value()(0), state->_calib_UWBtoIMU->value()(1),
               state->_calib_UWBtoIMU->value()(2));
  }
  for (const auto &it : state->_calib_GLOBALtoANCHORS) {
    if (!it.second->fixed()) {
      PRINT_INFO(YELLOW "anchor[%d]: p_AinG = [%.3f, %.3f, %.3f] | const_bias = %.4f | dist_bias = %.4f\n" RESET, it.first,
                 it.second->p_AinG()->value()(0), it.second->p_AinG()->value()(1), it.second->p_AinG()->value()(2),
                 it.second->const_bias()->value()(0), it.second->dist_bias()->value()(0));
    }
  }

  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

void UVioManager::initialize_uwb_anchors() {

  // Check if anchors are initialized
  if (are_initialized_anchors) {
    PRINT_ERROR("UWB anchors already initialized\n");
    return;
  }

  // Check if we have any anchors
  if (params.uwb_anchors.empty()) {
    PRINT_ERROR("UWB anchors not specified\n");
    return;
  }

  for (const auto &it : params.uwb_anchors) {
    std::shared_ptr<UWB_anchor> anchor = std::make_shared<UWB_anchor>(it);
    state->_calib_GLOBALtoANCHORS.insert({it.id, anchor});

    PRINT_INFO("Anchor[%d] initialized\n", it.id);

    // Initialize state variable if option enabled and anchor not fixed
    if (!it.fix) {

      // Initialize state variables
      std::vector<std::shared_ptr<ov_type::Type>> H_order;

      // Need to fake it...
      H_order.push_back(state->_state->_imu->q());

      Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(5, 3);
      Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(5, 5);
      Eigen::MatrixXd R = Eigen::MatrixXd::Identity(5, 5);
      Eigen::VectorXd res = Eigen::VectorXd::Zero(5);
      ov_msckf::StateHelper::initialize_invertible(state->_state, state->_calib_GLOBALtoANCHORS.at(it.id), H_order, H_R, H_L, R, res);

      H_order.clear();
      H_order.push_back(state->_calib_GLOBALtoANCHORS.at(it.id));

      ov_msckf::StateHelper::set_initial_covariance(state->_state, it.cov, H_order);

      PRINT_INFO("Anchor[%d] added to state\n", it.id);
    }

    // Print anchor info
    PRINT_INFO("anchor[%d]: p_AinG = [%.3f, %.3f, %.3f] | const_bias = %.4f | dist_bias = %.4f\n", it.id, it.p_AinG.x(), it.p_AinG.y(),
               it.p_AinG.z(), it.const_bias, it.dist_bias);
    std::cout << "cov = \n" << it.cov << "\n" << std::endl;
  }
  are_initialized_anchors = true;
  PRINT_INFO("UWB anchors correctly initialized\n");
}

void UVioManager::initialize_new_uwb_anchor(const AnchorData &anchor) {

  // Check if anchors are initialized
  if (!are_initialized_anchors) {
    PRINT_ERROR("UWB anchors not initialized\n");
    return;
  }

  // Check if anchor is already initialized
  if (state->_calib_GLOBALtoANCHORS.find(anchor.id) != state->_calib_GLOBALtoANCHORS.end()) {
    PRINT_ERROR("Anchor[%d] already initialized\n", anchor.id);
    return;
  }

  // Initialize anchor
  std::shared_ptr<UWB_anchor> uwb_anchor = std::make_shared<UWB_anchor>(anchor);
  state->_calib_GLOBALtoANCHORS.insert({anchor.id, uwb_anchor});

  PRINT_INFO("Anchor[%d] initialized\n", anchor.id);

  // Initialize state variable if option enabled and anchor not fixed
  if (!anchor.fix) {

    // Initialize state variables
    std::vector<std::shared_ptr<ov_type::Type>> H_order;

    // Need to fake it...
    H_order.push_back(state->_state->_imu->q());

    Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(5, 3);
    Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(5, 5);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(5);
    ov_msckf::StateHelper::initialize_invertible(state->_state, state->_calib_GLOBALtoANCHORS.at(anchor.id), H_order, H_R, H_L, R, res);

    H_order.clear();
    H_order.push_back(state->_calib_GLOBALtoANCHORS.at(anchor.id));

    ov_msckf::StateHelper::set_initial_covariance(state->_state, anchor.cov, H_order);

    PRINT_INFO("Anchor[%d] added to state\n", anchor.id);
  }

  // Print anchor info
  PRINT_INFO("anchor[%d]: p_AinG = [%.3f, %.3f, %.3f] | const_bias = %.4f | dist_bias = %.4f\n", anchor.id, anchor.p_AinG.x(),
             anchor.p_AinG.y(), anchor.p_AinG.z(), anchor.const_bias, anchor.dist_bias);
  std::cout << "cov = \n" << anchor.cov << "\n" << std::endl;
}

void UVioManager::do_uwb_propagate_update(const std::shared_ptr<UwbData> &message) {

  // Check if we have at least one measurement from an initialized anchor
  bool valid = false;
  for (const auto &it : message->uwb_ranges) {
    if (state->_calib_GLOBALtoANCHORS.find(it.first) != state->_calib_GLOBALtoANCHORS.end()) {
      valid = true;
      break;
    }
  }

  // Return if no matching was found
  if (!valid) {
    return;
  }

  // Propagate the state forward to the current update time
  propagator->propagate(state->_state, message->timestamp);

  // Return if we where unable to propagate
  if (state->_state->_timestamp != message->timestamp) {
    printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET, message->timestamp - state->_state->_timestamp);
    return;
  }

  // Iterate through single ranges and update
  /// Giulio: this is better because it allows to filter single measurements
  /// with the chi2 test instead of discarding all of them if just one is bad
  for (const auto &it : message->uwb_ranges) {
    // Check if measurement is from initialized anchor
    if (state->_calib_GLOBALtoANCHORS.find(it.first) != state->_calib_GLOBALtoANCHORS.end()) {
      // EKF Update with single UWB measurement
      updaterUWB->update_single(state, message->timestamp, it.first, it.second);
    }
  }
}
