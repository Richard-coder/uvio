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

#ifndef UVIO_ROS1VISUALIZER_H
#define UVIO_ROS1VISUALIZER_H

#include <mdek_uwb_driver/Uwb.h>
#include <ros/ROS1Visualizer.h>

#include "core/UVioManager.h"

namespace uvio
{

class UVIOROS1Visualizer : public ov_msckf::ROS1Visualizer
{

public:

  /**
   * @brief Default constructor
   * @param nh ROS node handler
   * @param app Core estimator manager
   * @param sim Simulator if we are simulating
   */
  UVIOROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<UVioManager> app, std::shared_ptr<ov_msckf::Simulator> sim = nullptr);

  /**
   * @brief Wrapper to ov_msckf::ROS!Visualizer::setup_subscribers. Will setup ROS subscribers and callbacks
   * @param parser Configuration file parser
   */
  void setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser);

  /// Callback for uwb information
  void callback_uwb(const mdek_uwb_driver::UwbConstPtr &msg_uwb);

private:

  /// UWB subscriber
  ros::Subscriber _sub_uwb;

  /// Core application of the filter system
  std::shared_ptr<UVioManager> _app;

};

} // namespace uvio

#endif // UVIO_ROS1VISUALIZER_H
