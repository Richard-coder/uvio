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

#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <core/UVioManager.h>
#include <core/VioManagerOptions.h>
#include <utils/dataset_reader.h>
#include <ros/UVIOROS1Visualizer.h>

namespace uvio {

class NodeletUVIO : public nodelet::Nodelet
{

private:

  /// Init
  virtual void onInit();

  /// Node handlers
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;

  /// Manager
  std::shared_ptr<UVioManager> sys_;

  /// Visualizer
  std::shared_ptr<UVIOROS1Visualizer> viz_;

  /// Config path
  std::string config_path_ = "";

  /// Booleans
  bool tcp_no_delay_ = false;

};

void NodeletUVIO::onInit()
{

  NODELET_INFO("Initializing UVIO nodelet...");

  // Node handlers
  nh_ = std::make_shared<ros::NodeHandle>(getNodeHandle());
  private_nh_ = std::make_shared<ros::NodeHandle>(getPrivateNodeHandle());

  // Load tcpNoDelay option
  private_nh_->param<bool>("tcp_no_delay", tcp_no_delay_, tcp_no_delay_);

  // Set tcpNoDelay if required
  if (tcp_no_delay_) {
    ros::TransportHints().tcpNoDelay();
  }

  // Load config path
  private_nh_->param<std::string>("config_path", config_path_, config_path_);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path_);
  parser->set_node_handler(private_nh_);

  // Verbosity
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys_ = std::make_shared<UVioManager>(params);
  viz_ = std::make_shared<UVIOROS1Visualizer>(private_nh_, sys_);
  viz_->setup_subscribers(parser);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvio::NodeletUVIO, nodelet::Nodelet)
