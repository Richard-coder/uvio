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

#include <memory>

#include "core/UVioManager.h"
#include "core/UVioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "ros/UVIOROS1Visualizer.h"
#include <ros/ros.h>

using namespace uvio;

std::shared_ptr<UVioManager> sys;
std::shared_ptr<UVIOROS1Visualizer> viz;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // Launch our ros node
  ros::init(argc, argv, "run_subscribe_msckf");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);

  // Verbosity
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  UVioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys = std::make_shared<UVioManager>(params);
  viz = std::make_shared<UVIOROS1Visualizer>(nh, sys);
  viz->setup_subscribers(parser);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Spin off to ROS
  PRINT_DEBUG("done...spinning to ros\n");
  // ros::spin();
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  // Final visualization
  viz->visualize_final();
  ros::shutdown();

  // Done!
  return EXIT_SUCCESS;
}
