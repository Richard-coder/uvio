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

#include <ros/UVIOROS1Visualizer.h>
#include <utils/opencv_yaml_parse.h>
#include <utils/utils.h>
#include <utils/uvio_sensor_data.h>

using namespace uvio;

UVIOROS1Visualizer::UVIOROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<UVioManager> app,
                                       std::shared_ptr<ov_msckf::Simulator> sim)
    : ov_msckf::ROS1Visualizer(nh, std::static_pointer_cast<ov_msckf::VioManager>(app), sim), _app(app) {}

void UVIOROS1Visualizer::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {
  ov_msckf::ROS1Visualizer::setup_subscribers(parser);

  std::string topic_uwb;
  _nh->param<std::string>("topic_uwb", topic_uwb, "/uwb");
  parser->parse_external("config_uwb", "tag0", "rostopic", topic_uwb);
  _sub_uwb = _nh->subscribe(topic_uwb, 1, &UVIOROS1Visualizer::callback_uwb, this);
  PRINT_DEBUG("subscribing to uwb: %s\n", topic_uwb.c_str());
}

#if UWB_DRIVER == EVB_DRIVER
void UVIOROS1Visualizer::callback_uwb(const evb1000_driver::TagDistanceConstPtr &msg_uwb) {

  // Convert measurement to correct format
  UwbData message;
  message.timestamp = msg_uwb->header.stamp.toSec();

  // Get the number of measurements
  int n = msg_uwb->valid.size();

  // Fill the map with only valid ranges
  for (int i = 0; i < n; ++i) {
    if (msg_uwb->valid[i]) {
      message.uwb_ranges.insert({i, msg_uwb->distance[i]});
    }
  }

  // send it to system
  _app->feed_measurement_uwb(message);
}
#else
void UVIOROS1Visualizer::callback_uwb(const mdek_uwb_driver::UwbConstPtr &msg_uwb) {
  UwbData message;
  message.timestamp = msg_uwb->header.stamp.toSec();

  // Check if we have measurements
  if (!msg_uwb->ranges.empty()) {
    for (const auto &it : msg_uwb->ranges) {
      // Check if the id is valid (contains only numbers)
      if (!containsChar(it.id)) {
        // Convert string id to size_t
        std::stringstream sstream(it.id);
        size_t id;
        sstream >> id;
        // Populate map
        message.uwb_ranges.insert({id, it.distance});
      } else {
        ROS_WARN("Received UWB message with characters within the id field");
      }
    }
  }

  // send it to system
  _app->feed_measurement_uwb(message);
}
#endif
