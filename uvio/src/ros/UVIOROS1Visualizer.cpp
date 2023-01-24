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

  // Re-assign imu subscriber
  std::string topic_imu;
  _nh->param<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  sub_imu = _nh->subscribe(topic_imu, 1000, &UVIOROS1Visualizer::callback_inertial, this);

  std::string topic_uwb;
  _nh->param<std::string>("topic_uwb", topic_uwb, "/uwb");
  parser->parse_external("config_uwb", "tag0", "rostopic", topic_uwb);
  _sub_uwb = _nh->subscribe(topic_uwb, 1, &UVIOROS1Visualizer::callback_uwb, this);
  PRINT_DEBUG("subscribing to uwb: %s\n", topic_uwb.c_str());
}

void UVIOROS1Visualizer::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);

  // If the processing queue is currently active / running just return so we can keep getting measurements
  // Otherwise create a second thread to do our update in an async manor
  // The visualization of the state, images, and features will be synchronous with the update!
  if (thread_update_running)
    return;
  thread_update_running = true;
  std::thread thread([&] {
    // Lock on the queue (prevents new images from appending)
    std::lock_guard<std::mutex> lck(camera_queue_mtx);

    // Count how many unique image streams
    std::map<int, bool> unique_cam_ids;
    for (const auto &cam_msg : camera_queue) {
      unique_cam_ids[cam_msg.sensor_ids.at(0)] = true;
    }

    // If we do not have enough unique cameras then we need to wait
    // We should wait till we have one of each camera to ensure we propagate in the correct order
    auto params = _app->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() == num_unique_cameras) {

      // Loop through our queue and see if we are able to process any of our camera measurements
      // We are able to process if we have at least one IMU measurement greater than the camera time
      double timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
        auto rT0_1 = boost::posix_time::microsec_clock::local_time();
        double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);
        _app->feed_measurement_camera(camera_queue.at(0));
        visualize();
        camera_queue.pop_front();
        auto rT0_2 = boost::posix_time::microsec_clock::local_time();
        double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
        PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);
      }
    }
    thread_update_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
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
