/**
 * ultralytics_ros
 * Copyright (C) 2023-2024  Alpaca-zip
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ultralytics_ros/msg/yolo_result.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2,
                                                        ultralytics_ros::msg::YoloResult>
    ApproximateSyncPolicy;

class TrackerWithCloudNode : public rclcpp::Node
{
private:
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<ultralytics_ros::msg::YoloResult> yolo_result_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
  std::string camera_info_topic_;
  std::string lidar_topic_;
  std::string yolo_result_topic_;

public:
  TrackerWithCloudNode();
  void syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                    const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_result_msg);
};
