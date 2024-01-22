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

#include "tracker_with_cloud_node/tracker_with_cloud_node.h"

TrackerWithCloudNode::TrackerWithCloudNode() : rclcpp::Node("tracker_with_cloud_node")
{
  this->declare_parameter<std::string>("camera_info_topic", "camera_info");
  this->declare_parameter<std::string>("lidar_topic", "points_raw");
  this->declare_parameter<std::string>("yolo_result_topic", "yolo_result");

  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("yolo_result_topic", yolo_result_topic_);
  camera_info_sub_.subscribe(this, camera_info_topic_);
  lidar_sub_.subscribe(this, lidar_topic_);
  yolo_result_sub_.subscribe(this, yolo_result_topic_);
  sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(1);
  sync_->connectInput(camera_info_sub_, lidar_sub_, yolo_result_sub_);
  sync_->registerCallback(std::bind(&TrackerWithCloudNode::syncCallback, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));
}

void TrackerWithCloudNode::syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& /*camera_info_msg*/,
                                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& /*cloud_msg*/,
                                        const ultralytics_ros::msg::YoloResult::ConstSharedPtr& /*yolo_result_msg*/)
{
  // TODO
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackerWithCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
