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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <ultralytics_ros/msg/yolo_result.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2,
                                                        ultralytics_ros::msg::YoloResult>
    ApproximateSyncPolicy;

class TrackerWithCloudNode : public rclcpp::Node
{
private:
  rclcpp::Time last_call_time_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  image_geometry::PinholeCameraModel cam_model_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<ultralytics_ros::msg::YoloResult> yolo_result_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string camera_info_topic_;
  std::string lidar_topic_;
  std::string yolo_result_topic_;
  std::string yolo_3d_result_topic_;
  float cluster_tolerance_;
  float voxel_leaf_size_;
  int min_cluster_size_;
  int max_cluster_size_;

public:
  TrackerWithCloudNode();
  void syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                    const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_result_msg);
  void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, const Eigen::Affine3f& transform);
  void projectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_result_msg,
                    const std_msgs::msg::Header& header, vision_msgs::msg::Detection3DArray& detection3d_array_msg,
                    sensor_msgs::msg::PointCloud2& combine_detection_cloud_msg);
  void processPointsWithBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const vision_msgs::msg::Detection2D& detection2d_msg,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw);
  void processPointsWithMask(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const sensor_msgs::msg::Image& mask_image_msg,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw);
  void createBoundingBox(vision_msgs::msg::Detection3DArray& detection3d_array_msg,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const std::vector<vision_msgs::msg::ObjectHypothesisWithPose>& detections_results_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                             const std::string& source_frame,
                                                             const std::string& target_frame,
                                                             const rclcpp::Time& stamp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  visualization_msgs::msg::MarkerArray
  createMarkerArray(const vision_msgs::msg::Detection3DArray& detection3d_array_msg, const double& duration);
};
