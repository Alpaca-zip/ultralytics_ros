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

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <ultralytics_ros/YoloResult.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2,
                                                        ultralytics_ros::YoloResult>
    ApproximateSyncPolicy;

class TrackerWithCloudNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher detection_cloud_pub_;
  ros::Publisher detection3d_pub_;
  ros::Publisher marker_pub_;
  ros::Time last_call_time_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
  message_filters::Subscriber<ultralytics_ros::YoloResult> yolo_result_sub_;
  boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
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
  void syncCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const ultralytics_ros::YoloResult::ConstPtr& yolo_result_msg);
  void projectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    const ultralytics_ros::YoloResultConstPtr& yolo_result_msg, const std_msgs::Header& header,
                    vision_msgs::Detection3DArray& detections3d_msg,
                    sensor_msgs::PointCloud2& combine_detection_cloud_msg);
  void processPointsWithBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const vision_msgs::Detection2D& detection,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw);
  void processPointsWithMask(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const sensor_msgs::Image& mask,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                             const std::string& source_frame,
                                                             const std::string& target_frame, const ros::Time& stamp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void createBoundingBox(vision_msgs::Detection3DArray& detections3d_msg,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const std::vector<vision_msgs::ObjectHypothesisWithPose>& detections_results);
  visualization_msgs::MarkerArray createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg,
                                                    const double& duration);
};
