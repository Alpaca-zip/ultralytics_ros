#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class TrackerWithCloudNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string camera_info_topic_;
  std::string lidar_topic_;
  std::string detection2d_topic_;
  std::string detection3d_topic_;
  ros::Publisher detection_cloud_pub_;
  ros::Publisher detection3d_pub_;
  ros::Publisher marker_pub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2,
                                                          vision_msgs::Detection2DArray>
      sensor_fusion_sync_subs_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
  message_filters::Subscriber<vision_msgs::Detection2DArray> detection2d_sub_;
  boost::shared_ptr<message_filters::Synchronizer<sensor_fusion_sync_subs_>> sensor_fusion_sync_;
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

public:
  TrackerWithCloudNode();
  void syncCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const vision_msgs::Detection2DArrayConstPtr& detections2d_msg);
  pcl::PointCloud<pcl::PointXYZ> msg2TransformedCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  std::tuple<vision_msgs::Detection3DArray, sensor_msgs::PointCloud2>
  projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
               const vision_msgs::Detection2DArrayConstPtr& detections2d_msg, const std_msgs::Header& header);
  pcl::PointCloud<pcl::PointXYZ> cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                        const std_msgs::Header& header);
  pcl::PointCloud<pcl::PointXYZ> euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void createBoundingBox(vision_msgs::Detection3DArray& detections3d_msg, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                         const std::vector<vision_msgs::ObjectHypothesisWithPose>& detections_results);
  visualization_msgs::MarkerArray createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg);
};
