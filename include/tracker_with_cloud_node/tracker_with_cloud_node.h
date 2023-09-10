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
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  std::string _camera_info_topic;
  std::string _lidar_topic;
  std::string _detection2d_topic;
  std::string _detection3d_topic;
  ros::Publisher _detection_cloud_pub;
  ros::Publisher _detection3d_pub;
  ros::Publisher _marker_pub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2,
                                                          vision_msgs::Detection2DArray>
      _sensor_fusion_sync_subs;
  message_filters::Subscriber<sensor_msgs::CameraInfo> _camera_info_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> _lidar_sub;
  message_filters::Subscriber<vision_msgs::Detection2DArray> _detection2d_sub;
  boost::shared_ptr<message_filters::Synchronizer<_sensor_fusion_sync_subs>> _sensor_fusion_sync;
  boost::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  image_geometry::PinholeCameraModel _cam_model;
  float _cluster_tolerance;
  int _min_cluster_size;
  int _max_cluster_size;

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
