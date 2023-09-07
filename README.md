# ultralytics_ros [![ROS-noetic Docker Build Check](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/docker-build-check-bot.yml/badge.svg?event=pull_request)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/docker-build-check-bot.yml)
ROS package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a" width="800px">
<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/158e7f0c-a823-4425-908d-1b63c11d6e51" width="800px">

## Setup
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
## Usage
### Run tracker_node
```
$ roslaunch ultralytics_ros tracker.launch debug:=true
```
### Run tracker_node & tracker_with_cloud_node
```
$ roslaunch ultralytics_ros tracker_with_cloud.launch debug:=true
```
### Params
- tracker_node
  - `yolo_model`: Pre-trained Weights.  
  For yolov8, you can choose `yolov8n.pt`, `yolov8s.pt`, `yolov8m.pt`, `yolov8l.pt`, `yolov8x.pt`.  
  See also: https://docs.ultralytics.com/models/
  - `image_topic`: Topic name for image. (sensor_msgs/Image)
  - `detection_topic`: Topic name for 2D bounding box. (vision_msgs/Detection2DArray)
  - `conf_thres`: Confidence threshold below which boxes will be filtered out.
  - `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
  - `max_det`: Maximum number of boxes to keep after NMS.
  - `tracker`: Tracking algorithms.
  - `classes`: List of class indices to consider.  
  See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
  - `debug`:  If true, run simple viewer.
  - `debug_conf`:  Whether to plot the detection confidence score.
  - `debug_line_width`: Line width of the bounding boxes.
  - `debug_font_size`: Font size of the text.
  - `debug_labels`: Font to use for the text.
  - `debug_font`: Whether to plot the label of bounding boxes.
  - `debug_boxes`: Whether to plot the bounding boxes.

- tracker_with_cloud_node
  - `camera_info_topic`: Topic name for camera_info. (sensor_msgs/CameraInfo)
  - `lidar_topic`: Topic name for lidar. (sensor_msgs/PointCloud2)
  - `detection3d_topic`: Topic name for 3D bounding box. (vision_msgs/Detection3DArray)
  - `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
  - `min_cluster_size`: Minimum number of points that a cluster needs to contain.
  - `max_cluster_size`: Maximum number of points that a cluster needs to contain.
## Docker with KITTI datasets 🐳
[![dockeri.co](https://dockerico.blankenship.io/image/alpacazip/ultralytics_ros)](https://hub.docker.com/r/alpacazip/ultralytics_ros)

### Docker Pull & Run
```
$ docker pull alpacazip/ultralytics_ros:noetic
$ docker run -p 6080:80 --shm-size=512m alpacazip/ultralytics_ros:noetic
```

### Run tracker_node & tracker_with_cloud_node
```
$ roslaunch ultralytics_ros kitti_tracker_with_cloud.launch
$ rosbag play kitti_2011_09_26_drive_0106_synced.bag --clock --loop
```
