# ultralytics_ros [![ROS-noetic Industrial CI](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-ci.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-ci.yml) [![ROS-noetic Docker Build Check](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-docker-build-check.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-docker-build-check.yml)
ROS package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

|  `tracker_node`  |  `tracker_with_cloud_node`  |
| :------------: | :-----------------------: |
| <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a" width="450px"> | <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/158e7f0c-a823-4425-908d-1b63c11d6e51" width="450px"> |

- The `tracker_node` provides real-time object detection on incoming ROS image messages using the Ultralytics YOLO model.
- The `tracker_with_cloud_node` provides functionality for 3D object detection by integrating 2D detections, LiDAR data, and camera information.

## Setup ⚙
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
## Run 🚀
**`tracker_node`**
```
$ roslaunch ultralytics_ros tracker.launch debug:=true
```
**`tracker_node` & `tracker_with_cloud_node`**
```
$ roslaunch ultralytics_ros tracker_with_cloud.launch debug:=true
```
## `tracker_node`
### Params
- `yolo_model`: Pre-trained Weights.  
For yolov8, you can choose `yolov8*.pt`, `yolov8*-seg.pt`, `yolov8*-pose.pt`.

  |  YOLOv8  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/08770080-bf20-470b-8269-eee7a7c41acc" width="350px">  |
  | :-------------: | :-------------: |
  |  **YOLOv8-seg**  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/7bb6650c-769d-41c1-86f7-39fcbf01bc7c" width="350px">  |
  |  **YOLOv8-pose**  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/46d2a5ef-193b-4f83-a0b3-6cc0d5a3756c" width="350px">  |
  
  See also: https://docs.ultralytics.com/models/
- `image_topic`: Topic name for image.
- `detection_topic`: Topic name for 2D bounding box.
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
### Topics
- Subscribed Topics:
  - Image data from `image_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- Published Topics:
  - Debug images to `/debug_image` topic. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
  - Detected objects(2D bounding box) to `detection_topic` parameter. ([vision_msgs/Detection2DArray](http://docs.ros.org/en/melodic/api/vision_msgs/html/msg/Detection2DArray.html))
## `tracker_with_cloud_node`
### Params
- `camera_info_topic`: Topic name for camera info.
- `lidar_topic`: Topic name for lidar.
- `detection2d_topic`: Topic name for 2D bounding box.
- `detection3d_topic`: Topic name for 3D bounding box.
- `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
- `min_cluster_size`: Minimum number of points that a cluster needs to contain.
- `max_cluster_size`: Maximum number of points that a cluster needs to contain.
### Topics
- Subscribed Topics:
  - Camera info from `camera_info_topic` parameter. ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
  - Lidar data from `lidar_topic` parameter. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(2D bounding box) from `detection2d_topic` parameter. ([vision_msgs/Detection2DArray](http://docs.ros.org/en/melodic/api/vision_msgs/html/msg/Detection2DArray.html))
- Published Topics:
  - Detected cloud points to `/detection_cloud` topic. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(3D bounding box) to `detection3d_topic` parameter. ([vision_msgs/Detection3DArray](http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html))
  - Visualization markers to `/detection_marker` topic. ([visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html))
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
