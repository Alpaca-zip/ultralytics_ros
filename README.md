# ultralytics_ros [![ROS-noetic Industrial CI](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-ci.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-ci.yml) [![ROS-noetic Docker Build Check](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-docker-build-check.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/noetic-docker-build-check.yml)
ROS package for real-time object detection and segmentation using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

|  `tracker_node`  |  `tracker_with_cloud_node`  |
| :------------: | :-----------------------: |
| <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/7ccefee5-1bf9-48de-97e0-a61000bba822" width="450px"> | <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/674f352f-5171-4fcf-beb5-394aa3dfe320" height="160px"> |

- The `tracker_node` provides real-time object detection and segmentation on incoming ROS image messages using the Ultralytics YOLO model.
- The `tracker_with_cloud_node` provides functionality for 3D object detection by integrating 2D detections, mask image, LiDAR data, and camera information.

## Setup ⚙
```
$ cd ~/catkin_ws/src
$ GIT_LFS_SKIP_SMUDGE=1 git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
**NOTE**: If you want to download KITTI datasets, remove `GIT_LFS_SKIP_SMUDGE=1` from the command line.
## Run 🚀
**`tracker_node`**
```
$ roslaunch ultralytics_ros tracker.launch debug:=true
```
**`tracker_node` & `tracker_with_cloud_node`**
```
$ roslaunch ultralytics_ros tracker_with_cloud.launch debug:=true
```
**NOTE**: If the 3D bounding box is not displayed correctly, please consider using a lighter yolo model(`yolov8n.pt`) or increasing the `voxel_leaf_size`.

## `tracker_node`
### Params
- `yolo_model`: Pre-trained Weights.  
For yolov8, you can choose `yolov8*.pt`, `yolov8*-seg.pt`.

  |  YOLOv8  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/08770080-bf20-470b-8269-eee7a7c41acc" width="350px">  |
  | :-------------: | :-------------: |
  |  **YOLOv8-seg**  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/7bb6650c-769d-41c1-86f7-39fcbf01bc7c" width="350px">  |
  
  See also: https://docs.ultralytics.com/models/
- `input_topic`: Topic name for input image.
- `result_topic`: Topic name of the custom message containing the 2D bounding box and the mask image.
- `result_image_topic`: Topic name of the image on which the detection and segmentation results are plotted.
- `conf_thres`: Confidence threshold below which boxes will be filtered out.
- `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
- `max_det`: Maximum number of boxes to keep after NMS.
- `tracker`: Tracking algorithms.
- `device`: Device to run the model on(e.g. cpu or cuda:0).
  ```xml
  <arg name="device" default="cpu"/> <!-- cpu -->
  ```
  ```xml
  <arg name="device" default="0"/> <!-- cuda:0 -->
  ```
- `classes`: List of class indices to consider.
  ```xml
  <arg name="classes" default="[0, 1]"/> <!-- person, bicycle -->
  ```
  See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
- `result_conf`:  Whether to plot the detection confidence score.
- `result_line_width`: Line width of the bounding boxes.
- `result_font_size`: Font size of the text.
- `result_labels`: Font to use for the text.
- `result_font`: Whether to plot the label of bounding boxes.
- `result_boxes`: Whether to plot the bounding boxes.
### Topics
- Subscribed Topics:
  - Image data from `input_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- Published Topics:
  - Plotted images to `result_image_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
  - Detected objects(2D bounding box, mask image) to `result_topic` parameter. (ultralytics_ros/YoloResult)
    ```
    std_msgs/Header header
    vision_msgs/Detection2DArray detections
    sensor_msgs/Image[] masks
    ```
## `tracker_with_cloud_node`
### Params
- `camera_info_topic`: Topic name for camera info.
- `lidar_topic`: Topic name for lidar.
- `yolo_result_topic`: Topic name of the custom message containing the 2D bounding box and the mask image.
- `yolo_3d_result_topic`: Topic name for 3D bounding box.
- `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
- `voxel_leaf_size`: Voxel size for pointcloud downsampling.
- `min_cluster_size`: Minimum number of points that a cluster needs to contain.
- `max_cluster_size`: Maximum number of points that a cluster needs to contain.
### Topics
- Subscribed Topics:
  - Camera info from `camera_info_topic` parameter. ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
  - Lidar data from `lidar_topic` parameter. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(2D bounding box, mask image) from `yolo_result_topic` parameter. (ultralytics_ros/YoloResult)
    ```
    std_msgs/Header header
    vision_msgs/Detection2DArray detections
    sensor_msgs/Image[] masks
    ```
- Published Topics:
  - Detected cloud points to `/detection_cloud` topic. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(3D bounding box) to `yolo_3d_result_topic` parameter. ([vision_msgs/Detection3DArray](http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html))
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
$ cd ~/catkin_ws/src/ultralytics_ros/rosbag && rosbag play kitti_2011_09_26_drive_0106_synced.bag --clock --loop
```
