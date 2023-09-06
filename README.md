# ultralytics_ros
ROS package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a" width="800px">
<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/158e7f0c-a823-4425-908d-1b63c11d6e51" width="800px">

## Setup
```
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ rosdep install -r -y -i --from-paths .
$ pip install pipenv
$ cd ultralytics_ros
$ pipenv install
$ pipenv shell
$ cd ~/catkin_ws
$ catkin build
```
## Usage
### Run predict_node
```
$ roslaunch ultralytics_ros predict.launch debug:=true
```
### Run predict_node & predict_with_cloud_node
```
$ roslaunch ultralytics_ros predict_with_cloud.launch debug:=true
```
### Params
- predict_node
  - `yolo_model`: Pre-trained Weights.  
  For yolov8, you can choose `yolov8n.pt`, `yolov8s.pt`, `yolov8m.pt`, `yolov8l.pt`, `yolov8x.pt`.  
  See also: https://docs.ultralytics.com/models/
  - `image_topic`: Topic name for image. (sensor_msgs/Image)
  - `detection_topic`: Topic name for 2D bounding box. (vision_msgs/Detection2DArray)
  - `conf_thres`: Confidence threshold below which boxes will be filtered out.
  - `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
  - `max_det`: Maximum number of boxes to keep after NMS.
  - `classes`: List of class indices to consider.  
  See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
  - `debug`:  If true, run simple viewer.
  - `debug_conf`:  Whether to plot the detection confidence score.
  - `debug_line_width`: Line width of the bounding boxes.
  - `debug_font_size`: Font size of the text.
  - `debug_labels`: Font to use for the text.
  - `debug_font`: Whether to plot the label of bounding boxes.
  - `debug_boxes`: Whether to plot the bounding boxes.

- predict_with_cloud_node
  - `camera_info_topic`: Topic name for camera_info. (sensor_msgs/CameraInfo)
  - `lidar_topic`: Topic name for lidar. (sensor_msgs/PointCloud2)
  - `detection3d_topic`: Topic name for 3D bounding box. (vision_msgs/Detection3DArray)
  - `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
  - `min_cluster_size`: Minimum number of points that a cluster needs to contain.
  - `max_cluster_size`: Maximum number of points that a cluster needs to contain.

## Docker with KITTI datasets 🐳
[![dockeri.co](https://dockerico.blankenship.io/image/alpacazip/pupbot)](https://hub.docker.com/r/alpacazip/pupbot)

<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/23d5b455-cecf-4705-9e2a-6914e01cc33f" width="600px">

### Docker Pull & Run
```
$ docker pull alpacazip/ultralytics_ros:melodic
$ docker run -p 6080:80 --shm-size=512m alpacazip/ultralytics_ros:melodic
```

### Run predict_node & predict_with_cloud_node
```
$ roscd ultralytics_ros && pipenv shell
$ roslaunch ultralytics_ros kitti_predict_with_cloud.launch
$ rosbag play kitti_2011_09_26_drive_0106_synced.bag --clock --loop
```
