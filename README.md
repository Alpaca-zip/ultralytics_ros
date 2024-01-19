# ultralytics_ros [![ROS2-humble Industrial CI](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/humble-ci.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/humble-ci.yml) [![ROS2-humble Docker Build Check](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/humble-docker-build-check.yml/badge.svg)](https://github.com/Alpaca-zip/ultralytics_ros/actions/workflows/humble-docker-build-check.yml)
ROS2 package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

|  `tracker_node`  |  `tracker_with_cloud_node`  |
| :------------: | :-----------------------: |
| <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a" width="450px"> | **TODO** |

- The `tracker_node` provides real-time object detection on incoming ROS image messages using the Ultralytics YOLO model.

## Setup ⚙
```
$ cd ~/colcon_ws/src
$ GIT_LFS_SKIP_SMUDGE=1 git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/colcon_ws
$ rosdep install -r -y -i --from-paths .
$ colcon build
```
**NOTE**: If you want to download KITTI datasets, remove `GIT_LFS_SKIP_SMUDGE=1` from the command line.
## Run 🚀
**`tracker_node`**
```
$ ros2 launch ultralytics_ros tracker.launch.xml debug:=true
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
- `publish_rate`: Publish rate for `image_topic`.
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
  - Detected objects(2D bounding box) to `detection_topic` parameter. (vision_msgs/Detection2DArray)
## `tracker_with_cloud_node`
**TODO**
## Docker with KITTI datasets 🐳
[![dockeri.co](https://dockerico.blankenship.io/image/alpacazip/ultralytics_ros)](https://hub.docker.com/r/alpacazip/ultralytics_ros)

### Docker Pull & Run
```
$ docker pull alpacazip/ultralytics_ros:humble
$ docker run -p 6080:80 --shm-size=512m alpacazip/ultralytics_ros:humble
```
### Run tracker_node & tracker_with_cloud_node
```
$ ros2 launch ultralytics_ros kitti_tracker.launch.xml
$ cd ~/colcon_ws/src/ultralytics_ros/ros2bag && ros2 bag play kitti_2011_09_26_drive_0106_synced --clock --loop
```
