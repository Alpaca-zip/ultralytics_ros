# ultralytics_ros [![](https://img.shields.io/badge/ROS-noetic-blue?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/noetic-devel) [![](https://img.shields.io/badge/ROS-melodic-blueviolet?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/melodic-devel)
ROS package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

![yolo](https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a)

## Setup
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
## Usage
### Run yolo_node
```
$ roslaunch ultralytics_ros run.launch
```
### Params
- `yolo_model`: Pre-trained Weights.  
For yolov8, you can choose `yolov8n.pt`, `yolov8s.pt`, `yolov8m.pt`, `yolov8l.pt`, `yolov8x.pt`.  
See also: https://docs.ultralytics.com/models/
- `publish_rate`: Publish rate for output topic.
- `input_topic`: Topic name for input.
- `output_topic`: Topic name for output.
- `conf_thres`: Confidence threshold below which boxes will be filtered out.
- `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
- `agnostic`: If true, the model is agnostic to the number of classes, and all classes will be considered as one.
- `max_det`: Maximum number of boxes to keep after NMS.
- `line_width`: Line width of the bounding boxes. If None, it is scaled to the image size.
- `debug`:  If true, run simple viewer for output topic.
- `classes`: List of class indices to consider.  
See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml

## Docker with KITTI datasets 🐳

<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/d5ae9d18-56b6-4df9-84a5-be5835c4356a" width="600px">

### Build & Run
```
$ cd ~/catkin_ws/src/ultralytics_ros/docker
$ bash build.sh
$ bash run.sh
```
### Quick start
```
$ roslaunch ultralytics_ros run.launch input_topic:=/kitti/camera_color_left/image_raw bebug:=true
$ rosbag play kitti_2011_09_26_drive_0106_synced.bag --clock --loop
```
