# ultralytics_ros [![](https://img.shields.io/badge/ROS-noetic-green?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/noetic-devel)
ROS package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

![yolo](https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/82e098fa-e652-4b1f-a036-337571a76a4b)

## Setup
```
$ cd ~/catkin_ws/src
$ https://github.com/Alpaca-zip/ultralytics_ros.git
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
