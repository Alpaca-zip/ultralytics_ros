# ultralytics_ros [![](https://img.shields.io/badge/ROS2-humble-important?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/humble-devel) [![](https://img.shields.io/badge/ROS-noetic-blue?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/noetic-devel) [![](https://img.shields.io/badge/ROS-melodic-blueviolet?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/melodic-devel)
ROS2 package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

![yolo](https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a)

## Setup
```
$ cd ~/colcon_ws/src
$ git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/colcon_ws
$ rosdep install -r -y -i --from-paths .
$ colcon build
```
## Usage
### Run tracker_node
```
$ ros2 launch ultralytics_ros tracker.launch.xml debug:=true
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
- `max_det`: Maximum number of boxes to keep after NMS.
- `tracker`: Tracking algorithms.
- `classes`: List of class indices to consider.  
See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
- `debug`:  If true, run simple viewer for output topic.
- `debug_conf`:  Whether to plot the detection confidence score.
- `debug_line_width`: Line width of the bounding boxes.
- `debug_font_size`: Font size of the text.
- `debug_labels`: Font to use for the text.
- `debug_font`: Whether to plot the label of bounding boxes.
- `debug_boxes`: Whether to plot the bounding boxes.

## Docker with KITTI datasets ?ê≥

Release soon.
