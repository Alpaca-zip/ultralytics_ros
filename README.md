# ultralytics_ros [![](https://img.shields.io/badge/ROS2-humble-important?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/humble-devel) [![](https://img.shields.io/badge/ROS-noetic-blue?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/noetic-devel) [![](https://img.shields.io/badge/ROS-melodic-blueviolet?style=flat-square&logo=ros)](https://github.com/Alpaca-zip/ultralytics_ros/tree/melodic-devel)
ROS2 package for real-time object detection using the Ultralytics YOLO, enabling flexible integration with various robotics applications.

![yolo](https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/9da7dbbf-5cc0-41bc-be82-d481abbf552a)

## Notice (For humble & WSL2)
`could not load library libcudnn_cnn_infer.so.8. Error: libcuda.so: cannot open shared object file: No such file or directory`

If you get an error like the above, add to the `.bashrc` file is:

`export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH`

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

## Docker with KITTI datasets 🐳

<img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/d5ae9d18-56b6-4df9-84a5-be5835c4356a" width="600px">

### Build & Run
```
$ cd ~/colcon_ws/src/ultralytics_ros/docker
$ bash build.sh
$ bash run.sh
```
### Quick start
```
$ ros2 launch ultralytics_ros tracker.launch.xml input_topic:=/kitti/camera_color_left/image_raw debug:=true
$ ros2 bag play kitti_2011_09_26_drive_0106_synced --clock --loop
```
