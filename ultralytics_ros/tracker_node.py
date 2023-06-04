#!/usr/bin/env python

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("publish_rate", 10)
        self.declare_parameter("detection_topic", "detection_result")
        self.declare_parameter("input_topic", "image_raw")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("classes", list(range(80)))
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("debug", False)
        self.declare_parameter("debug_conf", True)
        self.declare_parameter("debug_line_width", 1)
        self.declare_parameter("debug_font_size", 1)
        self.declare_parameter("debug_font", "Arial.ttf")
        self.declare_parameter("debug_labels", True)
        self.declare_parameter("debug_boxes", True)
        yolo_model = self.get_parameter("yolo_model").get_parameter_value().string_value
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().integer_value
        )
        detection_topic = (
            self.get_parameter("detection_topic").get_parameter_value().string_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        self.conf_thres = (
            self.get_parameter("conf_thres").get_parameter_value().double_value
        )
        self.iou_thres = (
            self.get_parameter("iou_thres").get_parameter_value().double_value
        )
        self.classes = (
            self.get_parameter("classes").get_parameter_value().integer_array_value
        )
        self.tracker = self.get_parameter("tracker").get_parameter_value().string_value
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.debug_conf = (
            self.get_parameter("debug_conf").get_parameter_value().bool_value
        )
        self.debug_line_width = (
            self.get_parameter("debug_line_width").get_parameter_value().integer_value
        )
        self.debug_font_size = (
            self.get_parameter("debug_font_size").get_parameter_value().integer_value
        )
        self.debug_font = (
            self.get_parameter("debug_font").get_parameter_value().string_value
        )
        self.debug_labels = (
            self.get_parameter("debug_labels").get_parameter_value().bool_value
        )
        self.debug_boxes = (
            self.get_parameter("debug_boxes").get_parameter_value().bool_value
        )
        self.bridge = CvBridge()
        path = get_package_share_directory("ultralytics_ros")
        self.model = YOLO(f"{path}/models/{yolo_model}")
        self.timer_image = self.create_timer(0.1, self.publish_debug_image)
        self.timer_detection = self.create_timer(
            1.0 / publish_rate, self.publish_detection
        )
        self.header = None
        self.results = None
        self.create_subscription(Image, input_topic, self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, "debug_image", 1)
        self.detection_pub = self.create_publisher(Detection2DArray, detection_topic, 1)

    def publish_debug_image(self):
        if self.debug and self.results is not None:
            plotted_image = self.results[0].plot(
                conf=self.debug_conf,
                line_width=self.debug_line_width,
                font_size=self.debug_font_size,
                font=self.debug_font,
                labels=self.debug_labels,
                boxes=self.debug_boxes,
            )
            debug_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
            self.image_pub.publish(debug_image_msg)

    def publish_detection(self):
        if self.results is not None:
            detections_msg = Detection2DArray()
            detections_msg.header = self.header
            bounding_box = self.results[0].boxes.xywh
            classes = self.results[0].boxes.cls
            confidence_score = self.results[0].boxes.conf
            for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
                detection = Detection2D()
                detection.bbox.center.position.x = float(bbox[0])
                detection.bbox.center.position.y = float(bbox[1])
                detection.bbox.size_x = float(bbox[2])
                detection.bbox.size_y = float(bbox[3])
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.results[0].names.get(int(cls))
                hypothesis.hypothesis.score = float(conf)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)
            self.detection_pub.publish(detections_msg)

    def image_callback(self, msg):
        self.header = msg.header
        numpy_image = self.bridge.imgmsg_to_cv2(msg)
        self.results = self.model.track(
            source=numpy_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            classes=self.classes,
            tracker=self.tracker,
            verbose=False,
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
