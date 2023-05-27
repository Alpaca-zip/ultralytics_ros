#!/usr/bin/env python3

import rospy
import ros_numpy
import roslib.packages
import numpy as np
import torch

from sensor_msgs.msg import Image
from ultralytics import YOLO
from ultralytics.yolo.data.augment import LetterBox
from ultralytics.yolo.utils import ops
from ultralytics.yolo.utils.plotting import Annotator, colors


class YoloNode:
    def __init__(self):
        yolo_model = rospy.get_param("~yolo_model", "yolov8n.pt")
        publish_rate = rospy.get_param("~publish_rate", 10)
        input_topic = rospy.get_param("~input_topic", "image_raw")
        output_topic = rospy.get_param("~output_topic", "result_image")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.classes = rospy.get_param("~classes", None)
        self.agnostic = rospy.get_param("~agnostic", False)
        self.max_det = rospy.get_param("~max_det", 300)
        self.line_width = rospy.get_param("~line_width", 1)
        path = roslib.packages.get_pkg_dir("ultralytics_ros")
        self.model = YOLO(f"{path}/models/{yolo_model}")
        self.result_msg = None
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate), self.publish_results
        )
        self.sub = rospy.Subscriber(
            input_topic, Image, self.process_image, queue_size=1, buff_size=2**24
        )
        self.pub = rospy.Publisher(output_topic, Image, queue_size=1)

    def publish_results(self, event):
        if self.result_msg is not None:
            self.pub.publish(self.result_msg)

    def process_image(self, msg):
        original_image = ros_numpy.numpify(msg)
        processed_image = self.preprocess_image(original_image)
        outputs = self.model.model(processed_image, augment=False)
        processed_outputs = self.postprocess_outputs(
            outputs, processed_image, original_image
        )
        result_image = self.draw_bounding_boxes(
            processed_outputs[0], original_image, self.model.names
        )
        self.result_msg = ros_numpy.msgify(Image, result_image, encoding="bgr8")

    def preprocess_image(self, img, size=640):
        img = LetterBox(size, True)(image=img)
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img)
        img = img.float()
        img /= 255
        return img.unsqueeze(0)

    def postprocess_outputs(self, preds, img, original_img):
        preds = ops.non_max_suppression(
            preds,
            self.conf_thres,
            self.iou_thres,
            self.classes,
            self.agnostic,
            max_det=self.max_det,
        )
        for pred in preds:
            shape = original_img.shape
            pred[:, :4] = ops.scale_boxes(img.shape[2:], pred[:, :4], shape).round()
        return preds

    def draw_bounding_boxes(self, pred, original_img, class_ids):
        annotator = Annotator(
            original_img, line_width=self.line_width, example=str(class_ids)
        )
        for *xyxy, conf, cls in reversed(pred):
            c = int(cls)
            label = f"{class_ids[c]} {conf:.2f}"
            annotator.box_label(xyxy, label, color=colors(c, True))
        return original_img


if __name__ == "__main__":
    rospy.init_node("yolo_node")
    node = YoloNode()
    rospy.spin()
