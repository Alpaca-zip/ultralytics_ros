#!/usr/bin/env python3

import ros_numpy
import rospy
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class TrafficLightDetectionNode:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "image_raw")
        self.debug = rospy.get_param("~debug", False)
        self.image_sub = rospy.Subscriber(
            input_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )
        self.detection_sub = rospy.Subscriber(
            "detection_result", Detection2DArray, self.detection_callback, queue_size=1
        )
        self.timer_image = rospy.Timer(rospy.Duration(0.1), self.publish_traffic_light_image)
        self.image_pub = rospy.Publisher("traffic_light_image", Image, queue_size=1)
        self.current_image = None
        self.traffic_light_image = None

    def publish_traffic_light_image(self, event):
        if self.debug and self.traffic_light_image is not None:
            image_msg = ros_numpy.msgify(Image, self.traffic_light_image, encoding="bgr8")
            self.image_pub.publish(image_msg)
    
    def image_callback(self, msg):
        self.current_image = ros_numpy.numpify(msg)

    def detection_callback(self, msg):
        cropped_image, score = self.crop_highest_score_object(msg)
        if cropped_image is not None and self.is_vertical(cropped_image):
            red_blue_images, cropped_image_with_bbox = self.extract_red_blue_area(cropped_image)
            self.traffic_light_image = cropped_image_with_bbox
            traffic_light_state = self.return_traffic_light_state(red_blue_images)
            rospy.loginfo("Score: {:.2f}, State: {}".format(score, traffic_light_state))

    def crop_highest_score_object(self, detection_msg):
        if self.current_image is not None:
            highest_score = 0.0
            highest_score_object = None
            for detection in detection_msg.detections:
                if detection.results[0].score > highest_score:
                    highest_score = detection.results[0].score
                    highest_score_object = detection
            if highest_score_object is not None:
                x = int(highest_score_object.bbox.center.x)
                y = int(highest_score_object.bbox.center.y)
                w = int(highest_score_object.bbox.size_x)
                h = int(highest_score_object.bbox.size_y)
                cropped_image = self.current_image[y-h//2:y+h//2, x-w//2:x+w//2]
                return cropped_image, highest_score
            else:
                return None, None
        else:
            return None, None
    
    def is_vertical(self, image):
        if image.shape[0] > image.shape[1]:
            return True
        else:
            return False
    
    def extract_red_blue_area(self, image):
        img_shape = image.shape
        w_c = int(img_shape[1] / 2)
        s = int(img_shape[1] / 6)
        upper_h_c = int(img_shape[0] / 4)
        lower_h_c = int(img_shape[0] * 3 / 4)
        red_blue_images = [image[upper_h_c - s:upper_h_c + s, w_c - s:w_c+s, :], image[lower_h_c - s:lower_h_c + s, w_c - s:w_c+s, :]]
        cv2.rectangle(image, (w_c - s, upper_h_c - s), (w_c + s, upper_h_c + s), (0, 255, 0), 1)
        cv2.rectangle(image, (w_c - s, lower_h_c - s), (w_c + s, lower_h_c + s), (0, 0, 255), 1)
        return red_blue_images, image

    def return_traffic_light_state(self, img_list):
        upper_img = img_list[0]
        lower_img = img_list[1]
        upper_delta = self.calculate_mean_color(upper_img)
        lower_delta = self.calculate_mean_color(lower_img)
        if upper_delta >= lower_delta:
            return 'red'
        else:
            return 'blue'
    
    def calculate_mean_color(self, image):
        red_mean = image[:,:,2].mean()
        blue_mean = image[:,:,0].mean()
        return abs(red_mean - blue_mean)


if __name__ == "__main__":
    rospy.init_node("traffic_light_detection_node")
    node = TrafficLightDetectionNode()
    rospy.spin()
