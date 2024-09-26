#!/usr/bin/env python3

import logging
import cv2
import torch
from ultralytics import YOLO
import math
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# For AMD ROCm
# from os import putenv
# putenv("HSA_OVERRIDE_GFX_VERSION", "10.3.0")
# For NVIDIA CUDA
torch.cuda.set_device(0)

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.detections = self.create_publisher(Image, '/yolo_detections', 10)
        self.detected_images = self.create_publisher(String, '/detected_images', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.model = YOLO(os.path.expanduser('~/arams_project//src/yolo_ros/best.pt'))

        # Suppress YOLOv8 logging
        logging.getLogger('ultralytics').setLevel(logging.ERROR)

        # Dictionary to keep track of detected objects and their last detection time
        self.detected_objects = {}
        # Set a time threshold (in seconds) for rediscovery of an object
        self.detection_interval = 10  # seconds

    def image_callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        results = self.model(frame, stream=True)

        detection_made = False
        current_time = time.time()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                confidence = math.ceil((box.conf[0] * 100)) / 100

                # Only process and publish if confidence is above 75%
                if confidence < 0.75:
                    continue

                detection_made = True

                # Pixel coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Class name
                cls = int(box.cls[0])
                class_name = r.names[cls]

                # Calculate the center of the bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Check if the object has already been detected
                if class_name in self.detected_objects:
                    prev_center_x, prev_center_y, last_detected_time = self.detected_objects[class_name]

                    # If the object was detected recently, skip further action
                    if current_time - last_detected_time < self.detection_interval:
                        continue

                # Update the detected_objects dictionary with the new position and current time
                self.detected_objects[class_name] = (center_x, center_y, current_time)

                # Print the detected object and confidence
                print(f"Detected: {class_name} with confidence {confidence}")

                # Draw the bounding box and label on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 0, 255), 1)
                org = (x1, y1 - 10 if y1 > 20 else y1 + 20)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.6
                color = (100, 0, 255)
                thickness = 1
                cv2.putText(frame, f"{class_name} {confidence}", org, font, fontScale, color, thickness)

                
        if detection_made:  # Only publish if a detection was made
            self.detections.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            # Publish the class_name as a string message
            self.detected_images.publish(String(data=class_name))


def main():
    rclpy.init()
    detection_node = DetectionNode()
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
