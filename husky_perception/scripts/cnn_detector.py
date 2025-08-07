#!/usr/bin/env python3
import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class CNNDetector:
    def __init__(self):
        rospy.init_node('cnn_detector')

        # Load YOLOv5s model from PyTorch Hub
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # Bridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Publisher for detection results (2D bounding boxes)
        self.pub = rospy.Publisher('/detections', Detection2DArray, queue_size=10)

        # Subscriber to RGB image stream from the camera
        self.sub = rospy.Subscriber('/realsense/color/image_raw', Image, self.image_callback)
        
        # Inference frequency: process 1 out of every N frames
        self.frame_skip = rospy.get_param('~frame_skip', 10)  # Default: process every 10 frames
        self.frame_count = 0

    def image_callback(self, msg):
        # Increment frame count and skip if not the right frame
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return  # Skip this frame

        try:
            # Convert ROS Image message to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLOv5 inference on the image
            results = self.model(cv_image)
            
            # Create ROS message to hold all detections
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header  # Use same timestamp and frame_id as the input image
            
            # Iterate over detections
            for *xyxy, conf, cls in results.xyxy[0].tolist():
                detection = Detection2D()

                # Calculate bounding box center and size
                detection.bbox.center.x = (xyxy[0] + xyxy[2]) / 2
                detection.bbox.center.y = (xyxy[1] + xyxy[3]) / 2
                detection.bbox.size_x = xyxy[2] - xyxy[0]
                detection.bbox.size_y = xyxy[3] - xyxy[1]
                
                # Add classification hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(cls)     # Class ID (e.g., 0 for 'person')
                hypothesis.score = conf      # Confidence score (0 to 1)
                detection.results.append(hypothesis)
                
                # Append this detection to the message
                detections_msg.detections.append(detection)
            
            # Publish detection array
            self.pub.publish(detections_msg)
            
        except Exception as e:
            # Log error if detection failed
            rospy.logerr(f"Detection error: {e}")

if __name__ == '__main__':
    node = CNNDetector()
    rospy.spin()
