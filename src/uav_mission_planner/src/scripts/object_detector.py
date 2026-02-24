#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ObjectDetector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('shape_detector')
        # Create a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    # This function converts ROS image messages to OpenCV images
    def imgmsg_to_cv2(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return None
        return cv_image

    # This function processes the images and returns the contours
    def process_image(cv_image):
        # Convert to grayscale and blur
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Threshold the image
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    # This function detects an 'H' shape and a rectangle frame from the contours
    def detect_shapes(contours):
        shape_contours = {'H': [], 'rectangle': []}
        for cnt in contours:
            # Approximate the contour to a polygon
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            # If the polygon has 4 vertices, it could be a rectangle or 'H' shape
            if len(approx) == 4:
                # Check if the shape is a rectangle by comparing the width and height
                _, _, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if aspect_ratio > 0.8 and aspect_ratio < 1.2:
                    shape_contours['rectangle'].append(approx)
            elif len(approx) == 12:  # Example: 'H' shape might be approximated to 12 vertices
                shape_contours['H'].append(approx)
        return shape_contours

    # This function draws the detected shapes on the image
    def draw_shapes(cv_image, shape_contours):
        for shape in shape_contours:
            for cnt in shape_contours[shape]:
                cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.putText(cv_image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Define the image callback function
    def image_callback(self, msg):
        cv_image = self.imgmsg_to_cv2(msg)
        if cv_image is not None:
            contours = self.process_image(cv_image)
            shape_contours = self.detect_shapes(contours)
            self.draw_shapes(cv_image, shape_contours)
            cv2.imshow("Image Window", cv_image)
            cv2.waitKey(3)

def main():
    try:
        od = ObjectDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()