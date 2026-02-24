#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar.pyzbar import decode

class CameraReader:
    def __init__(self):
        rospy.init_node('camera_read', anonymous=False)
        self.bridge = CvBridge()
        self.subscription = rospy.Subscriber('/camera_1', Image, self.callback)
        self.image_pub = rospy.Publisher('/camera_1/qrcode', Image, queue_size=10)
        self.qr_to_move = rospy.Publisher('/qrcode/raw', String, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        (rows, cols, channels) = cv_image.shape

        # TODO resize image based on the camera
        resized_image = cv2.resize(cv_image, (480, 640))

        # gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        # _, img_bw = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

        qr_result = decode(resized_image)

        #print(qr_result)

        if qr_result:
            qr_data = qr_result[0].data.decode('utf-8')
            rospy.loginfo("QR Code detected: %s", qr_data)
            (x, y, w, h) = qr_result[0].rect
            cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)
            text = "{}".format(qr_data)
            cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.qr_to_move.publish(qr_data)
        # cv2.imshow("Camera output", resized_image)
        cv2.waitKey(5)



def main():
    cr = CameraReader()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
