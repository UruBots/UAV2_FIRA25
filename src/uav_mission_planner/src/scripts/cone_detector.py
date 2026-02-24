#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import struct
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge

class ConeTrackerPointCloud:
    def __init__(self):
        rospy.init_node('cone_tracker_pointcloud', anonymous=True)

        self.bridge = CvBridge()
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_img = rospy.Publisher('/cone_image', Image, queue_size=10)
        self.pub_cords = rospy.Publisher('/cone_coords', Point, queue_size=10)

        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/zed/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_callback)
        
        self.latest_pointcloud = None
        rospy.loginfo("Nodo cone_tracker_pointcloud iniciado")

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg

    def get_xyz_from_pointcloud(self, u, v):
        """Obtiene coordenadas 3D (X,Y,Z) de un píxel usando PointCloud2."""
        if self.latest_pointcloud is None:
            return None

        width = self.latest_pointcloud.width
        height = self.latest_pointcloud.height

        if u < 0 or v < 0 or u >= width or v >= height:
            return None

        # Cada punto tiene 4 floats (X,Y,Z,RGB)
        point_step = self.latest_pointcloud.point_step
        row_step = self.latest_pointcloud.row_step

        index = v * row_step + u * point_step
        data = self.latest_pointcloud.data[index:index + 16]

        X, Y, Z, _ = struct.unpack('ffff', data)

        if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
            return (X, Y, Z)
        return None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        coords = [0.0, 0.0, 0.0]

        if contours and self.latest_pointcloud is not None:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            radius = int(radius)

            if radius > 10:
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                coords = self.get_xyz_from_pointcloud(center[0], center[1])
                if coords is not None:
                    X, Y, Z = coords
                    self.pub_cords.publish(Point(*coords))

                else:
                    self.stop_robot()
            else:
                self.stop_robot()
        else:
            self.stop_robot()

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        
    def stop_robot(self):
        twist = Twist()
        #self.vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        tracker = ConeTrackerPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
