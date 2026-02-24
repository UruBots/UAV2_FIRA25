#!/usr/bin/env python
# -*- coding: utf-8 -*-
import struct
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, Point

class H_detector:
    def __init__(self):
        rospy.init_node("h_detector")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pub_img = rospy.Publisher('/h_image', Image, queue_size=10)
        self.pub_cords = rospy.Publisher('/h_coords', Point, queue_size=10)
        self.pub = rospy.Publisher('/h_pixel_coords', Point, queue_size=1)
        # self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)


        rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.image_zed_callback)
        rospy.Subscriber('/zed/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_callback)

        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 1.5  # Altura inicial

        self.center_tolerance = 30  # píxeles
        self.landing = False
        self.latest_pointcloud = None


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

    def image_zed_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filtrar azul (H de aterrizaje)

        debug = 0
        # Si debug es 1, usar tonos más claros para la cinta verde
        if debug:
        # cinta verde 
            lower_blue = np.array([40, 50, 50])   # tono, saturación, valor mínimos
            upper_blue = np.array([80, 255, 255]) # tono, saturación, valor máximos
        else:
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

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
        print("stop h-detector")
        #twist = Twist()
        pass
        #self.vel_pub.publish(twist)    

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Filtrar azul (H de aterrizaje)
        debug = 0
        if debug:
            # cinta verde 
            lower_blue = np.array([40, 50, 50])   # tono, saturación, valor mínimos
            upper_blue = np.array([80, 255, 255]) # tono, saturación, valor máximos
        else:
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Contorno más grande
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                pt = Point()
                pt.x = cx
                pt.y = cy
                pt.z = 0.0  # opcional
                pub.publish(pt)

            # Centro de la H detectada
            cx = x + w // 2
            cy = y + h // 2

            # Centro de la imagen
            img_h, img_w = frame.shape[:2]
            center_x = img_w // 2
            center_y = img_h // 2

            # Dibujar para debug
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # Control simple de centrado
            error_x = cx - center_x
            error_y = cy - center_y

            if abs(error_x) > self.center_tolerance:
                self.current_pose.pose.position.y -= error_x * 0.002  # Ajuste lateral

            if abs(error_y) > self.center_tolerance:
                self.current_pose.pose.position.x -= error_y * 0.002  # Ajuste frontal

            # Si está centrado, iniciar descenso
            if abs(error_x) <= self.center_tolerance and abs(error_y) <= self.center_tolerance:
                rospy.loginfo("H centrada → Descendiendo...")
                self.current_pose.pose.position.z -= 0.05  # Bajar poco a poco

        # Publicar pose
        # self.pose_pub.publish(self.current_pose)

        # Mostrar para debug
        # cv2.imshow("Frame", frame)
        # cv2.imshow("Mask", mask)
        # cv2.waitKey(1)

    # def run(self):
    #     rate = rospy.Rate(20)
    #     while not rospy.is_shutdown():
    #         self.pose_pub.publish(self.current_pose)
    #         rate.sleep()

if __name__ == '__main__':
    try:
        tracker = H_detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
