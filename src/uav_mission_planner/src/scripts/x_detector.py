#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import struct
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge

class X_detector:
    def __init__(self):
        rospy.init_node('x_detector', anonymous=True)
        self.bridge = CvBridge()

        self.pub_img = rospy.Publisher('/x_image', Image, queue_size=10)
        self.pub_cords = rospy.Publisher('/x_coords', Point, queue_size=10)

        # Suscripción a la cámara inferior
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.loginfo("Nodo x_detector iniciado con cámara inferior")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Rango para el rojo
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 | mask2

            # Filtrado morfológico
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cruz_detectada = False
            point_msg = Point()

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = float(w) / h if h != 0 else 0

                    if 0.8 < aspect_ratio < 1.2:
                        cx = int(x + w / 2)
                        cy = int(y + h / 2)

                        # Dibujar en la imagen
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                        cv2.putText(frame, "CRUZ ROJA DETECTADA", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        # Publicar coordenadas en píxeles
                        point_msg.x = cx
                        point_msg.y = cy
                        point_msg.z = 0  # sin profundidad
                        self.pub_cords.publish(point_msg)

                        cruz_detectada = True

            self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            if cruz_detectada:
                rospy.loginfo(f"Coordenadas X detectada (px): ({point_msg.x}, {point_msg.y})")

        except Exception as e:
            rospy.logerr(f"Error en image_callback: {e}")

if __name__ == '__main__':
    X_detector()
    rospy.spin()


    return frame, cruz_detectada
        
    def stop_robot(self):
        pass

if __name__ == '__main__':
    try:
        tracker = x_detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
