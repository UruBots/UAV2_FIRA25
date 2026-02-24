#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class LineFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('line_follower')
        # Create a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()
        # Subscribe to the camera image topic
        # self.image_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        #self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        # Publicador de corrección de posición
        self.correction_pub = rospy.Publisher('/line_follower/pos_correction', Float32MultiArray, queue_size=10)
    
    
    # def image_callback(self, msg: Image):
    #     try:
    #         # Convertir imagen
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         scale = 2.0
    #         image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    #         height, width, _ = image.shape

    #         # Convertir a HSV y aplicar máscara de color
    #         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #         lower_color = (100, 114, 95)
    #         upper_color = (170, 180, 180)
    #         mask = cv2.inRange(hsv, lower_color, upper_color)
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         twist = Twist()
    #         dx, dy = 0.0, 0.0  # Correcciones de posición

    #         if contours:
    #             largest_contour = max(contours, key=cv2.contourArea)
    #             M = cv2.moments(largest_contour)

    #             if M['m00'] != 0:
    #                 cx = int(M['m10'] / M['m00'])
    #                 cy = int(M['m01'] / M['m00'])

    #                 # Dibujar centroide
    #                 cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
    #                 cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

    #                 # Error horizontal
    #                 error_x = cx - width // 2

    #                 # Control proporcional
    #                 Kp_ang = 0.005
    #                 twist.linear.x = 0.1
    #                 twist.angular.z = -Kp_ang * error_x

    #                 # Calcular correcciones para el nodo de navegación
    #                 dx = -error_x * 0.001  # de píxeles a metros aprox.
    #                 dy = 0.05  # avanzar siempre un poco

    #                  # Publicar corrección de posición
    #                 corr_msg = Float32MultiArray()
    #                 corr_msg.data = [dx, dy]
    #                 print(f"seguidor de linea x={dx} y={dy}")
    #                 self.correction_pub.publish(corr_msg)
    #             else:
    #                 twist.linear.x = 0.0
    #                 twist.angular.z = 0.0
    #         else:
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0

    #         # Publicar velocidad directa (opcional)
    #         #self.cmd_vel_pub.publish(twist)

    #         # Publicar corrección de posición
    #         # corr_msg = Float32MultiArray()
    #         # corr_msg.data = [dx, dy]
    #         # self.correction_pub.publish(corr_msg)

    #         # # Mostrar imagen
    #         # cv2.imshow("Track Follower", image)
    #         # cv2.waitKey(1)

    #     except Exception as e:
    #         rospy.logerr(f"Image processing error: {e}")

    def image_callback(self, msg: Image):
        try:
            # === Parámetros ajustables ===
            scale = 2.0               # reescalado para mejorar resolución del análisis
            ROI_Y_RATIO = 0.55        # usar la franja inferior (55% hacia abajo)
            Kp_ang = 0.005            # ganancia proporcional para giro
            px_to_m = 0.001           # conversión aprox. píxel -> metro para dx
            v_forward = 0.10          # avance base
            CONNECT_DASHES = True     # conectar un poco los segmentos punteados

            # === Convertir imagen ROS -> OpenCV y escalar ===
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            height, width, _ = image.shape

            # === ROI: parte inferior de la imagen ===
            y0 = int(height * ROI_Y_RATIO)
            roi = image[y0:, :]

            # === Suavizado leve ===
            roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)

            # === HLS: detectamos "blanco" (L alta, S baja) ===
            hls = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HLS)
            # Rango típico para blanco en fondo oscuro (ajustable):
            hls_lower = (0, 200, 0)     # H no importa, L alta, S baja
            hls_upper = (180, 255, 80)
            mask = cv2.inRange(hls, np.array(hls_lower), np.array(hls_upper))

            # === Morfología: conectar un poco los trazos punteados ===
            if CONNECT_DASHES:
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))
                mask = cv2.dilate(mask, kernel, iterations=1)
            else:
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

            # Limpieza opcional
            mask = cv2.medianBlur(mask, 5)

            twist = Twist()
            dx, dy = 0.0, 0.0  # Correcciones de posición

            # === Centroide de la máscara (más robusto que solo el mayor contorno en punteado) ===
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])           # cx en coordenadas de la ROI
                cy = int(M['m01'] / M['m00'])

                # Para dibujar en la imagen completa
                cx_full = cx
                cy_full = cy + y0

                # Visual opcional (si tenés un flag self.debug)
                # if getattr(self, "debug", False):
                #     cv2.circle(image, (cx_full, cy_full), 6, (0, 0, 255), -1)
                #     cv2.line(image, (width//2, y0), (width//2, height), (0, 255, 0), 1)

                # Error horizontal respecto al centro de la imagen completa
                error_x = cx - (width // 2)

                # Control: avanzar y corregir rumbo
                twist.linear.x = v_forward
                twist.angular.z = -Kp_ang * error_x

                # Correcciones para el nodo de navegación (tu formato)
                dx = -error_x * px_to_m
                dy = 0.05  # avanzar un poco siempre

                # Publicar corrección de posición
                corr_msg = Float32MultiArray()
                corr_msg.data = [dx, dy]
                print(f"seguidor de linea x={dx:.4f} y={dy:.4f}")
                self.correction_pub.publish(corr_msg)
            # else:
            #     # Sin detección: frená (o podés mantener última consigna si preferís)
            #     twist.linear.x = 0.0
            #     twist.angular.z = 0.0

            # self.correction_pub.publish(corr_msg)

        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

    
    # def image_callback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridge.CvBridgeError as e:
    #         rospy.logerr("CvBridge Error: {0}".format(e))
    #     processed_image = self.process_image(cv_image)
    #     self.follow_line(processed_image)
    #     # Display the result
    #     cv2.imshow("Line Following", processed_image)
    #     cv2.waitKey(3)

    # def process_image(self, cv_image):
    #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #     _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    #     edges = cv2.Canny(thresholded, 50, 150)
    #     return edges
    
    # def process_image(self, cv_image):
    #     # Convert to grayscale
    #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
    #     # Threshold the image to isolate white lines
    #     _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    #     # Find edges for better line detection
    #     edges = cv2.Canny(thresholded, 50, 150)

    #     # Optionally, apply Hough Line Transform to find lines
    #     lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
    #     if lines is not None:
    #         for line in lines:
    #             x1, y1, x2, y2 = line[0]
    #             cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

    #     return cv_image
    

    # def follow_line(self, processed_image):
    #     lines = cv2.HoughLinesP(processed_image, 1, np.pi/180, 50, None, 50, 10)
    #     cmd_vel = Twist()
    #     if lines is not None:
    #         cmd_vel.linear.x = 0.5  # Example forward speed
    #         cmd_vel.angular.z = 0.1  # Example angular speed
    #     else:
    #         cmd_vel.linear.x = 0
    #         cmd_vel.angular.z = 0

    # def image_callback(self, msg: Image):
    #     try:
    #         # Convert image and resize
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         scale = 2.0
    #         image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    #         height, width, _ = image.shape

    #         # Convert to HSV
    #         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #         # HSV range for the track color (adjust as needed)
    #         lower_color = (100, 114, 95)
    #         upper_color = (170, 180, 180)

    #         # Mask + contour detection
    #         mask = cv2.inRange(hsv, lower_color, upper_color)
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         twist = Twist()

    #         if contours:
    #             largest_contour = max(contours, key=cv2.contourArea)
    #             M = cv2.moments(largest_contour)

    #             if M['m00'] != 0:
    #                 # Track centroid
    #                 cx = int(M['m10'] / M['m00'])
    #                 cy = int(M['m01'] / M['m00'])

    #                 # Draw track and centroid
    #                 cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
    #                 cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
    #                 cv2.putText(image, f"Centroid: ({cx}, {cy})", (cx + 10, cy),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    #                 # Compute error from center of image
    #                 error_x = cx - width // 2

    #                 # === Simple proportional controller ===
    #                 Kp_ang = 0.005  # gain for angular velocity
    #                 Kp_lin = 0.1    # optional: gain for forward speed

    #                 twist.linear.x = 0.1  # constant forward speed
    #                 twist.angular.z = -Kp_ang * error_x  # turn to minimize error

    #             else:
    #                 self.get_logger().warn("Zero-area contour detected, stopping robot")
    #                 twist.linear.x = 0.0
    #                 twist.angular.z = 0.0
    #         else:
    #             self.get_logger().info("No track found, stopping robot")
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0

    #         # Publish velocity command
    #         self.cmd_pub.publish(twist)

    #         # Show the image
    #         window_name = "Track Follower"
    #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    #         cv2.imshow(window_name, image)
    #         cv2.waitKey(1)

    #     except Exception as e:
    #         self.get_logger().error(f"Image processing error: {e}")

    # def image_callback(self, msg: Image):
    #     try:
    #         # Convertir imagen
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         scale = 2.0
    #         image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    #         height, width, _ = image.shape

    #         # Convertir a HSV y aplicar máscara de color
    #         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #         lower_color = (100, 114, 95)
    #         upper_color = (170, 180, 180)
    #         mask = cv2.inRange(hsv, lower_color, upper_color)
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         twist = Twist()
    #         dx, dy = 0.0, 0.0  # Correcciones de posición

    #         if contours:
    #             largest_contour = max(contours, key=cv2.contourArea)
    #             M = cv2.moments(largest_contour)

    #             if M['m00'] != 0:
    #                 cx = int(M['m10'] / M['m00'])
    #                 cy = int(M['m01'] / M['m00'])

    #                 # Dibujar centroide
    #                 cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
    #                 cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

    #                 # Error horizontal
    #                 error_x = cx - width // 2

    #                 # Control proporcional
    #                 Kp_ang = 0.005
    #                 twist.linear.x = 0.1
    #                 twist.angular.z = -Kp_ang * error_x

    #                 # Calcular correcciones para el nodo de navegación
    #                 dx = -error_x * 0.001  # de píxeles a metros aprox.
    #                 dy = 0.05  # avanzar siempre un poco
    #             else:
    #                 twist.linear.x = 0.0
    #                 twist.angular.z = 0.0
    #         else:
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0

    #         # Publicar velocidad directa (opcional)
    #         self.cmd_vel_pub.publish(twist)

    #         # Publicar corrección de posición
    #         corr_msg = Float32MultiArray()
    #         corr_msg.data = [dx, dy]
    #         self.correction_pub.publish(corr_msg)

    #         # Mostrar imagen
    #         # cv2.imshow("Track Follower", image)
    #         # cv2.waitKey(1)

    #     except Exception as e:
    #         rospy.logerr(f"Image processing error: {e}")


def main():
    try:
        lf = LineFollower()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()