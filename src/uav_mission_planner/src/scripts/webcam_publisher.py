#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread, Lock
from flask import Flask, Response, render_template_string

app = Flask(__name__)

html = """
<!DOCTYPE html>
<html>
<head>
    <title>Vista de Cámara</title>
</head>
<body>
    <h1>Live Camera View</h1>
    <h3>/camera_1 (desde webcam local)</h3>
    <img src="/video_feed" width="640" height="480"/>
    <!--
    <h3>/cone_image (ZED)</h3>
    <img src="/video_stereo" width="640" height="480"/>
    <h3>/h_image (ZED)</h3>
    <img src="/video_h" width="640" height="480"/>
    -->
</body>
</html>
"""

# =========================
#   VIDEO PUBLISHER (ZED → WEB)  --- DESACTIVADO
# =========================
class VideoPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = Lock()
        self.latest_frame = None
        self.lock_h = Lock()
        self.latest_h_frame = None

        self.debug_visual = False

        # ===== ZED DESACTIVADA =====
        # rospy.Subscriber('/cone_image', Image, self.cone_callback, queue_size=1)
        # rospy.Subscriber('/h_image', Image, self.h_callback, queue_size=1)

    # ===== FUNCIONES ZED DESACTIVADAS =====
    # def cone_callback(self, msg):
    #     pass

    # def h_callback(self, msg):
    #     pass

    def get_frame(self):
        return None

    def get_h_frame(self):
        return None


# =========================
#   IMAGE PUBLISHER (WEBCAM + QR)
# =========================
class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video0')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.pub = rospy.Publisher('/camera_1', Image, queue_size=10)
        self.pub_processed = rospy.Publisher('/camera_1/processed', Image, queue_size=10)

        # ===== PUBLICADORES DESACTIVADOS =====
        # self.rgb8pub = rospy.Publisher('/camera_1/rgb', Image, queue_size=10)
        # self.bgr8pub = rospy.Publisher('/camera_1/bgr', Image, queue_size=10)
        # self.mono8pub = rospy.Publisher('/camera_1/mono', Image, queue_size=10)
        # self.is_x = rospy.Publisher('/detect_x', Bool, queue_size=10)
        # self.is_h = rospy.Publisher('/detect_h', Bool, queue_size=10)

        self.qr_pub = rospy.Publisher('/qr_code', String, queue_size=10)

        self.latest_frame = None
        self.lock = Lock()

        self.crop_x = 160
        self.crop_y = 80
        self.crop_w = 480
        self.crop_h = 320

        self.qr_detector = cv2.QRCodeDetector()

    # ===== LINE FOLLOWER DESACTIVADO =====
    # def process_line_following(self, frame):
    #     return frame

    # ===== DETECCION X DESACTIVADA =====
    # def process_x_cross(self, frame):
    #     return frame, False

    # ===== DETECCION H DESACTIVADA =====
    # def process_h_blue(self, frame):
    #     return frame, False

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    rate.sleep()
                    continue

                h, w, _ = frame.shape
                x1 = max(0, self.crop_x)
                y1 = max(0, self.crop_y)
                x2 = min(w, x1 + self.crop_w)
                y2 = min(h, y1 + self.crop_h)

                cropped_frame = frame[y1:y2, x1:x2]

                # Publicación básica
                self.pub.publish(self.bridge.cv2_to_imgmsg(cropped_frame, "bgr8"))

                processed_frame = cropped_frame.copy()

                # =========================
                # SOLO QR ACTIVO
                # =========================
                data, points, _ = self.qr_detector.detectAndDecode(cropped_frame)

                if points is not None:
                    pts = points[0].astype(int)
                    cv2.polylines(processed_frame, [pts], True, (0, 255, 0), 2)

                    if data:
                        rospy.loginfo(f"QR detectado: {data}")
                        self.qr_pub.publish(data)
                        cv2.putText(processed_frame,
                                    data,
                                    (pts[0][0], pts[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (0, 255, 0),
                                    2)

                self.pub_processed.publish(
                    self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                )

                with self.lock:
                    self.latest_frame = processed_frame.copy()

                rate.sleep()

            except CvBridgeError as e:
                rospy.logerr(e)
                break

    def get_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            ok, jpeg = cv2.imencode('.jpg', self.latest_frame)
            return jpeg.tobytes() if ok else None


# ======= INSTANCIAS =======
ip_instance = ImagePublisher()
video_instance = VideoPublisher()  # queda instanciado pero inactivo


# =========================
#   FLASK ENDPOINTS
# =========================
@app.route('/')
def index():
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = ip_instance.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# ===== ZED ENDPOINTS DESACTIVADOS =====
# @app.route('/video_stereo')
# def video_stereo():
#     pass

# @app.route('/video_h')
# def video_h():
#     pass


# =========================
#   MAIN (LOCALHOST IGUAL)
# =========================
def main(args=None):
    try:
        ros_thread = Thread(target=ip_instance.run, daemon=True)
        ros_thread.start()

        host = '0.0.0.0'
        port = 8081

        rospy.loginfo("Iniciando publicador de imágenes...")
        print(f"✅ Servidor activo en http://{host}:{port}")

        app.run(host=host, port=port, threaded=True)

    except rospy.ROSInterruptException:
        os._exit(0)


if __name__ == '__main__':
    main()