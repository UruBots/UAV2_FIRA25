#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *
#from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, Bool 
from geometry_msgs.msg import Point


#global variable
latitude = 0.0
longitude = 0.0
home_altitude = HomePosition()  #new
qr_code = None
global local_position
local_position = None
global pose_x
global pose_y
global distanceQRy
global distanceQRx
distanceQRx = 1.25
distanceQRy = 1.38
global mission
global pose
pose = PoseStamped()
global base_pose
base_pose = PoseStamped()
global target_flag
target_flag = 0
global latest_qr
latest_qr = None
#bridge = CvBridge()
#global homeposition_z = HomePosition()

# Variable global para correcciones de línea
correccion_pos = [0.0, 0.0]
ruta_recorrida = []
home_altitude = 0.0
objetivo_encontrado = False
estado = None
cone_coords = None
h_coords = None
h_pixel_coords = None

# === Callbacks ===
def home_altitude_callback(msg):
   """Guarda la altura inicial al encender"""
   global home_altitude
   home_altitude = msg.pose


def local_position_callback(data):
    global local_position
    local_position = data
    #print("#### local_position_callback updated ###", local_position)

# def pos_correction_callback(msg):
#     global correccion_pos
#     correccion_pos = msg.data  # [dx, dy]

def pos_correction_callback(msg):
   """Recibe corrección lateral y de avance del seguidor de línea"""
   global correccion_pos
   if len(msg.data) == 2:
       correccion_pos = msg.data
   else:
       correccion_pos = [0.0, 0.0]

def cone_callback(msg):
   global cone_coords
   #cone_coords = msg.pose
   cone_coords = (msg.x, msg.y, msg.z)

def h_coords_callback(msg):
    global h_coords
    h_coords = (msg.x, msg.y, msg.z)

def h_pixel_coords_callback(msg):
    global h_pixel_coords
    h_pixel_coords = msg



#def image_callback(data):
#    try:
#        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#    except CvBridgeError as e:
#        rospy.logerr(e)
#        return

#    result_image, detection = process_image(cv_image)

#    if detection:
#        rospy.loginfo("H detected at: %s", detection)
        # this is for land
#        setLandMode()

#    try:
#        self.image_pub.publish(bridge.cv2_to_imgmsg(result_image, "bgr8"))
#    except CvBridgeError as e:
#       rospy.logerr(e)

def home_position_callback(msg):  #neww 
    global home_altitude
    if(msg):
        home_altitude = msg.position.z
    else:
        home_altitude = 0.0
    
    rospy.loginfo("Altura HOME actual: %.2f m" % home_altitude)

def process_image(cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 12:  # Assuming H shape approximates to 12 points
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, "H", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            return cv_image, (x, y, w, h)

    return cv_image, None

set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, local_position_callback)
#rospy.Subscriber('/camera_1', Image, image_callback)

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/set_mode")
        print(service(mode_ID, mode))
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)

def pub_reset_gps():
    for i in range (0,5):
     try:
         msg = GeoPointStamped()
         reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
         reset_gps.publish(msg)
     except rospy.ServiceException as e:
         print('Service call failed: %s' % e)

# TODO change to call_set_mode
#http://wiki.ros.org/mavros/CustomModes for custom modes
def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

# TODO change to call_set_mode
def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " % e)

def setArm():
    pub_reset_gps()
    time.sleep(1)
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        # TODO this is to verify why the drone is arming but not launching
        rospy.set_param("/mavros/vision_pose/tf/listen", True)
        pub_reset_gps()

        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s"%e)

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s"%e)


def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 0.8, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def set_target_position(x,y,z,w):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = w

    try:
        set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        set_point_pub.publish(pose)
    except rospy.ServiceException as e:
        print("Service set_target_position call failed: %s" % e)

def follow_line():
    pass

def string_to_pose(input_string):
    global mission
    global pose
    global distanceQRx
    global distanceQRy
    #print("#### input_string  ###", input_string)
    #TODO replace data from string 
    parts = str(input_string).replace('data: ', '').replace('"', '').split(',')

    print("#### parts ###", parts)
    if len(parts) != 5:
        raise ValueError("Input string should have 5 comma-separated values")
    if (parts[-1] == '0'):
        # Asignar la orientacion basada en la direccion de la secuencia
        if parts[mission-1] == 'N':
            pose.pose.position.x += distanceQRx
            pose.pose.position.y += 0.0
        elif parts[mission-1] == 'E':
            pose.pose.position.x += 0.0
            pose.pose.position.y += -distanceQRy
        elif parts[mission-1] == 'S':
            pose.pose.position.x += -distanceQRx
            pose.pose.position.y += 0.0
        elif parts[mission-1] == 'W':
            pose.pose.position.x += 0.0
            pose.pose.position.y += distanceQRy

        #rospy.loginfo("string_to_pose -> %s" % pose)
        # Asignar el numero de posicion objetivo
        #pose.header.stamp = rospy.Time.now()
        #pose.header.frame_id = 'map'  # Ajustar el frame_id segun sea necesario
        #pose.pose.position.z = float(parts[4])

        return pose, parts[4]

def go_to_destination(dest = "1.0, 0.0, 1.0, 0.0"):        
    x, y, z, w = dest.split(",")
    setGuidedMode()
    time.sleep(1)
    pub_reset_gps()
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    setArm()
    time.sleep(5)
    setTakeoffMode()
    time.sleep(5)
    set_target_position(float(x), float(y), float(z), float(w))
    time.sleep(1)
    setTakeoffMode()
    time.sleep(1)
    setDisarm()

#2026 revisado, funciona
def prueba():        
    setGuidedMode()
    print("Guidado")
    time.sleep(1)
    pub_reset_gps()
    print("fakegps")
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    setArm()
    print("Armado")
    time.sleep(3)
    setTakeoffMode()
    print("Takeoff")
    time.sleep(6)
    setLandMode()
    print("Landed")
    time.sleep(2)
    setDisarm()
    print("Desarmado")
    time.sleep(1)

def Callback():  # Formato: "y,w"
   global home_altitude
   z = home_altitude.position.z + 0.8  # altura deseada sobre el suelo
   print (z)

#2026 revisado,funciona    
def Box():        
    global home_altitude
    setGuidedMode()
    print("Guidado")
    time.sleep(1)
    pub_reset_gps()
    print("Fake GPS")
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    setArm()
    print("Armado")
    time.sleep(3)
    setTakeoffMode()
    print("TakeOFF")
    time.sleep(5)
    print("Frente")
    set_target_position(float(0.3), float(0.0), float(0.6), float(0.3))
    time.sleep(5)
    print("Izquierda")
    set_target_position(float(0.3), float(0.3), float(0.6), float(0.3))
    time.sleep(5)
    print("Atrás")
    set_target_position(float(0.0), float(0.3), float(0.6), float(0.3))
    time.sleep(5)
    print("Derecha")
    set_target_position(float(0.0), float(0.1), float(0.6), float(0.3))
    time.sleep(5)
    setLandMode()
    print("Landed")
    time.sleep(2)
    setDisarm()
    print("Desarmado")
    time.sleep(1)

#2026 revisado, funciona
def Z_adelante():    
    global home_altitude
    z = home_altitude + 0.8
    x = 0.8
    y = 0.0
    w = 0.3

    setGuidedMode()
    print("Guidado")
    time.sleep(1)
    pub_reset_gps()
    print("Fake GPS")
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    #set_target_position(float(0.0), float(0.0), float(0.0), float(0.0))
    #time.sleep(5)
    setArm()
    print("Armado")
    time.sleep(5)
    setTakeoffMode()
    print("TakeOFF")
    time.sleep(5)
    print("Frente")
    set_target_position(float(x), float(y), float(z), float(w))
    time.sleep(5)
    setLandMode()
    print("Landed")
    time.sleep(2)
    setDisarm()
    print("Desarmado")
    time.sleep(1)


#2026 TDP
def TDP():    
    global home_altitude
    z = home_altitude + 0.8
    #x = 0.4
    #y = 0.0
    #w = 0.3

    setGuidedMode()
    print("Guidado")
    time.sleep(1)
    pub_reset_gps()
    print("Fake GPS")
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    #set_target_position(float(0.0), float(0.0), float(0.0), float(0.0))
    #time.sleep(5)
    setArm()
    print("Armado")
    time.sleep(3)
    setTakeoffMode()
    print("TakeOFF")
    time.sleep(5)
    print("Frente")
    set_target_position(float(0.9), float(0.0), float(z), float(0.3))
    time.sleep(5)
    print("Izquierda")
    set_target_position(float(0.9), float(0.6), float(z), float(0.3))
    time.sleep(5)
    setLandMode()
    print("Landed")
    time.sleep(2)
    setDisarm()
    print("Desarmado")
    time.sleep(1)

def Pasos():

    global home_altitude
    z = 0.5
    total_pasos = 2
    paso_x = 0.2  # tamaño del paso en metros
    y = 0.0
    w = 0.3

    setGuidedMode()
    print("Guidado")
    time.sleep(1)
    pub_reset_gps()
    print("Fake GPS")
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    setArm()
    print("Armado.")
    time.sleep(3)
    setTakeoffMode()
    print("TakeOFF")
    time.sleep(4)

    # Movimiento por pasos en X
    for i in range(1, total_pasos + 1):
        x = paso_x * i
        print(f"Paso {i}: Avanzando a X = {x:.1f} m")
        set_target_position(float(x), float(y), float(z), float(w))
        time.sleep(3)  # Esperar a que complete el paso

    setLandMode()
    print("Landed")
    time.sleep(2)
    setDisarm()
    print("Desarmado")
    time.sleep(1)

def Pasos_QRSTOP():

    global home_altitude
    z = 0.6
    total_pasos = 2
    paso_x = 0.3  # tamaño del paso en metros
    y = 0.0
    w = 0.3

    setGuidedMode()
    print("Guidado.")
    time.sleep(1)

    pub_reset_gps()
    print("Fake GPS enviado.")
    time.sleep(1)

    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(2)

    setArm()
    print("Armado.")
    time.sleep(5)

    setTakeoffMode()
    print("Despegando...")
    time.sleep(5)

    print("Iniciando Navegación...")

    for i in range(1, total_pasos + 1):
        x = paso_x * i
        print(f"➡️ Paso {i}: Avanzando a X = {x:.2f} m")
        set_target_position(float(x), float(y), float(z), float(w))

        qr_detected = None
        try:
            # Espera hasta 3 segundos para ver si detecta un QR
            qr_detected = rospy.wait_for_message('/qrcode/raw', String, timeout=3)
        except:
            qr_detected = None

        if qr_detected:
            print(f"❌ QR detectado: '{qr_detected.data}'. Iniciando aterrizaje inmediato...")
            setLandMode()
            time.sleep(5)
            setDisarm()
            print("🔻 Desarmado.")
            return

    print("Todos los pasos completados. Aterrizando...")
    setLandMode()
    time.sleep(5)
    setDisarm()
    print("Desarmado.")

def Navegacion_Qrs():

    global home_altitude
    print('##### porque mierda falla ####', home_altitude)
    z = home_altitude + 1.35
    print("Z esta en :", z)
    paso = 0.3
    x = 0.0
    y = 0.0
    w = 0.3
    estado = None

    for i in range (0,10):
        setGuidedMode()
        print("Guidado.")
        time.sleep(0.3)

    pub_reset_gps()
    print("Fake GPS enviado.")
    time.sleep(1)

    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(2)

    setArm()
    print("Armado.")
    time.sleep(5)

    setTakeoffMode()
    print("Despegando...")
    time.sleep(5)

    print("Movimiento inicial hacia adelante (X+)")

    while estado is None and not rospy.is_shutdown():
        x += paso
        print(f"➡️ Movimiento inicial → X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        try:
            qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.5)
            letra = qr.data.strip().upper()[0]
            if letra in ['N', 'S', 'E', 'W']:
                estado = letra
                print(f"🧭 QR detectado → Estado inicial: {estado}")
                if estado == 'N':
                    print("🧭 Dirección: NORTE (X+)")
                elif estado == 'S':
                    print("🧭 Dirección: SUR (X−)")
                elif estado == 'E':
                    print("🧭 Dirección: ESTE (Y−)")
                elif estado == 'W':
                    print("🧭 Dirección: OESTE (Y+)")
                break
            else:
                print(f"⚠️ Código QR ignorado: '{qr.data}'")
        except:
            pass

        time.sleep(3)

    while not rospy.is_shutdown():
        if estado == 'N':
            x += paso
            print("🧭 Dirección: NORTE (X+)")
        elif estado == 'S':
            x -= paso
            print("🧭 Dirección: SUR (X−)")
        elif estado == 'E':
            y -= paso
            print("🧭 Dirección: ESTE (Y−)")
        elif estado == 'W':
            y += paso
            print("🧭 Dirección: OESTE (Y+)")

        print(f"➡️ Estado: {estado} → X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        for _ in range(30):
            try:
                qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.1)
                letra = qr.data.strip().upper()[0]
                if letra in ['N', 'S', 'E', 'W'] and letra != estado:
                    print(f"🔁 Cambio de dirección: {estado} → {letra}")
                    estado = letra
                    if estado == 'N':
                        print("🧭 Nueva dirección: NORTE (X+)")
                    elif estado == 'S':
                        print("🧭 Nueva dirección: SUR (X−)")
                    elif estado == 'E':
                        print("🧭 Nueva dirección: ESTE (Y−)")
                    elif estado == 'W':
                        print("🧭 Nueva dirección: OESTE (Y+)")
                    break
                else:
                    print(f"🔁 QR recibido sin cambio: '{qr.data}'")
            except:
                pass

        time.sleep(5)

# ta mal, revisar
def Navegacion_Qrs_FIRA25():
    global home_altitude
    print('##### porque mierda falla ####', home_altitude)
    z = home_altitude + 1.5
    print("Z esta en :", z)
    paso = 0.3
    x = 0.0
    y = 0.0
    w = 0.3
    estado = None
    qr_detectado = False  # Para asegurarnos que no lea QR más de una vez.

    for i in range(0, 10):
        setGuidedMode()
        print("Guidado.")
        time.sleep(0.3)

    pub_reset_gps()
    print("Fake GPS enviado.")
    time.sleep(1)

    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(2)

    setArm()
    print("Armado.")
    time.sleep(5)

    setTakeoffMode()
    print("Despegando...")
    time.sleep(5)

    print("Movimiento inicial hacia adelante (X+)")

    while estado is None and not rospy.is_shutdown():
        x += paso
        print(f"➡️ Movimiento inicial → X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        try:
            qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.5)
            letra = qr.data.strip().upper()[0]
            if letra in ['N', 'S', 'E', 'W'] and not qr_detectado:
                estado = letra
                qr_detectado = True  # Marcar que el QR fue detectado y ya no se debe volver a leer
                print(f"🧭 QR detectado → Estado inicial: {estado}")
                if estado == 'N':
                    print("🧭 Dirección: NORTE (X+)")
                elif estado == 'S':
                    print("🧭 Dirección: SUR (X−)")
                elif estado == 'E':
                    print("🧭 Dirección: ESTE (Y−)")
                elif estado == 'W':
                    print("🧭 Dirección: OESTE (Y+)")
                break
            else:
                print(f"⚠️ Código QR ignorado: '{qr.data}'")
        except:
            pass

        time.sleep(3)

    while not rospy.is_shutdown():
        if estado == 'N':
            x += paso
            print("🧭 Dirección: NORTE (X+)")
        elif estado == 'S':
            x -= paso
            print("🧭 Dirección: SUR (X−)")
        elif estado == 'E':
            y -= paso
            print("🧭 Dirección: ESTE (Y−)")
        elif estado == 'W':
            y += paso
            print("🧭 Dirección: OESTE (Y+)")

        print(f"➡️ Estado: {estado} → X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        for _ in range(30):
            try:
                qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.1)
                letra = qr.data.strip().upper()[0]
                if letra in ['N', 'S', 'E', 'W'] and letra != estado:
                    print(f"🔁 Cambio de dirección: {estado} → {letra}")
                    estado = letra
                    if estado == 'N':
                        print("🧭 Nueva dirección: NORTE (X+)")
                    elif estado == 'S':
                        print("🧭 Nueva dirección: SUR (X−)")
                    elif estado == 'E':
                        print("🧭 Nueva dirección: ESTE (Y−)")
                    elif estado == 'W':
                        print("🧭 Nueva dirección: OESTE (Y+)")
                    break
                else:
                    print(f"🔁 QR recibido sin cambio: '{qr.data}'")
            except:
                pass

        time.sleep(5)


def read_qr_and_go_to_destination(direction):
    global mission
    global pose
    global local_position
    global target_flag
    global latest_qr
    global altitude
    global distanceQRx
    global distanceQRy
    global base_pose
    mission = 1
    try:
        if (local_position != None):
            time.sleep(1)
            setGuidedMode()
            rospy.loginfo("Setou")
            time.sleep(1)
            rospy.set_param("/mavros/vision_pose/tf/listen", True)
            time.sleep(1)
            pub_reset_gps()
            pub_reset_gps()
            rospy.loginfo("Fake gps")
            time.sleep(5)
            setArm()
            time.sleep(1)
            setArm()
            rospy.loginfo("armou")
            time.sleep(1)
            setTakeoffMode()
            rospy.loginfo("take off")
            time.sleep(5)
            # 1 meter to the rgight
            print("### GET OVER THE QR CODE")
            base_pose.pose.position.x = 0
            base_pose.pose.position.y = 0
            #set_target_position(base_pose, 0.0, -2.2, 0.0)
            if(direction == 'right'):
                  set_target_position(base_pose, 0.0, -8.0, 0.0)
            else:
                  set_target_position(base_pose, 0.0, 8.0, 0.0)
            #set_target_position(0.0,1.0,0.0)
            time.sleep(1.5)
            #base_pose.pose.position.x = 0
            #base_pose.pose.position.y = -7.2

            #set_target_position(base_pose, 1.25, -7.2, 0.0)
            #time.sleep(1.5)
            #print("Going down")
            #set_target_position(distanceQRx, 0.0, -0.5)
            # set_target_position(0.0, 2.0, 0.0)
            #time.sleep(10)
            # set_target_position(0.0, 3.0, 0.0)
            # time.sleep(5)


            while True:
                rospy.loginfo("### WHILE ###")
                try:
                    qr_detected = rospy.wait_for_message('/qrcode/raw', String, timeout=10)
                except:
                    qr_detected = None
                    rospy.loginfo("### Exception on QR_DETECTED ###")
                    break

                if(qr_detected):
                    latest_qr = qr_detected
                    print("QR Code detected", qr_detected)
                    print("#### BASE POSITION BEFORE ->", base_pose.pose.position.x, base_pose.pose.position.y)
                    #rospy.loginfo("QR Code detected", qr_detected)
                    pose, target_flag = string_to_pose(qr_detected)
                else:
                    rospy.loginfo("QR Code not detected")
                    # setLandMode()
                break
                print("#### BASE POSITION ->", base_pose.pose.position.x, base_pose.pose.position.y)
                print("#### GOING NEXT POSITION ->", pose.pose.position.x, pose.pose.position.y)
                set_target_position(base_pose, pose.pose.position.x, pose.pose.position.y)
                base_pose = pose
                #print("#### pose ->", pose)
                #print("#### GOING NEXT POSITION ->", pose.pose.position.x, pose.pose.position.y)
                #time.sleep(10)

                if (target_flag == 1): mission = 2
                if (target_flag == 2): mission = 3
                if (target_flag == 3): mission = 4
                if (target_flag == 4): break

            # read qr codes with node.
            new_pose = PoseStamped()
            new_pose.pose.position.x = 0
            new_pose.pose.position.y = -7.2
            time.sleep(5)
            try:
                qr_detected = rospy.wait_for_message('/qrcode/raw', String, timeout=10)
            except:
                qr_detected = None
                setLandMode()
                rospy.loginfo("### Exception on QR_DETECTED ###")

            set_target_position(new_pose, 1.25, -7.2)
            time.sleep(1.5)
            try:
                qr_detected = rospy.wait_for_message('/qrcode/raw', String, timeout=10)
            except:
                qr_detected = None
                rospy.loginfo("### Exception on QR_DETECTED ###")

            setLandMode()
    except rospy.ServiceException as e:
        rospy.loginfo("Exception read_qr_and_go_to_destination", e)
        setLandMode()

def task_1():
    global home_altitude, correccion_pos
    z = home_altitude + 1.5
    w = 0.3
    x, y = 0.0, 0.0
    estado = None

    # Suscripción al seguidor de línea
    rospy.Subscriber('/line_follower/pos_correction', Float32MultiArray, pos_correction_callback)

    # ==== Inicialización del dron ====
    for _ in range(10):
        setGuidedMode()
        print("Guidado.")
        time.sleep(0.3)

    pub_reset_gps()
    print("Fake GPS enviado.")
    time.sleep(1)

    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(2)

    setArm()
    print("Armado.")
    time.sleep(5)

    setTakeoffMode()
    print("Despegando...")
    time.sleep(5)

    print("Movimiento inicial siguiendo línea hasta encontrar QR...")

    # ==== Movimiento inicial con seguidor de línea ====
    while estado is None and not rospy.is_shutdown():
        # Aplicar corrección de línea
        x += correccion_pos[0]
        y += correccion_pos[1]
        print(f"➡️ X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        try:
            qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.5)
            letra = qr.data.strip().upper()[0]
            if letra in ['N', 'S', 'E', 'W']:
                estado = letra
                print(f"🧭 Dirección inicial detectada: {estado}")
                break
        except rospy.ROSException:
            pass

    # ==== Navegación principal ====
    while not rospy.is_shutdown():
        if estado == 'N':
            #y += correccion_pos[1]  # avanzar en Y
            x += correccion_pos[0]  # corrección lateral
        elif estado == 'S':
            #y -= correccion_pos[1]
            x -= correccion_pos[0]
        elif estado == 'E':
            #x += correccion_pos[1]
            y += correccion_pos[0]
        elif estado == 'W':
            #x -= correccion_pos[1]
            y -= correccion_pos[0]

        print(f"➡️ Estado: {estado} → X={x:.2f}, Y={y:.2f}")
        set_target_position(float(x), float(y), float(z), float(w))

        # Revisar QR para cambio de dirección
        for _ in range(30):
            try:
                qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.1)
                letra = qr.data.strip().upper()[0]
                if letra in ['N', 'S', 'E', 'W'] and letra != estado:
                    print(f"🔁 Cambio de dirección: {estado} → {letra}")
                    estado = letra
                    break
            except rospy.ROSException:
                pass

        time.sleep(0.3)

    # no aterrizamos
    # # ==== Aterrizaje final ====
    # print("Aterrizando...")
    # setLandMode()
    # time.sleep(5)
    
    # tarea 6
    # ==== Retorno a HOME ====
    print("🔙 Retornando al punto inicial siguiendo la ruta...")
    for (rx, ry) in reversed(ruta_recorrida):
        set_target_position(rx, ry, z, w)
        time.sleep(0.3)

    # Aterrizar
    setLandMode()
    print("✅ Misión completada. Dron en tierra.")
    time.sleep(5)
    setDisarm()
    print("Desarmado.")

# === Lógica principal ===
def task_1_y_6():
    global home_altitude, correccion_pos, estado, objetivo_encontrado, ruta_recorrida
    #rospy.init_node("fira_task_1_6")
    # Suscriptores
    rospy.Subscriber('/line_follower/pos_correction', Float32MultiArray, pos_correction_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, home_altitude_callback)

    print(f"correcion_pos x {correccion_pos[0]} y {correccion_pos[1]}")

    # Esperar home_altitude
    # rospy.loginfo("Esperando altura inicial...")
    # while home_altitude is None and not rospy.is_shutdown():
    #     rospy.sleep(0.1)

    z = home_altitude + 1.5
    w = 0.3
    x, y = 0.0, 0.0
    paso = 0.1


    # ==== Inicialización ====
    for _ in range(10):
        setGuidedMode()
        rospy.sleep(0.3)

    pub_reset_gps()
    rospy.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    rospy.sleep(2)
    setArm()
    rospy.sleep(5)
    setTakeoffMode()
    rospy.sleep(5)

    rospy.loginfo("🚀 Iniciando Tarea 1 (Visual Navigation)...")
    
    rospy.loginfo("🚀 Moviendo hacia adelante")
    # Moviendo hacia delante
    set_target_position(float(x + 0.5), float(y), float(z), float(w))
    first = True
    # ==== Movimiento inicial hasta primer QR ====
    while estado is None and not rospy.is_shutdown():
        if first:
           x += correccion_pos[0] + paso
           first = False 

        x += correccion_pos[0] + 0.02
        y += correccion_pos[1]
        ruta_recorrida.append((x, y))
        set_target_position(x, y, z, w)

        # rospy.loginfo(f"🚀 While moviendo x:{x}, y={y}")


        try:
            qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.5)
            letra = qr.data.strip().upper()[0]
            rospy.loginfo(f"🚀 QR DETECTADO  letra={letra}")

            if letra in ['N', 'S', 'E', 'W']:
                estado = letra
                rospy.loginfo(f"🧭 Dirección inicial: {estado}")
                break
        except rospy.ROSException:
            pass

    # ==== Navegación principal ====
    while not rospy.is_shutdown() and not objetivo_encontrado:
        if estado == 'N':
            # y += correccion_pos[1]
            x += correccion_pos[0]
        elif estado == 'S':
            # y -= correccion_pos[1]
            x -= correccion_pos[0]
        elif estado == 'E':
            x += correccion_pos[1]
            # y -= correccion_pos[0]
        elif estado == 'W':
            x -= correccion_pos[1]
            # y += correccion_pos[0]

    # while not rospy.is_shutdown():
    #     if estado == 'N':
    #         x += paso
    #         print("🧭 Dirección: NORTE (X+)")
    #     elif estado == 'S':
    #         x -= paso
    #         print("🧭 Dirección: SUR (X−)")
    #     elif estado == 'E':
    #         y -= paso
    #         print("🧭 Dirección: ESTE (Y−)")
    #     elif estado == 'W':
    #         y += paso
    #         print("🧭 Dirección: OESTE (Y+)")

        ruta_recorrida.append((x, y))
        set_target_position(x, y, z, w)

        rospy.loginfo(f"🚀 SetTarget position moviendo x:{x}, y={y}")


        # Revisar QR
        try:
            qr = rospy.wait_for_message('/qrcode/raw', String, timeout=0.1)
            is_x = rospy.wait_for_message('detect_x', Bool, timeout=0.1)
            letra = qr.data.strip().upper()[0]
            if letra in ['N', 'S', 'E', 'W'] and letra != estado:
                rospy.loginfo(f"🔁 Cambio de dirección: {estado} → {letra}")
                estado = letra
            # detectar h en el pisodetectar_cruz_roja
            elif is_x:
                rospy.loginfo("🎯 Objetivo final detectado")
                objetivo_encontrado = True
                rospy.sleep(2)  # Esperar un poco para estabilizar
                setLandMode()
                rospy.sleep(3)
                setDisarm()
                rospy.sleep(3)
                rospy.sleep(5)
                setTakeoffMode()
                rospy.sleep(5)
                # girar el dron
                set_target_position(x, y, z + 0.2, w)  #

        except rospy.ROSException:
            pass

        rospy.sleep(0.3)

    rospy.loginfo("🏁 Tarea 1 completada, iniciando Tarea 6 (retorno)...")

    # ==== Retorno a HOME ====
    for (rx, ry) in reversed(ruta_recorrida):
        set_target_position(rx, ry, z + 0.2, w)  # un poco más alto por seguridad
        rospy.sleep(0.3)

    setLandMode()
    rospy.sleep(5)
    setDisarm()
    rospy.loginfo("✅ Misión completada. Dron en tierra.")

# def challenge_1():
#     global home_altitude, cone_coords
#     # Suscriptores
#     rospy.Subscriber('/cone_coords', Point, cone_callback)
#     rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, local_position_callback)

#     z = home_altitude + 1.0
#     w = 0.3
#     x, y = 0.0, 0.0

#     # ==== Inicialización ====
#     for _ in range(10):
#         setGuidedMode()
#         rospy.sleep(0.3)

#     pub_reset_gps()
#     rospy.sleep(1)
#     rospy.set_param("/mavros/vision_pose/tf/listen", True)
#     rospy.sleep(2)
#     setArm()
#     rospy.sleep(5)
#     setTakeoffMode()
#     rospy.sleep(5)

#     rospy.loginfo("🚀 Iniciando Tarea de deteccion de objeto...")

#     if cone_coords:
#         rospy.loginfo('navegando al dron')
#         x, y, z = cone_coords
#         tolerance = 0.05
#         paso = 0.3

#         while not rospy.is_shutdown():
#             diff = x - local_position[0]

#             if abs(diff) - tolerance:
#                 rospy.loginfo("Objetivo alcanzado")
#                 break

#             if diff > 0:
#                 x += min(paso, diff)
#             else:
#                 x += max(-paso, diff)
            
#             rospy.loginfo(f"🚀 Navegando en x={x}, y={y}, z={z}, w={w}")
#             set_target_position(float(x), float(y), float(z), float(1.0))


#     setLandMode()
#     rospy.sleep(5)
#     setDisarm()
#     rospy.loginfo("✅ Misión completada. Dron en tierra.")

def challenge_1():
    global home_altitude, cone_coords, local_position  # local_position é PoseStamped vindo do callback

    # --- Inscrições (idealmente faça isso uma vez fora) ---
    rospy.Subscriber('/cone_coords', Point, cone_callback)
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, local_position_callback)

    z = float(home_altitude) + 1.0
    w = 1.0
    x_goal, y_goal = 0.0, 0.0

    # ==== Inicialização de voo ====
    for _ in range(10):
        setGuidedMode()
        rospy.sleep(0.3)

    pub_reset_gps()
    rospy.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    rospy.sleep(2)
    setArm()
    rospy.sleep(5)
    setTakeoffMode()
    rospy.sleep(5)

    rospy.loginfo("🚀 Iniciando Tarefa de detecção de objeto...")

    # Espera pose válida
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if local_position is not None:
            break
        rospy.logwarn_throttle(2.0, "[challenge_1] Aguardando pose local...")
        rate.sleep()

    # Se já temos coordenadas do cone, define alvo
    if cone_coords:
        rospy.loginfo('🧭 Navegando até o cone')
        # cone_coords pode ser Point ou tupla
        if isinstance(cone_coords, Point):
            x_goal, y_goal = float(cone_coords.x), float(cone_coords.y)
            z_goal = float(cone_coords.z) if hasattr(cone_coords, 'z') else z
        else:
            x_goal, y_goal, z_goal = float(cone_coords[0]), float(cone_coords[1]), float(cone_coords[2])
    else:
        rospy.logwarn("⚠️ Sem coords do cone ainda; mantendo alvo atual.")
        x_goal, y_goal, z_goal = 0.0, 0.0, z

    tolerance = 0.05  # m
    paso = 0.3        # m por iteração (máx)
    rospy.loginfo(f"[challenge_1] Alvo: X={x_goal:.2f}, Y={y_goal:.2f}, Z={z_goal:.2f}")

    z_goal = z + 0.12 # mantener 1 m de altitud (o lo que uses en z)

    kp = 0.6
    rate = rospy.Rate(10)

    err_prev = None
    diverg_count = 0
    MAX_DIVERG = 5

    while not rospy.is_shutdown():
        p = local_position.pose.position
        x_cur = float(p.x)

        diff = x_goal - x_cur

        # llegada
        if abs(diff) <= tolerance:
            rospy.loginfo("✅ Objetivo en X alcanzado (|err| <= %.2f m)", tolerance)
            break

        # guarda anti-divergencia
        if err_prev is not None and abs(diff) > abs(err_prev) + 1e-3:
            diverg_count += 1
        else:
            diverg_count = 0
        err_prev = diff

        if diverg_count >= MAX_DIVERG:
            rospy.logwarn("⚠️ Error creciendo %d ciclos. Posible mismatch de frame (X invertido o X↔Y). Invirtiendo X de forma segura.", MAX_DIVERG)
            # opción A: invertir signo de diff
            diff = -diff
            diverg_count = 0
            # (si sigue divergiendo, intercambia X/Y abajo)

        # control proporcional con límite de paso
        delta = max(-paso, min(paso, kp * diff))
        new_x = x_cur + delta

        rospy.loginfo(f"🚀 Navegando: x_cmd={new_x:.2f} (err={diff:.2f})  goal={x_goal:.2f}")
        set_target_position(float(new_x), float(y_goal), float(z_goal), float(1.0))

        rate.sleep()

    # while not rospy.is_shutdown():
    #     # Leitura atual
    #     p = local_position.pose.position   # PoseStamped
    #     x_cur, y_cur, z_cur = float(p.x), float(p.y), float(p.z)

    #     # Erro apenas no eixo X (como seu código sugere)
    #     diff = x_goal - x_cur

    #     # Chegou?
    #     if abs(diff) <= tolerance:
    #         rospy.loginfo("✅ Objetivo em X alcançado (|erro| <= %.2f m).", tolerance)
    #         break

    #     # Avanço limitado por passo (evita overshoot)
    #     delta = diff
    #     if delta > 0:
    #         delta = min(paso, delta)
    #     else:
    #         delta = max(-paso, delta)

    #     new_x = x_cur + delta

    #     rospy.loginfo(f"🚀 Navegando: x={new_x:.2f} (err={diff:.2f}), y={y_goal:.2f}, z={z_goal:.2f}")
    #     set_target_position(float(new_x), float(y_goal), float(z_goal), float(w))

    #     rate.sleep()

    # Go back 1 meter and then land
    set_target_position(float(new_x - 1.0), float(y_goal), float(z_goal), float(1.0))
    rospy.sleep(3)
    rospy.loginfo("🏁 Missão concluída. Dron em terra.")
    setLandMode()
    rospy.sleep(5)
    setDisarm()
    rospy.loginfo("🏁 Missão concluída. Dron em terra.")

def challenge_2():
    global home_altitude, cone_coords, local_position, h_pixel_coords

    # --- Subscripciones (solo una vez fuera idealmente) ---
    rospy.Subscriber('/h_coords', Point, h_coords_callback)  # coords 3D desde ZED Mini
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, local_position_callback)
    rospy.Subscriber('/h_pixel_coords', Point, h_pixel_coords_callback)  # coords en pixeles desde cámara inferior

    # Parámetros iniciales
    z_takeoff = float(home_altitude) + 1.0
    paso = 0.3
    kp_pos = 0.6
    kp_pix = 0.002  # Ganancia para corrección en píxeles
    tolerance_xy = 0.05  # m
    tolerance_pix = 10   # px
    dist_switch_camera = 1.0  # m para pasar a cámara inferior

    # ==== Inicialización vuelo ====
    for _ in range(10):
        setGuidedMode()
        rospy.sleep(0.3)

    pub_reset_gps()
    rospy.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    rospy.sleep(2)
    setArm()
    rospy.sleep(5)
    setTakeoffMode()
    rospy.sleep(5)

    rospy.loginfo("🚀 Iniciando seguimiento de la H en movimiento...")

    # Espera pose inicial
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and local_position is None:
        rospy.logwarn_throttle(2.0, "Esperando pose local...")
        rate.sleep()

    #fase = "SEGUIMIENTO_3D"
    fase = "SEGUIMIENTO_VISUAL"

    while not rospy.is_shutdown():
        p = local_position.pose.position
        x_cur, y_cur, z_cur = float(p.x), float(p.y), float(p.z)

        if fase == "SEGUIMIENTO_3D":
            if cone_coords is not None:
                x_goal, y_goal, z_goal = cone_coords.x, cone_coords.y, z_takeoff

                # Calcular error y mover
                err_x = x_goal - x_cur
                err_y = y_goal - y_cur

                delta_x = max(-paso, min(paso, kp_pos * err_x))
                delta_y = max(-paso, min(paso, kp_pos * err_y))

                set_target_position(x_cur + delta_x, y_cur + delta_y, z_goal, 1.0)

                rospy.loginfo(f"[3D] goal=({x_goal:.2f},{y_goal:.2f}) err=({err_x:.2f},{err_y:.2f})")

                # Cambiar a cámara inferior si está cerca
                dist_xy = (err_x**2 + err_y**2)**0.5
                if dist_xy <= dist_switch_camera:
                    rospy.loginfo("📷 Cambiando a seguimiento visual (cámara inferior)")
                    fase = "SEGUIMIENTO_VISUAL"
                    continue
            else:
                rospy.logwarn("⚠️ Sin coords 3D aún")

        elif fase == "SEGUIMIENTO_VISUAL":
            if h_pixel_coords is not None:
                img_center_x, img_center_y = 320, 240  # para cámara 640x480
                err_x_pix = h_pixel_coords.x - img_center_x
                err_y_pix = h_pixel_coords.y - img_center_y

                # Convertir error de píxeles a corrección en metros
                corr_x = kp_pix * err_x_pix
                corr_y = kp_pix * err_y_pix

                set_target_position(x_cur + corr_x, y_cur + corr_y, z_cur, 1.0)

                rospy.loginfo(f"[VISUAL] err_pix=({err_x_pix},{err_y_pix}) corr=({corr_x:.3f},{corr_y:.3f})")

                # Si centrado, bajar lentamente
                if abs(err_x_pix) <= tolerance_pix and abs(err_y_pix) <= tolerance_pix:
                    z_goal = z_cur - 0.05
                    set_target_position(x_cur, y_cur, z_goal, 1.0)

                    if z_goal <= 0.15:
                        rospy.loginfo("🏁 H alcanzada. Aterrizando...")
                        setLandMode()
                        break
            else:
                rospy.logwarn("⚠️ Sin coords en píxeles aún")

        rate.sleep()

    rospy.sleep(5)
    setDisarm()
    rospy.loginfo("✅ Misión completada.")




def menu():
    print("Press")
    print("1: to set mode to GUIDED")
    print("2: to set mode to STABILIZE")
    print("3: to set mode to ARM the drone")
    print("4: to set mode to DISARM the drone")
    print("5: to set mode to TAKEOFF")
    print("6: to set mode to LAND")
    print("7: fakegps")
    print("8: print GPS coordinates")
    print("9: Go to destination")
    print("10: Box")
    print("11: Prueba")
    print("12: Adelante")
    print("13: TDP")
    print("14: Print Callback Z")
    print("15: Navegación Qr's")
    print("16: FIRA25 task 1")
    print("17: FIRA25 task 1 y 6")
    print("18: Navegación QR's FIRA25")
    print("19: Tech Challenge 1")
    print("20: Tech Challenge 2")
    
def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7','8','9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20'])):
        menu()
        x = input("Enter your input: ")
        if (x=='1'):
            setGuidedMode()
        elif(x=='2'):
            setStabilizeMode()
        elif(x=='3'):
            setArm()
        elif(x=='4'):
            setDisarm()
        elif(x=='5'):
            setTakeoffMode()
        elif(x=='6'):
            setLandMode()
        elif(x=='7'):
            pub_reset_gps()
        elif(x=='8'):
            global latitude
            global longitude
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
        elif(x=='9'):
            #dest = raw_input("Enter location to go : example 2.8, 0.0, 2.0, 2.0")
            dest = "1.0,0.0,0.5,0.3"
            go_to_destination(dest)
        elif(x=='10'):
            Box()
        elif(x=='11'):
            prueba()
        elif(x=='12'):
            Z_adelante()
        elif(x=='13'):
            TDP()
        elif(x=='14'):
            Callback()
        elif(x=='15'):
            Navegacion_Qrs()
        elif(x=='16'):
            task_1()
        elif(x=='17'):
            task_1_y_6()
        elif(x=='18'):
            Navegacion_Qrs_FIRA25()
        elif(x== '19'):
            challenge_1()
        elif(x== '20'):
            challenge_2()
        else:
            print("Exit")


if __name__ == '__main__':
    try:
        rospy.init_node('uav_mission_planner', anonymous=True)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
        rospy.Subscriber("/mavros/home_position/home", HomePosition, home_position_callback)

        # state = rospy.Subscriber('/mavros/state', State, state_callback)
        # TODO check with subscription to state
        myLoop()
        # rospy.spin()
    except rospy.ROSInterruptException:
        setLandMode()
        rospy.loginfo("#### Exception uav ####")
