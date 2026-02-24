#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped

class QRtoMove:
    def __init__(self):
        rospy.init_node('qr_to_move', anonymous=False)
        self.subscription = rospy.Subscriber('/qrcode/raw', String, self.callback)
        self.set_position = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)


    def callback(self, data):
        print("###### data", data)
        pose = self.string_to_pose(str(data))
        rospy.loginfo("info message %s" % pose)
        self.set_position.publish(pose)

    
    def string_to_pose(self, input_string):
        #TODO replace data from string 
        parts = input_string.replace('data: ', '').replace('"', '').split(',')

        if len(parts) != 5:
            raise ValueError("Input string should have 5 comma-separated values")

        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        # Asignar la orientacion basada en la direccion de la secuencia
        if parts[0] == 'N':
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
        elif parts[0] == 'E':
            pose.pose.position.x = 0.0
            pose.pose.position.y = 1.0
            pose.pose.orientation.w = 0.0
            pose.pose.orientation.z = 1.0
        elif parts[0] == 'S':
            pose.pose.position.x = -1.0
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = -1.0
        elif parts[0] == 'W':
            pose.pose.position.x = 0.0
            pose.pose.position.y = -1.0
            pose.pose.orientation.w = 0.0
            pose.pose.orientation.z = -1.0
        
        # Asignar el numero de posicion objetivo
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'  # Ajustar el frame_id segun sea necesario
        pose.pose.position.z = float(parts[4])

        return pose

def main():
    qtm = QRtoMove()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
