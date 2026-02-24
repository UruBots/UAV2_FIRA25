#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

# Global variables to store the current state and odometry data
px4_current_state = State()
zed_current_odom = Odometry()
zed_current_odom_cov = Odometry()

def px4_state_cb(msg):
    """ Callback function for PX4 state updates """
    global px4_current_state
    px4_current_state = msg

def zed_odom_cb(msg):
    """ Callback function for ZED odometry updates """
    global zed_current_odom
    zed_current_odom = msg

def zed_odom_cov_cb(msg):
    """ Callback function for ZED odometry updates """
    global zed_current_odom_cov
    zed_current_odom_cov = msg

def main():
    rospy.init_node("remap_pose_node", anonymous=True)

    # Subscribers
    rospy.Subscriber("/mavros/state", State, px4_state_cb)
    rospy.Subscriber("/zed/zed_node/pose", PoseStamped, zed_odom_cb)  # Now uses /zedm/zed_node/pose
    rospy.Subscriber("/zed/zed_node/pose_with_covariance", PoseWithCovarianceStamped, zed_odom_cov_cb)  # New subscriber

    # Publishers
    vision_pose_pub_cov = rospy.Publisher("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=1)
    vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)  # New publisher

    # Sleep for 60 seconds to allow startup time
    rospy.sleep(60)

    # Set loop rate 45 Hz (45 times per second)
    rate = rospy.Rate(30)

    # Wait until PX4 is connected
    while not rospy.is_shutdown() and not px4_current_state.connected:
        rospy.sleep(0.1)

    rospy.loginfo("Start vision pose information message!")

    while not rospy.is_shutdown():
        rospy.loginfo("Remapping vision pose information message!")
        #print("#### zed_current_odom",zed_current_odom)
        #print("#### zed_current_odom_cov",zed_current_odom_cov.pose)
        # Create PoseWithCovarianceStamped message
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "odom"
        cur_pose.header.stamp = rospy.Time.now()
        cur_pose.pose = zed_current_odom.pose

        # Create PoseStamped message
        cur_pose_cov = PoseWithCovarianceStamped()
        cur_pose_cov.header.frame_id = "odom"
        cur_pose_cov.header.stamp = rospy.Time.now()
        cur_pose_cov.pose = zed_current_odom_cov.pose  # Adjusting for PoseStamped format

        # Publish messages
        vision_pose_pub.publish(cur_pose)
        vision_pose_pub_cov.publish(cur_pose_cov)
        rospy.loginfo("Published remapped poses.")
        # rospy.loginfo("Published remapped poses.", cur_pose_cov, cur_pose)

        # Maintain loop rate
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Remapping Node interrupted.")
