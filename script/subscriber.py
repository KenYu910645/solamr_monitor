#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String
import tf


def callback(data):
    rospy.loginfo("Frame at : " + data.header.frame_id)
    rospy.loginfo("Position : " + str(data.pose.position))
    rospy.loginfo("Orientation : " + str(data.pose.orientation))

def main(args):
    #----- Init node ------#
    # Name of this node, anonymous means name will be auto generated.
    rospy.init_node('subscriber', anonymous=False)
    #----- Subscriber -------# 
    sub = rospy.Subscriber("topic", PoseStamped, callback)
    
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
