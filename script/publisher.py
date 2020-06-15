#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String
import tf

def main(args):
    #----- Init node ------# 
    # Name of this node, anonymous means name will be auto generated.
    rospy.init_node('publisher', anonymous=False)
    #----- Publisher -------# 
    # Latch means subscriber will get the latest msg, even it's publish before subscribe
    pub_1 = rospy.Publisher('topic'  , PoseStamped ,queue_size = 10,  latch=True)
    
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        # Create msg 
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.header.stamp = rospy.get_rostime()
        p.pose.position = Point(87, 123, 0) # x,y,z
        q = tf.transformations.quaternion_from_euler(0, 0, 666) # r,p,y
        p.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        # Publish msg
        pub_1.publish(p)
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass

