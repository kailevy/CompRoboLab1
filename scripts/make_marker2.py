#!/usr/bin/env python

"""Script used to publish a marker 10 times per second that will be in front of Neato"""

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('make_marker2')

pub = rospy.Publisher("/marker2_msg", Marker, queue_size=10)

pose_msg = Pose(position=Point(x=1))
header_msg = Header(frame_id="base_link")
vis_msg = Marker(header=header_msg,
                type=2,
                scale=Vector3(0.5,0.5,0.5),
                pose=pose_msg,
                color=ColorRGBA(a=1,g=1))

r = rospy.Rate(10)


while not rospy.is_shutdown():
    vis_msg.header.stamp = rospy.Time.now()
    pub.publish(vis_msg)
    r.sleep()
