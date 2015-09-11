#!/usr/bin/env python

"""Script used to publish a marker 10 times per second"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('make_marker')

pub = rospy.Publisher("/marker_msg", Marker, queue_size=10)

pose_msg = Pose(position=Point(x=1,y=2))
header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
vis_msg = Marker(header=header_msg,
                type=2,
                scale=Vector3(1,1,1),
                pose=pose_msg,
                color=ColorRGBA(a=1))

r = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(vis_msg)
    r.sleep()
