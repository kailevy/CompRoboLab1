#!/usr/bin/env python

"""Node that controls Neato with wasd and space to stop"""
import rospy
import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3

movers = {'w':1,'s':-1}
turners = {'a':1,'d':-1}
stop = ' '

def getKey():
    """Helper function to get the pressed key"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None
pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
rospy.init_node('custom_twister')

while key != '\x03':
    key = getKey()
    if key in movers.keys():
        mv = movers[key]
    else:
        mv = 0
    if key in turners.keys():
        tn = turners[key]
    else:
        tn = 0
    twister = Twist(linear=Vector3(y=0,z=0),angular=Vector3(x=0,y=0))
    twister.linear.x = mv
    twister.angular.z = tn
    if key == stop:
        twister.linear.x = 0
        twister.angular.z = 0
    pub.publish(twister)
