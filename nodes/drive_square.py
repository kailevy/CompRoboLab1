#!/usr/bin/env python

"""Node to drive Neato in a square (approximately) using odom measurements"""
import rospy
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry

def move_forward(sequence_num, position):
    """Moves the robot forward ~1m"""
    global sequence_keys
    global move_sequence

    print 'forward'
    print(move_sequence[sequence_num])
    print(position.x,position.y)

    twister = Twist(linear=Vector3(x=0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
    pub.publish(twister)
    if sequence_keys[sequence_num] == 'x':
        if position.x >= move_sequence[sequence_num]:
            print 'next1'
            return 1

    if sequence_keys[sequence_num] == 'y':
        if position.y <= move_sequence[sequence_num]:
            print 'next3'
            return 1

    if sequence_keys[sequence_num] == '-x':
        if position.x <= move_sequence[sequence_num]:
            print 'next5'
            return 1

    if sequence_keys[sequence_num] == '-y':
        if position.y >= move_sequence[sequence_num]:
            print 'next7'
            return 1

    return 0

def turn_right(sequence_num, orientation):
    """Turns robot to the right ~90degrees"""
    global sequence_keys
    global move_sequence

    print 'turn'
    twister = Twist(linear=Vector3(x=0,y=0,z=0),angular=Vector3(x=0,y=0,z=-0.2))
    pub.publish(twister)
    w_desired = move_sequence[sequence_num][0]
    z_desired = move_sequence[sequence_num][1]
    print (w_desired,z_desired)
    print(orientation.w,orientation.z)
    if sequence_keys[sequence_num] == 'zw1':
        if orientation.w <= w_desired and orientation.z <= z_desired:
            print 'next2'
            return 1

    if sequence_keys[sequence_num] == 'zw2':
        if orientation.w <= w_desired and orientation.z <= z_desired:
            print 'next4'
            return 1

    if sequence_keys[sequence_num] == 'zw3':
        if orientation.w <= w_desired and orientation.z <= z_desired:
            print 'next6'
            return 1

    if sequence_keys[sequence_num] == 'zw4':
        if orientation.w >= w_desired and orientation.z >= z_desired:
            print 'next8'
            return 1

    return 0

def callback(msg):
    """Callback function to handle the odom message"""
    neat_pose = msg.pose.pose
    global initial_pos
    global sequence
    global move_sequence

    if initial_pos == None:
        print 'here!'
        initial_pos = {'x': neat_pose.position.x,
                        'y': neat_pose.position.y,
                        'z': neat_pose.orientation.z,
                        'w': neat_pose.orientation.w}
        move_sequence = {0:initial_pos['x'] + 1,
                    1:(initial_pos['w']-0.25,initial_pos['z']-0.65),
                    2:initial_pos['y'] - 1,
                    3:(initial_pos['w']-0.9,initial_pos['z']-0.9),
                    4:initial_pos['x'],
                    5:(initial_pos['w']-1.75,initial_pos['z']-0.65),
                    6:initial_pos['y'],
                    7:(initial_pos['w']-0.03,initial_pos['z']+0.03)}
        # orientations adapted from, and further found with guess and check:
        # http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        # the 0.9, 0.7, and 0.2 correspond to quaternion angle values (approximately)
        # hopefully this is enough do defend my magic numbers

    if sequence % 2 == 0:
        sequence+= move_forward(sequence,neat_pose.position)
    else:
        sequence+= turn_right(sequence,neat_pose.orientation)
    sequence = sequence % 8

def listener():
    """Main function handler"""
    rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('square_driver')

    initial_pos = None
    move_sequence = None
    sequence = 0
    last_pos = None

    sequence_keys = ['x','zw1','y','zw2','-x','zw3','-y','zw4']

    while not rospy.is_shutdown():
        listener()
