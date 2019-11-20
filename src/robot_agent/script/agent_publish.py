#!/usr/bin/env python

import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from sampling_msgs.msg import measurement

goal_pos = 0
pub = rospy.Publisher('/tilt_controller/command', Float64)

def transform_callback(data):
    global goal_pos
    rospy.loginfo(rospy.get_name() + ': Current motor angle {0}'.format(data.current_pos))

    # If the motor has reached its limit, publish a new command.
    if fabs(goal_pos-data.current_pos) < 0.01:
        if goal_pos == 0:
            goal_pos = 3.141592
        else:
            goal_pos = 0

        str = "Time: {0} Moving motor to {1}" .format(rospy.get_time(), goal_pos)
        rospy.loginfo(str)
        pub.publish(Float64(goal_pos))


def dxl_control():
    rospy.init_node('dxl_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, transform_callback).
    # Initial movement.
    pub.publish(Float64(goal_pos))
    rospy.spin()


if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass