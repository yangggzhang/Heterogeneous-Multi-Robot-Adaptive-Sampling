#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
from temp.msg import temp_odom


def tmp_callback(data, dataDict):
    rospy.loginfo("I heard %s",str(data.data))
    dataDict['temp'] = data.data
    print(data.data)

def odom_callback(data, dataDict):
    dataDict['x'] = data.pose.pose.position.x
    dataDict['y'] = data.pose.pose.position.y

def makeTempOdom(dataDict):
    msg = temp_odom()
    msg.x = dataDict['x']
    msg.y = dataDict['y']
    msg.temp = dataDict['temp']
#    rospy.loginfo("successfully made message!")
    return msg

if __name__ == "__main__":
    rospy.init_node('node_name')
    dataDict = dict()
    rospy.Subscriber("temp", Float64, tmp_callback,(dataDict))
    rospy.Subscriber("odometry/filtered", Odometry, odom_callback, (dataDict))
    pub = rospy.Publisher('temp_odom', temp_odom)
    r = rospy.Rate(10)
    rospy.sleep(2)
    while not rospy.is_shutdown():
        msg = makeTempOdom(dataDict)
        pub.publish(msg)
        r.sleep

        
