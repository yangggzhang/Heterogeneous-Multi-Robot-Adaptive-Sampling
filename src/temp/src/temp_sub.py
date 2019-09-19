import rospy
from temp.msg import Temperature

def callback(msg):
    rospy.loginfo("Time:%s \t I heard %s",str(msg.header.stamp.to_sec()), str(msg.data))
    print(msg.data)

def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("temp", Temperature, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()