import rospy
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo("I heard %s",str(data.data))
    print(data.data)

def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("temp", Float64, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()