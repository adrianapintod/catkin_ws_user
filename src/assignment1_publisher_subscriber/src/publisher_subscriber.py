#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

pub = rospy.Publisher('assignment1_publisher_subscriber', String, queue_size=10)

def callback(data):
    hello_str = rospy.get_caller_id() + "I heard: %s", data.data
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

def publisher_subscriber():
    # Initialize node
    rospy.init_node('publisher_subscriber', anonymous=True)
    # Initialize Subscriber
    rospy.Subscriber("yaw", Float32, callback)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    publisher_subscriber()