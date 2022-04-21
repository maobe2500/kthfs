#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def publish_nums(n=4):
    pub = rospy.Publisher('/Oberg', Float32, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(20)
    rospy.loginfo('Publisher node started, now publishing... ')
    k = 0
    while not rospy.is_shutdown():
        k_current = '{}'.format(k)
        rospy.loginfo(k_current)
        pub.publish(k)
        rate.sleep()
        k += n


if __name__ == '__main__':
    try:
        publish_nums()
    except rospy.ROSInterruptException:
        pass
