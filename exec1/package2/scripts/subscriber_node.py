#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def publish_result(data, q=0.15):
    """
    A callback that recieves data from one topic
    and publishes to different topic
    """
    num = data.data / q
    # It doesn't let me use f-strings for some reason
    rospy.loginfo('publishing {} to /kthfs/result'.format(num))
    # Not sure if this what is meant in the exercise description
    # or if I should have made a whole other publisher_node for this
    pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
    pub.publish(num)


def listener():
    """A Subscriber node that caclulates a result based on what is publihsed to /Oberg"""
    rospy.init_node('Subscriber_node', anonymous=True)
    rospy.Subscriber('/Oberg', Float32, publish_result)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

