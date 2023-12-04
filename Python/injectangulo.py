#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
valor = 0

def publishMethod():
    global valor
    while not rospy.is_shutdown():
        publishString = valor
        valor = valor +1
        if valor >= 360:
            valor = 0
        rospy.loginfo("Orientation is being sent")
        pub.publish(publishString)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('angulo_magnetometro', Float64, queue_size=1)
        rospy.init_node('publisher_node', anonymous=True)
        rate = rospy.Rate(20)
        publishMethod()
    except rospy.RateROSInterruptException:
        pass