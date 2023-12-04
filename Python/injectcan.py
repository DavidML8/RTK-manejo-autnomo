#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16


def publishMethod():
    while not rospy.is_shutdown():
        publishAngulo = 53
        publishVelocidad = 255
        pub_angulo.publish(publishAngulo)
        rospy.loginfo("Angle is being sent")
        pub_velocidad.publish(publishVelocidad)
        rospy.loginfo("Velocity is being sent")
        rate.sleep()    # wait


if __name__ == '__main__':
    pub_angulo = rospy.Publisher('direccionAMR', Int16, queue_size=10)
    pub_velocidad = rospy.Publisher('velocidadAMR', Int16, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(10)
    try:
        publishMethod()
    except rospy.RateROSInterruptException:
        pass