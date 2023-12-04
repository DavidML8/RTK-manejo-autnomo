#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64



def publishMethod():
    while not rospy.is_shutdown():
        publishLat = 19.017262
        publishLong = -98.242303
        pub_lat.publish(publishLat)
        rospy.loginfo("Latitude is being sent")
        pub_long.publish(publishLong)
        rospy.loginfo("Longitude is being sent")
        rate.sleep()    # wait


if __name__ == '__main__':
    pub_lat = rospy.Publisher('latitude', Float64, queue_size=10)
    pub_long = rospy.Publisher('longitude', Float64, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(1)
    try:
        publishMethod()
    except rospy.RateROSInterruptException:
        pass
