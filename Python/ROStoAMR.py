#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import Int16


puerto_serial = serial.Serial('/dev/ttyACM2', 115200, write_timeout=0.1)  # Cambia '/dev/ttyUSB0' al puerto correcto
time.sleep(2)


def subscriberCallBackDireccion(dataAngle):
    rospy.loginfo(rospy.get_caller_id() + " I received angle -- %s", dataAngle.data)
    global AMRangulo
    AMRangulo = dataAngle.data * 1


def subscriberCallBackVelocidad(dataVelocity):
    rospy.loginfo(rospy.get_caller_id() + " I received velocity -- %s", dataVelocity.data)
    global AMRvelocidad
    AMRvelocidad = dataVelocity.data * 1


def listener():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("direccionAMR", Int16, subscriberCallBackDireccion)
    rospy.Subscriber("velocidadAMR", Int16, subscriberCallBackVelocidad)

    rate = rospy.Rate(2)
    if not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    AMRangulo = 50
    AMRvelocidad = 0
    while True:
        listener()

        print(AMRangulo)
        giro = AMRangulo
        print(AMRvelocidad)
        velocidad = AMRvelocidad

        puerto_serial.write(bytes(str(giro) + ',' + str(0) + '\n', 'utf-8'))
        puerto_serial.flushOutput()
