#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64

puerto_serial = serial.Serial('/dev/ttyUSB0', 9600)


def subscriberCallBackDireccion(data):
    rospy.loginfo(rospy.get_caller_id() + " I received angle -- %s", data.data)
    global AMRangulo
    AMRangulo = data.data * 1


def subscriberCallBackVelocidad(data):
    rospy.loginfo(rospy.get_caller_id() + " I received velocity -- %s", data.data)
    global AMRvelocidad
    AMRvelocidad = data.data * 1


def subscriberCallBackLong(dataLong):
    rospy.loginfo(rospy.get_caller_id() + " I received longitude -- %s", dataLong.data)
    global longitud
    longitud = dataLong.data * 1


def subscriberCallBackLat(dataLat):
    rospy.loginfo(rospy.get_caller_id() + " I received latitude -- %s", dataLat.data)
    global latitud
    latitud = dataLat.data * 1


def subscriberCallBackOrientation(dataAngle):
    rospy.loginfo(rospy.get_caller_id() + " I received orientation -- %s", dataAngle.data)
    global magnetometro
    magnetometro = dataAngle.data * 1


def listener():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("direccionAMR", Int16, subscriberCallBackDireccion)
    rospy.Subscriber("velocidadAMR", Int16, subscriberCallBackVelocidad)
    rospy.Subscriber("longitude", Float64, subscriberCallBackLong)
    rospy.Subscriber("latitude", Float64, subscriberCallBackLat)
    rospy.Subscriber("angulo_magnetometro", Float64, subscriberCallBackOrientation)

    rate = rospy.Rate(1)
    if not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    while True:
        listener()

        giro = AMRangulo
        velocidad = AMRvelocidad
        direccion = magnetometro
        latitude = latitud
        longitude = longitud

        puerto_serial.write(bytes(str(giro) + ',' + str(velocidad) + ',' + str(longitude) + ',' + str(latitude) + ',' + str(direccion) + '\n', 'utf-8'))
        print(str(giro) + ',' + str(velocidad) + '\n', 'utf-8')
        time.sleep(5)
