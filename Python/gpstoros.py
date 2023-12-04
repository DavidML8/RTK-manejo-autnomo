#!/usr/bin/env python

import serial
from pynmea2 import parse
import rospy
from std_msgs.msg import Float64


def leer_puerto_serial(puerto, velocidad):
    try:
        ser = serial.Serial(puerto, velocidad)
        print(f"Leyendo desde {puerto} a {velocidad} bps. Presiona Ctrl+C para detener.")
        
        palabra = ""
        
        while True:
            dato = ser.read()
            dato_decodificado = dato.decode('utf-8', errors='replace')
            palabra += dato_decodificado
            
            if dato_decodificado == '\n':
                if palabra.startswith('$GNGLL'):
                    nmea_data = parse(palabra)

                    lat = float(nmea_data.latitude)
                    long = float(nmea_data.longitude)
                    print(f'Latitud: {lat}, Longitud: {long}')

                    rospy.loginfo(lat)
                    pub_lat.publish(lat)

                    rospy.loginfo(long)
                    pub_long.publish(long)

                    rate.sleep()
                palabra = ""
    except KeyboardInterrupt:
        print("Lectura del puerto serial detenida.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()


if __name__ == '__main__':

    pub_lat = rospy.Publisher('latitud', Float64, queue_size=1)
    pub_long = rospy.Publisher('longitud', Float64, queue_size=1)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(8)
    
    puerto_serial = '/dev/ttyACM0'
    velocidad_comunicacion = 9600

    leer_puerto_serial(puerto_serial, velocidad_comunicacion)
