#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float64


def leer_puerto_serial(puerto, velocidad):
    try:
        ser = serial.Serial(puerto, velocidad)
        print(f"Leyendo desde {puerto} a {velocidad} bps. Presiona Ctrl+C para detener.")
        
        palabra = ""
        
        while True:
            dato = ser.read()
            
            dato_decodificado = dato.decode('utf-8' , errors='replace')
            palabra += dato_decodificado
            
            if dato_decodificado == '\n':
                print(palabra.strip())
                angulo = float(palabra)
                palabra = ""

                rospy.loginfo(angulo)
                pub_angle.publish(angulo)
                
    except KeyboardInterrupt:
        print("Lectura del puerto serial detenida.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()


if __name__ == '__main__':
    pub_angle = rospy.Publisher('angulo_magnetometro', Float64, queue_size=1)
    rospy.init_node('pose_angle', anonymous=True)
    rate = rospy.Rate(10)

    puerto_serial = '/dev/ttyACM1'
    velocidad_comunicacion = 9600

    leer_puerto_serial(puerto_serial, velocidad_comunicacion)
