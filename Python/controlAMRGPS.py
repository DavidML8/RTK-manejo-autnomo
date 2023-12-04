#!/usr/bin/env python
import cv2
import numpy as np
import math
import pandas as pd
from geopy.distance import geodesic
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int16


dataFrame = pd.read_csv("/home/cuwu/catkin_ws/src/my_compas/scripts/basep.csv")
print("\nReading the CSV file...\n", dataFrame)

long = dataFrame['Longitud'].values.tolist()
lat = dataFrame['Latitud'].values.tolist()
print("Longitude: ", long)
print("min: ", min(long))
print("max: ", max(long))
print("Latitude: ", lat)
print("min: ", min(lat))
print("max: ", max(lat))
print(len(long))
print(len(lat))

cartesianasImagenX = []
cartesianasImagenY = []

cartesianasX = []
cartesianasY = []

image_path = r'/home/cuwu/catkin_ws/src/my_compas/scripts/Foto1.png'

image = cv2.imread(image_path)
height = image.shape[0]
width = image.shape[1]
print("height: ", height)
print("width: ", width)

latitud = 19.0183016
longitud = -98.2402208

magnetometro = 0

# -------------------------------------------Funciones---------------------------------------------------
def subscriberCallBackLong(dataLong):
    #rospy.loginfo(rospy.get_caller_id() + " I received longitude -- %s", dataLong.data)
    global longitud
    longitud = dataLong.data * 1
    # Process and store longitude data here as needed


def subscriberCallBackLat(dataLat):
    #rospy.loginfo(rospy.get_caller_id() + " I received latitude -- %s", dataLat.data)
    global latitud
    latitud = dataLat.data * 1
    # Process and store latitude data here as needed


def subscriberCallBackOrientation(dataAngle):
    #rospy.loginfo(rospy.get_caller_id() + " I received orientation -- %s", dataAngle.data)
    global magnetometro
    magnetometro = dataAngle.data * 1


def listener():
    rospy.Subscriber("longitud", Float64, subscriberCallBackLong)
    rospy.Subscriber("latitud", Float64, subscriberCallBackLat)
    rospy.Subscriber("angulo_magnetometro", Float64, subscriberCallBackOrientation)

    rate = rospy.Rate(20)
    if not rospy.is_shutdown():
        rate.sleep()


def publishDirection(direccionROS):
    if not rospy.is_shutdown():
        publishAMRDirection= direccionROS
        rospy.loginfo("Direction Error is being published")
        pubDirectionAMR.publish(publishAMRDirection)
        rate.sleep()


def publishVelocity(velocidadROS):
    if not rospy.is_shutdown():
        publishAMRVelocity= velocidadROS   # variable
        rospy.loginfo("Direction Error is being published")
        pubVelocityAMR.publish(publishAMRVelocity)
        rate.sleep()


def publishEROR(direccionErrorROS):
    if not rospy.is_shutdown():
        publishErrorAMR = direccionErrorROS
        rospy.loginfo("Direction Error is being published")
        pubErrorAMR.publish(publishErrorAMR)
        rate.sleep()


def polarAcartesianaP(lat, lon):
    R = 6371
    xCartesiano = R * math.cos(math.radians(lat)) * math.cos(math.radians(lon))
    yCartesiano = R * math.sin(math.radians(lat))
    xPixel = int((1778.9356 * xCartesiano) + 1536723.11)
    yPixel = int((-1878.46*yCartesiano) + 3900407.83)
    return xPixel, yPixel


def polarAcartesiana(lat, lon):
    R = 6371
    xCartesiano = R * math.cos(math.radians(lat)) * math.cos(math.radians(lon))
    yCartesiano = R * math.sin(math.radians(lat))
    return xCartesiano, yCartesiano


def cartesianaApolar(Xprox, Yprox):
    R = 6371
    xCoordenada = (Xprox-1536723.11)/1778.9356
    yCoordenada = (Yprox-3900407.83)/-1878.46
    lati = math.asin(yCoordenada/R)
    long = math.atan2(yCoordenada, xCoordenada)
    print("xC2: ", xCoordenada, " yC2: ", yCoordenada)
    # print("x: ", xPixel, " y: ", yPixel)
    return lati, long


def orientacionCarritoP(carritoX, carritoY, largo, angulo):
    orientacionX = int((largo * math.cos(math.radians(angulo))) + carritoX)
    orientacionY = int(carritoY - (largo * math.sin(math.radians(angulo))))
    return orientacionX, orientacionY


def orientacionCarrito(carritoX, carritoY, largo, angulo):
    orientacionX = (largo * math.cos(math.radians(angulo))) + carritoX
    orientacionY = carritoY + (largo * math.sin(math.radians(angulo)))
    return orientacionX, orientacionY


def anguloVectores(inicioX, inicioY, Ax, Ay, Bx, By):
    punto = np.dot([(Ax-inicioX), (inicioY-Ay)], [(Bx-inicioX), (inicioY-By)])
    distanciaA = math.sqrt(((Ax-inicioX)**2) + ((inicioY-Ay)**2))
    distanciaB = math.sqrt(((Bx-inicioX)**2) + ((inicioY-By)**2))
    distancias = distanciaA * distanciaB
    anguloRadianes = math.acos(min(max(punto / distancias, -1), 1))
    anguloV = math.degrees(anguloRadianes)
    return anguloV


def anguloDesfase(inicioX, inicioY, Ax, Ay, Bx, By):
    actual = math.degrees(math.atan2((Ay-inicioY),(Ax-inicioX)))
    objetivo = math.degrees(math.atan2((By-inicioY),(Bx-inicioX)))
    ObjetivoActual = [(Bx-inicioX), (By-inicioY)]

    if actual < 0:
        actual += 360
    if objetivo < 0:
        objetivo += 360

    theta = math.radians(-(actual-90))
    MatrixRotacion = [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    ObjetivoNuevo = np.matmul(MatrixRotacion, ObjetivoActual)
    prueba = math.degrees(math.atan2(ObjetivoNuevo[0], ObjetivoNuevo[1]))
    anguloAMR = int(np.clip((prueba + 50), 0, 100))
    return prueba, anguloAMR


def empty(a):
    pass


def coordenadasApixeles(latitudes, longitudes):
    for i in range(len(long)):
        Ximagen, Yimagen = polarAcartesianaP(latitudes[i], longitudes[i])
        cartesianasImagenX.append(Ximagen)
        cartesianasImagenY.append(Yimagen)


def dibujarCoordenadas(pixelesX, pixelesY, imagenDibujar):
    for i in range(len(pixelesX)):                               # B   G    R
        cv2.circle(imagenDibujar, (pixelesX[i], pixelesY[i]), 2, (255, 0, 255), -1)


def coordenadasACartesianas(latitudes, longitudes):
    for i in range(len(long)):
        Xcartesiana, Ycartesiana = polarAcartesiana(latitudes[i], longitudes[i])
        cartesianasX.append(Xcartesiana)
        cartesianasY.append(Ycartesiana)


coordenadasApixeles(lat, long)
coordenadasACartesianas(lat, long)
print("X pixel ", cartesianasImagenX)
print("Y pixel ", cartesianasImagenY)
print("X cartesiana ", cartesianasX)
print("Y cartesiana ", cartesianasY)

n = 0
p = 0 + n


if __name__ == '__main__':
    pubErrorAMR = rospy.Publisher('errorAMR', Float64, queue_size=1)  # defining publisher by topic, msg type, queue
    pubDirectionAMR = rospy.Publisher('direccionAMR', Int16, queue_size=1)
    pubVelocityAMR = rospy.Publisher('velocidadAMR', Int16, queue_size=1)
    rospy.init_node('subpub_node', anonymous=True)  # initializing the ros node - publish_node
    rate = rospy.Rate(5)  # 10Hz
    while True:
        listener()
        print("Longitud: ", longitud)
        print("Latitud: ", latitud)
        LatitudActual = latitud
        LongitudActual = longitud
        print("Lon Actual: ", LongitudActual)
        print("Lan Actual: ", LatitudActual)
        myFrame = image
        img = myFrame.copy()
        carritoX, carritoY = polarAcartesianaP(LatitudActual, LongitudActual)
        XcartesianaActual, YcartesianaActual = polarAcartesiana(LatitudActual, LongitudActual)
        orientacion = magnetometro
        largo = 32
        Xp, Yp = polarAcartesianaP(LatitudActual, LongitudActual)
        Xorientacion, Yorientacion = orientacionCarritoP(Xp, Yp, largo, orientacion)
        print("Angulo Cartesiano")
        XorientacionCartesiano, YorientacionCartesiano = orientacionCarrito(XcartesianaActual, YcartesianaActual, 0.025,
                                                                            orientacion)
        anguloPuntoCartesiano = anguloVectores(XcartesianaActual, YcartesianaActual, XorientacionCartesiano,
                                               YorientacionCartesiano, cartesianasX[p], cartesianasY[p])
        ErrorAnguloReal, AnguloControlAMR = anguloDesfase(XcartesianaActual, YcartesianaActual, XorientacionCartesiano,
                                                          YorientacionCartesiano, cartesianasX[p], cartesianasY[p])
        print("Angulo Error: ", ErrorAnguloReal)
        print("Angulo AMR: ", AnguloControlAMR)
        coordenadas_AMR = (LatitudActual, LongitudActual)  # Por ejemplo, Nueva York
        coordenadas_WAYPOINTS = (lat[p], long[p])  # Por ejemplo, Los Ángeles
        distanciaGeopy = geodesic(coordenadas_AMR, coordenadas_WAYPOINTS).meters
        velocidaAMR = int(np.clip((distanciaGeopy / 0.035), 0, 255))
        print("Distancia Geopy: ", distanciaGeopy)

        dibujarCoordenadas(cartesianasImagenX, cartesianasImagenY, img)
        cv2.line(img, (Xp, Yp), (Xorientacion, Yorientacion), (0, 255, 0), 1)
        cv2.line(img, (carritoX, carritoY), (cartesianasImagenX[p], cartesianasImagenY[p]), (0, 0, 255), 2)
        cv2.circle(img, (carritoX, carritoY), 2, (0, 165, 255), -1)
        cv2.rectangle(img, (0, 0), (200, 170), (255, 255, 255), -1)

        textoError = "Error: " + str(round(ErrorAnguloReal, 3))
        textoAMR = "AMR: " + str(AnguloControlAMR)
        textoDistancia = "Dist: " + str(round(distanciaGeopy, 3))
        textoVelocidad = "vAMR: " + str(velocidaAMR)

        cv2.putText(img, textoError, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoAMR, (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoDistancia, (5, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoVelocidad, (5, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('Cartesian Map', img)

        publishEROR(ErrorAnguloReal)
        publishDirection(100 - AnguloControlAMR)
        publishVelocity(velocidaAMR)

        ROI = img[(carritoY-50):(carritoY+50), (carritoX-100):(carritoX+100)]
        ROI = cv2.resize(ROI, (600, 300), interpolation=cv2.INTER_AREA)
        cv2.imshow('ROI', ROI)
        # print(cv2.getWindowProperty('Cartesian Map', cv2.WND_PROP_VISIBLE))
        if distanciaGeopy < 1:
            p += 1
        if cv2.waitKey(1) and 0xFF == ord('q'):
            break


