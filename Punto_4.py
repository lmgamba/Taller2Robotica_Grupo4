#!/usr/bin/env python3
##
import cv2
import os
import numpy as np

cap = cv2.VideoCapture(0)
radius=10

# Se definen los filtros
azulBajo = np.array([100, 100, 20], np.uint8)
azulAlto = np.array([125, 255, 255], np.uint8)

amarilloBajo = np.array([15, 100, 20], np.uint8)
amarilloAlto = np.array([45, 255, 255], np.uint8)


redBajo1 = np.array([0,100,20],np.uint8)
redAlto1 = np.array([5,255,255],np.uint8)
redBajo2 = np.array([175,100,20],np.uint8)
redAlto2 = np.array([179,255,255],np.uint8)

def colocar_contornos(contornos, color):
    global frame
    for i in range(len(contornos)):
        area = cv2.contourArea(contornos[i])
        if area > 10000:
            ((x, y), r) = cv2.minEnclosingCircle(contornos[i])
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 1)
            print(color, 'x =', x-320, 'y =', y-235, "radio ", r)

i = 0
while True:
    i+=1
    _, frame = cap.read()

    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_azul = cv2.inRange(frameHSV, azulBajo, azulAlto)
    mask_amarillo = cv2.inRange(frameHSV, amarilloBajo, amarilloAlto)
    mask_red1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    mask_red2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(mask_red1, mask_red2)

    contornos_amarillo, _ = cv2.findContours(mask_amarillo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos_azul, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos_rojo, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    colocar_contornos(contornos_amarillo, "Amarillo -----------------")
    colocar_contornos(contornos_azul, "Azul -----------------")
    colocar_contornos(contornos_rojo, "Rojo -----------------")

    cv2.imshow('Camera', frame)
    if(cv2.waitKey(1) & 0xFF == ord('q')) or i==1000:
        break

cap.release()
cv2.destroyAllWindows()

##

