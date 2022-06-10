#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading
import time
import os
import pytesseract
import cv2
import math

###################################################### Lectura palabra

def lectura_Palabra():
    cap = cv2.VideoCapture(0)
    leido, frame = cap.read()

    if leido == True:
        cv2.imwrite("Foto.png", frame)
        print("Foto tomada correctamente")
    else:
        print("Error al acceder a la cámara")

    cap.release()
    im = Image.open("/home/robotica/catkin_ws/Foto.png")
    texto = pytesseract.image_to_string(im)

    print("Texto leido:", texto)

###################################################### Brazos

def send_servos_instructions(instruction):
    Pub_Servos_Instruction = rospy.Publisher('Servos_Instruction', String, queue_size=10)
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        Pub_Servos_Instruction.publish(Instruction_Servos)
        i += 1
        if i == 10:
            break
        rate.sleep()


def ubicar_pelota_selec(pelota_seleccionada):
    global ubicacion_pelots
    cap = cv2.VideoCapture(0)
    leido, frame = cap.read()

    radius = 10

    # Se definen los filtros
    azulBajo = np.array([85, 100, 20], np.uint8)
    azulAlto = np.array([125, 255, 255], np.uint8)

    amarilloBajo = np.array([15, 90, 20], np.uint8)
    amarilloAlto = np.array([25, 255, 255], np.uint8)

    redBajo1 = np.array([0, 100, 20], np.uint8)
    redAlto1 = np.array([5, 255, 255], np.uint8)
    redBajo2 = np.array([165, 100, 20], np.uint8)
    redAlto2 = np.array([179, 255, 255], np.uint8)

    def colocar_contornos(frame, contornos, color):
        global ubicacion_pelots
        for i in range(len(contornos)):
            area = cv2.contourArea(contornos[i])
            if area > 1000:
                ((x, y), r) = cv2.minEnclosingCircle(contornos[i])
                pos_x = round(x - np.shape(frame)[0] / 2 - 140)
                pos_y = round(y - np.shape(frame)[1] / 2 + 180)
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 1)
                print(color, 'x =', pos_x, 'y =', pos_y, "radio ", round(r))
                ubicacion = None
                if pos_x < -200:
                    ubicacion = "Izquierda"
                elif pos_x > 200:
                    ubicacion = "Derecha"
                elif pos_x >= -200 and pos_x <= 200:
                    ubicacion = "Centro"
                    if abs(round(r) - 104) < 5:
                        print("El robot esta lo suficientemente cerca!!")
                    else:
                        print("El robot esta muy lejos!!")
                    if abs(pos_x) < 10:
                        print("Estś centrado!")
                    else:
                        print("No está centrado")
                else:
                    ubicacion = "No definida"
                ubicacion_pelots[color] = ubicacion
        return ubicacion_colores

    scale_percent = 40  # percent of original size
    width = int(np.shape(frame)[1] * scale_percent / 100)
    height = int(np.shape(frame)[0] * scale_percent / 100)
    dim = (width, height)

    frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_azul = cv2.inRange(frameHSV, azulBajo, azulAlto)
    mask_amarillo = cv2.inRange(frameHSV, amarilloBajo, amarilloAlto)
    mask_red1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    mask_red2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(mask_red1, mask_red2)

    contornos_amarillo, _ = cv2.findContours(mask_amarillo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos_azul, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos_rojo, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    colocar_contornos(frame, contornos_amarillo, "Amarillo")
    colocar_contornos(frame, contornos_azul, "Azul")
    colocar_contornos(frame, contornos_rojo, "Rojo")

    return ubicacion_pelots


###################################################### Atrapar pelota

def recoger_pelota():
    global pelota_seleccionada
    ubicaciones_pelotas = ubicar_pelota_selec(pelota_seleccionada)
    orden_pelota = ubicaciones_pelotas[pelota_seleccionada]
    send_servos_instructions(orden_pelota)


###################################################### Lanzar pelota

def Lanzar_pelota():
    orden_pelota = "Lanzar"
    send_servos_instructions(orden_pelota)


###################################################### Callback conteo Stops

def callback_conteo_stop(data_Stop):
    global count_Stop
    global previous_Stop
    global Stop
    print("en callback conteo")
    if previous_Stop == False and data_Stop.data == True:
        count_Stop += 1
        Stop = True
        print("ya contó")
    else:
        Stop = False
    previous_Stop = data_Stop.data
    return False

###################################################### Movimiento Robot

def control_Motors(Motors_Izq_Vel, Motors_Der_Vel, Motors_time):
    print("En control motors")
    if Stop == True:
        Instruction_Motors.linear.x = 0
        Instruction_Motors.linear.y = 0
        Instruction_Motors.linear.z = 0

    else:
        Instruction_Motors.linear.x = Motors_Izq_Vel
        Instruction_Motors.linear.y = Motors_Der_Vel
        Instruction_Motors.linear.z = Motors_time

    rospy.loginfo(Instruction_Motors)
    Pub_Motors_Intructions.publish(Instruction_Motors)
    time.sleep(Motors_time + 3)
    #time.sleep(2)
    return False


def ajuste_Translacional():
    global Motors_Izq_Vel
    global Motors_Der_Vel
    global Motors_time
    

    Vel_Ang =1 # Deg/S
    L = 15 + 2.5  # Cm
    Step = 1 # Cm

    Motors_Izq_Vel = 110
    Motors_Der_Vel = 110
    Motors_time =0.000005# S

    print("En ajuste translacional")
    control_Motors(Motors_Izq_Vel, Motors_Der_Vel, Motors_time)

    return False


def calculo_Ajuste_Rotacional(Step_Position_x, Step_Position_y, Step_Position_Theta, Robot_Position_x, Robot_Position_y,
                              Robot_Position_Theta):
    global Motors_Izq_Vel
    global Motors_Der_Vel
    global Motors_time
    vel=110
    tiempo90=5
    tiempo180=10

    Vel_Ang = 1  # Deg/S
    L = 15 + 2.5  # Cm
    Deltax=Step_Position_x-Robot_Position_x
    Deltay=Step_Position_y-Robot_Position_y
    print(Step_Position_x)
    print(Step_Position_y)
    if Deltax >0 :
    	Delta_Theta=-90
    	Delta_Time = abs(Delta_Theta) /(2*vel)  # S
    	Motors_Izq_Vel=vel
    	Motors_Der_Vel=-vel
    elif Deltax <0:
    	Delta_Theta=90
    	Delta_Time = abs(Delta_Theta) /(2*vel)   # S
    	Motors_Izq_Vel=-vel
    	Motors_Der_Vel=vel
    else:
    	if Deltay <0:
    		Delta_Theta=180
    		Delta_Time=abs(Delta_Theta) /(2*vel)  # S
    		Motors_Izq_Vel=vel
    		Motors_Der_Vel=-vel
    	else:
    		Delta_Theta=0
    		Delta_Time=0
    		Motors_Izq_Vel=0
    		Motors_Der_Vel=0
    #Delta_Theta = math.atan((Deltay/Deltax)-Robot_Position_Theta # Deg
    
    #Motors_Izq_Vel = -Vel_Ang * L
    #Motors_Der_Vel = Vel_Ang * L
    #Motors_time = Delta_Time
    print("CALCULO ajuste rotacional")

    return Motors_Izq_Vel, Motors_Der_Vel, Motors_time

def ajuste_Rotacional(Step_Position_x, Step_Position_y, Step_Position_Theta, Robot_Position_x, Robot_Position_y,
                      Robot_Position_Theta):
    print("ajuste rotacional")
    Motors_Izq_Vel, Motors_Der_Vel, Motors_time = calculo_Ajuste_Rotacional(Step_Position_x, Step_Position_y,
                                                                            Step_Position_Theta, Robot_Position_x,
                                                                            Robot_Position_y, Robot_Position_Theta)
    control_Motors(Motors_Izq_Vel, Motors_Der_Vel, Motors_time)
    return False

def recorrido_Lista(Paso):
    global Step_Position_x
    global Step_Position_y
    global Step_Position_Theta

    global Robot_Position_x
    global Robot_Position_y
    global Robot_Position_Theta

    datos = Paso
    if datos == "end":
        Step_Position_x = 0
        Step_Position_y = 0
        Step_Position_Theta = 0

    else:
        step = datos.split(",")

        Step_Position_x = float(step[0])
        Step_Position_y = float(step[1])
        Step_Position_Theta = float(step[2])

    ajuste_Rotacional(Step_Position_x, Step_Position_y, Step_Position_Theta, Robot_Position_x, Robot_Position_y,
                      Robot_Position_Theta)
    ajuste_Translacional()

    Pub_Paso_Actual.publish(Paso)
    print("En recorrido lista")

    return False

###################################################### Main


if __name__ == '__main__':

    pelota_Recoger = input("Ingrese cuadrante para recoger pelota (a/b/c): ")
    pelota_Lanzar = input("Ingrese cuadrante para lanzar pelota (a/b/c): ")
    pelota_seleccionada = input("Ingrese color de pelota a recoger (Amarillo/Rojo/Azul): ")

    if pelota_Recoger == "a" and pelota_Lanzar == "a":
        caso = "aa"
    elif pelota_Recoger == "a" and pelota_Lanzar == "b":
        caso = "ab"
    elif pelota_Recoger == "a" and pelota_Lanzar == "c":
        caso = "ac"
    elif pelota_Recoger == "b" and pelota_Lanzar == "a":
        caso = "ba"
    elif pelota_Recoger == "b" and pelota_Lanzar == "b":
        caso = "bb"
    elif pelota_Recoger == "b" and pelota_Lanzar == "c":
        caso = "bc"
    elif pelota_Recoger == "c" and pelota_Lanzar == "a":
        caso = "ca"
    elif pelota_Recoger == "c" and pelota_Lanzar == "b":
        caso = "cb"
    elif pelota_Recoger == "c" and pelota_Lanzar == "c":
        caso = "cc"

############################################################### Comunicación

    rospy.init_node('MotherOfMaster', anonymous=True)
    Pub_Motors_Intructions = rospy.Publisher('Motors_Instructions', Twist, queue_size=10)
    Pub_Paso_Actual = rospy.Publisher('Paso_Actual', String, queue_size=10)

############################################################### Constanstes

    nombre_Lista = "null" + ".txt"
    previous_nombre_Lista = "null" + ".txt"
    count_Stop = 0
    Stop = False
    previous_Stop = False

    Instruction_Servos = "Quieto"
    ubicacion_pelots = {"Amarillo": None, "Rojo": None, "Azul": None}

    Robot_Position_x = 0
    Robot_Position_y = 0
    Robot_Position_Theta = 0

    Step_Position_x = 0
    Step_Position_y = 0
    Step_Position_Theta = 0

    Motors_Izq_Vel = 0
    Motors_Der_Vel = 0
    Motors_time = 0
    
    Instruction_Motors = Twist()

############################################################### Recorrido

    while True:
        if count_Stop == 0:
            nombre_Lista = "null.txt"
            count_Stop = count_Stop + 1
            print("acabado 0, pasare a 1")

        # Inicio a punto antes de recoger pelota
        elif count_Stop == 1:
            nombre_Lista = "R1.txt"
            file1 = open(os.path.join(os.getcwd(), nombre_Lista), 'r')
            Lines = file1.readlines()
            count = 0
            for line in Lines:
                count += 1
                if line.strip() == "end":
                    print("Numero de filas leidas: " + str(count))
                    control_Motors(0, 0, 0)
                    break
                else:
                    print("Numero de filas leidas: " + str(count), "\n" ,line.strip())
                    recorrido_Lista("{}".format(line.strip()))
            print("acabado 1, pasare a 2")
            count_Stop = count_Stop + 1

        # Punto antes de recoger pelota a Punto de recolección
        elif count_Stop == 2:
            if caso == "aa" or caso == "ab" or caso == "ac":
                nombre_Lista = "R2A.txt"
            elif caso == "ba" or caso == "bb" or caso == "bc":
                nombre_Lista = "R2B.txt"
            elif caso == "ca" or caso == "cb" or caso == "cc":
                nombre_Lista = "R2C.txt"
            file1 = open(os.path.join(os.getcwd(), nombre_Lista), 'r')
            Lines = file1.readlines()
            count = 0
            for line in Lines:
                count += 1
                if line.strip() == "end":
                    print("Numero de filas leidas: " + str(count))
                    control_Motors(0, 0, 0)
                    break
                else:
                    print("Numero de filas leidas: " + str(count), "\n" ,line.strip())
                    recorrido_Lista("{}".format(line.strip()))
            count_Stop = count_Stop + 1

        # Punto de recolección a punto después de recoger pelota
        elif count_Stop == 3:
            #recoger_pelota()
            time.sleep(120)
            if caso == "aa" or caso == "ab" or caso == "ac":
                nombre_Lista = "R3A.txt"
            elif caso == "ba" or caso == "bb" or caso == "bc":
                nombre_Lista = "R3B.txt"
            elif caso == "ca" or caso == "cb" or caso == "cc":
                nombre_Lista = "R3C.txt"
            file1 = open(os.path.join(os.getcwd(), nombre_Lista), 'r')
            Lines = file1.readlines()
            count = 0
            for line in Lines:
                count += 1
                if line.strip() == "end":
                    print("Numero de filas leidas: " + str(count))
                    control_Motors(0, 0, 0)
                    break
                else:
                    recorrido_Lista("{}".format(line.strip()))
            count_Stop = count_Stop + 1

        # Punto despues de recoger pelota a punto de lectura
        elif count_Stop == 4:
            nombre_Lista = "R4.txt"
            file1 = open(os.path.join(os.getcwd(), nombre_Lista), 'r')
            Lines = file1.readlines()
            count = 0
            for line in Lines:
                count += 1
                if line.strip() == "end":
                    print("Numero de filas leidas: " + str(count))
                    control_Motors(0, 0, 0)
                    break
                else:
                    recorrido_Lista("{}".format(line.strip()))
            count_Stop = count_Stop + 1

        # Punto de lectura a punto de lanzamiento
        elif count_Stop == 5:
        #lectura_Palabra()
            time.sleep(30)
            ################################################## Lectura palabra
            if caso == "aa" or caso == "ba" or caso == "ca":
                nombre_Lista = "R5A.txt"
            elif caso == "ab" or caso == "bb" or caso == "cb":
                nombre_Lista = "R5A.txt"
            elif caso == "ac" or caso == "bc" or caso == "cc":
                nombre_Lista = "R5A.txt"
            file1 = open(os.path.join(os.getcwd(), nombre_Lista), 'r')
            Lines = file1.readlines()
            count = 0
            for line in Lines:
                count += 1
                if line.strip() == "end":
                    print("Numero de filas leidas: " + str(count))
                    control_Motors(0, 0, 0)
                    break
                else:
                    recorrido_Lista("{}".format(line.strip()))
            count_Stop = count_Stop + 1

        # En punto de lanzamiento
        elif count_Stop == 6:
            #Lanzar_pelota()
            time.sleep(120)
            count_Stop = count_Stop + 1

