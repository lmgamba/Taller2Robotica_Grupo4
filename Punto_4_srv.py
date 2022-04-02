#!/usr/bin/env python3
from robotica_pkg.srv import recorrido_predefinido, recorrido_predefinidoResponse
from geometry_msgs.msg import Twist
import numpy as np
import time
from gpiozero import LED
from std_msgs.msg import String
import rospy
import RPi.GPIO as GPIO   # Import the GPIO library.



def on_press(key):
    global led_up
    global led_down
    global led_left
    global led_right

    if key == 'w':
        print('Arriba')
        led_up.toggle()
        led_down.off()
        led_left.off()
        led_right.off()

    if key == 's':
        print('Abajo')
        led_up.off()
        led_down.toggle()
        led_left.off()
        led_right.off()

    if key == 'a':
        print('Derecha')
        led_up.off()
        led_down.off()
        led_left.off()
        led_right.toggle()

    if key == 'd':
        print('Izquierda')
        led_up.off()
        led_down.off()
        led_left.toggle()
        led_right.off()

    if key == 'p':
        print('Stop')
        led_up.off()
        led_down.off()
        led_left.off()
        led_right.off()




def calcular_velocidad_ruedas(v_lineal, v_angular):
    global l

    #La estimación de la velocidad de las ruedas se basa en modelo planteado en https://www.redalyc.org/pdf/849/84903803.pdf

    a = np.array([[0.5, 0.5], [1/(2*l), -(1/(2*l))]])
    vel_robot = np.array([[v_lineal],[v_angular]])
    vel_ruedas = np.dot(np.linalg.inv(a),vel_robot)

    #Se estiman las velocidades de las ruedas en cm/s
    Vr = vel_ruedas[0]
    Vl = vel_ruedas[1]

    #Se mapean en las magnitudes enviadas para PWM

    Vr = 255*Vr/83.77
    Vl = 255 * Vl / 83.77

    return Vr, Vl

def inicio(ordenes, tiempos):
    global  t0

    running = True
    i = 1
    limit = len(ordenes)

    print("Ejecutando recorrido ...")
    while running:
        t_local = time.time() - t0
        if i < limit:
            if t_local > tiempos[i]:
                key = ordenes[i]
                on_press(key)
                i += 1
            else:
                pass
        else:
            running = False
    print("Finalizando ejecución de recorrido...")


def lectura_ord(file_name):
    f = open(file_name, "r")
    if f.mode == 'r':
        data = f.read()
        info = data.split("/")

        tiempos = np.array(info[0].split(";"))
        tiempos = tiempos.astype("float")
        ordenes = info[1].split(";")

        print("Ordenes leidas: ", ordenes)
        print("Tiempo leidas: ", tiempos)
        res = input("Presione [ENTER] para iniciar la simulacion")
        return ordenes, tiempos


def handle_recorrido_predefinido(req):
    global t0

    t0 = time.time()
    print("Entró acá")  # El cod NO llega hasta acá
    file_name = req.nombre_archivo
    file_name += ".txt"
    Input_velocidad_lineal = float(req.velocidad_lineal)
    Input_velocidad_angular = float(req.velocidad_angular)

    vruedas =calcular_velocidad_ruedas(Input_velocidad_lineal, Input_velocidad_angular)
    Vr = vruedas[0]
    Vl = vruedas[1]

    ordenes = lectura_ord(file_name)[0]  # Lee el archivo txt y arregla una lista
    tiempos = lectura_ord(file_name)[1]

    inicio(ordenes, tiempos)  # Las ejecuta como el punto 1
    print("Fin de simulación")
    return sim_datos_userResponse()


def sim_datos_user_server():  # Función ejecitada en el main
    rospy.init_node('robot_player')
    s = rospy.Service('recorrido_predefinido', recorrido_predefinido, handle_recorrido_predefinido)
    print("Ready to start simulation")
    rospy.spin()

if __name__ == "__main__":
    led_up = LED(23) #Verde
    led_down = LED(24) #Naranja
    led_right = LED(25) #Blanco
    led_left = LED(22) #Morado

    t0 = 0
    l = 15.49 #cm


    sim_datos_user_server()