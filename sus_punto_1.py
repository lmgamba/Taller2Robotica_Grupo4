#!/usr/bin/env python3
import rospy
from gpiozero import LED
from std_msgs.msg import String, Int16
import pickle
import serial
import time
import numpy as np
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
        close_pmw()

def close_pmw():
    global pmw_der
    global pmw_izq

    pmw_der.stop() #Se detiene el pulso PMW anterior
    pmw_izq.stop()

def send_arduino_pmw_info(Vr, Vl):
    global pwm_der
    global pmw_izq

    pwm_der.stop() #Se detiene el pulso PMW anterior
    pmw_izq.stop()

    dc_1 = Vr*100/83.77  #Se calcula nueva modulación
    dc_2 = Vl*100/83.77

    pwm_der.start(dc_1)  # Se inicia el nuevo
    pwm_izq.start(dc_2)

    print("Publicando...") #Se publica en tópicos correspondientes
    pub_der = rospy.Publisher('pmw_info_get_der', Int16, queue_size=10)
    rospy.loginfo(Vr)
    pub_der.publish(Vr)

    pub_izq = rospy.Publisher('pmw_info_get_izq', Int16, queue_size=10)
    rospy.loginfo(Vl)
    pub_izq.publish(Vl)
    print("Se ha piblicado con éxito")

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

def send_info_arduino(data):
    msg = data.data
    info = msg.split(",")

    vel_lineal = float(info[0])
    vel_angular = float(info[1])
    key = info[2]

    if key=="w":
        v_lineal_final = vel_lineal
        v_angular_final = 0
    elif key=="a":
        v_lineal_final = 0
        v_angular_final = -vel_angular
    elif key=="d":
        v_lineal_final = 0
        v_angular_final = vel_angular
    elif key=="s":
        v_lineal_final = -vel_lineal
        v_angular_final = 0
    else:
        v_lineal_final = 0
        v_angular_final = 0

    print("Vel_lineal: ", v_lineal_final, "\n Vel_angular: ", v_angular_final, "\n Pressed key: ", key)

    Vr = min(int(calcular_velocidad_ruedas(v_lineal_final, v_angular_final)[0]),255)
    Vl = min(int(calcular_velocidad_ruedas(v_lineal_final, v_angular_final)[1]), 255)

    send_arduino_pmw_info(abs(Vr), abs(Vl))

    on_press(key)

def inicio():
    print("Inicializando subscriber...")
    input("Presione [ENTER] Para continuar...")

    while True:
        # Se lee la info del nodo
        rospy.init_node('robot_teleop', anonymous=True)
        rospy.Subscriber("turtlebot_cmdVel", String, send_info_arduino)
        rospy.spin()

if __name__=='__main__':
    led_up = LED(23) #Verde
    led_down = LED(24) #Naranja
    led_right = LED(25) #Blanco
    led_left = LED(22) #Morado

    GPIO.setup(12, GPIO.OUT)  # Set GPIO pin 12 to output mode (PMW derecho)
    GPIO.setup(13, GPIO.OUT)  # Set GPIO pin 13 to output mode (PMW izquierdo)
    pwm_der = GPIO.PWM(12, 100)  # Initialize PWM on pwmPin 100Hz frequency
    pmw_izq = GPIO.PWM(13, 100)  # Initialize PWM on pwmPin 100Hz frequency

    #Parámetros geométricos del robot
    l = 15.49  # cm

    inicio()