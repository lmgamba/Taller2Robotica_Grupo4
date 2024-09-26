#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard
from gpiozero import LED
import numpy as np
import math
import RPi.GPIO as GPIO  # Import the GPIO library.
import time

# Se inicializan parámetros del brazo
Step_1 = 0.7  # Tiempo entre pulsos (s).
Step_2 = 0.7  # Tiempo entre pulsos (s).
Step_3 = 0.7  # Tiempo entre pulsos (s).
Step_4 = 0.7  # Tiempo entre pulsos (s).

a_1 = 7.8 # Base - Brazo
a_2 = 8.8  + 5 # Brazo - Pinza

Var_Angle = 5

angle_1 = 90
angle_2 = 30
angle_3 = 130
angle_4 = 120

angle_1_new = angle_1
angle_2_new = angle_2
angle_3_new = angle_3
angle_4_new = angle_4

Theta_1 = 0
Theta_2 = 0
Theta_3 = 0
Theta_4 = 0


def on_press(x,y,z):
    global led_servo_1R
    global led_servo_2R
    global led_servo_3R
    global led_servo_1L
    global led_servo_2L
    global led_servo_3L
    global led_servo_4
    global pub
    global rate

    z = z - 15

    m = math.sqrt((x ** 2) + (y ** 2))  # Plano en dirección a punto deseado

    Theta_3 = math.atan(x / y)
    Theta_2 = math.acos((1 / (2 * a_1 * a_2)) * (((m ** 2) + (z ** 2)) - ((a_1 ** 2) + (a_2 ** 2))))
    Theta_1 = math.acos((1 / ((m ** 2) + (z ** 2))) * (
                (m * (a_1 + a_2 * math.cos(Theta_2))) + (z * a_2 * math.sqrt(1 - ((math.cos(Theta_2)) ** 2)))))

    pi = math.pi
    Theta_3 = Theta_3 * 180 / pi
    Theta_2 = Theta_2 * 180 / pi
    Theta_1 = Theta_1 * 180 / pi

    Theta_1 = ((Theta_1-angle_2)/ Var_Angle + 0.5) * Var_Angle;
    Times_Theta_1 = Theta_1 / 5;
    Theta_2 = ((Theta_2-angle_3) / Var_Angle + 0.5) * Var_Angle;
    Times_Theta_2 = Theta_2 / 5;
    Theta_3 = ((Theta_3-angle_1) / Var_Angle + 0.5) * Var_Angle;
    Times_Theta_3 = Theta_3 / 5;

# Base
    for i in range(1, Times_Theta_3):
        if Theta_3 > 0:
            led_servo_1R.on()
            led_servo_2R.off()
            led_servo_3R.off()
            led_servo_1L.off()
            led_servo_2L.off()
            led_servo_3L.off()

            angle_1_new = angle_1 + Var_Angle
            time.sleep(Step_1)
            led_servo_1R.off()

        elif Theta_3 > 0:
            led_servo_1R.off()
            led_servo_2R.off()
            led_servo_3R.off()
            led_servo_1L.on()
            led_servo_2L.off()
            led_servo_3L.off()

            angle_1_new = angle_1 - Var_Angle
            time.sleep(Step_1)
            led_servo_1L.off()

        time.sleep(Step_1)
# Brazo
    for i in range(1, Times_Theta_2):
        if Theta_2 > 0:
            led_servo_1R.off()
            led_servo_2R.on()
            led_servo_3R.off()
            led_servo_1L.off()
            led_servo_2L.off()
            led_servo_3L.off()

            angle_2_new = angle_2 + Var_Angle
            time.sleep(Step_2)
            led_servo_2R.off()

        elif Theta_2 > 0:
            led_servo_1R.off()
            led_servo_2R.off()
            led_servo_3R.off()
            led_servo_1L.off()
            led_servo_2L.on()
            led_servo_3L.off()

            angle_2_new = angle_2 - Var_Angle
            time.sleep(Step_2)
            led_servo_2L.off()

        time.sleep(Step_2)
# Antebrazo
    for i in range(1, Times_Theta_1):
        if Theta_1 > 0:
            led_servo_1R.off()
            led_servo_2R.off()
            led_servo_3R.on()
            led_servo_1L.off()
            led_servo_2L.off()
            led_servo_3L.off()

            angle_3_new = angle_3 + Var_Angle
            time.sleep(Step_3)
            led_servo_3R.off()

        elif Theta_1 > 0:
            led_servo_1R.off()
            led_servo_2R.off()
            led_servo_3R.off()
            led_servo_1L.off()
            led_servo_2L.off()
            led_servo_3L.on()

            angle_3_new = angle_3 - Var_Angle
            time.sleep(Step_3)
            led_servo_3L.off()

# Pinza

    return False



def inicio():
    x = float(input("Ingrese x deseado: "))
    y = float(input("Ingrese y deseado: "))
    z= float(input("Ingrese z deseado: "))

    on_press(x, y, z)




if __name__ == '__main__':
    led_servo_1R = LED(21)
    led_servo_2R = LED(20)
    led_servo_3R = LED(16)
    led_servo_1L = LED(1)
    led_servo_2L = LED(7)
    led_servo_3L = LED(8)
    led_servo_4 = LED(12)

    pub = rospy.Publisher('turtlebot_position', String, queue_size=10)
    rospy.init_node('robot_teleop_pub', anonymous=True)
    rate = rospy.Rate(10)

    x_est = np.array([0])
    y_est = np.array([0])
    z_est = np.array([0])

    ord_prev = "p"

    inicio()
