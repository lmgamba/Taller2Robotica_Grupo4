#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gpiozero import LED
import time
import numpy as np
import RPi.GPIO as GPIO  # Import the GPIO library.
import time


def on_press(key):
    global led_servo_1R
    global led_servo_2R
    global led_servo_3R
    global led_servo_1L
    global led_servo_2L
    global led_servo_3L
    global led_servo_4
    global angle_2_new
    global angle_1_new
    global angle_3_new

    global pub
    global rate

    if key == 'r':
        print('r')
        led_servo_1R.on()
        led_servo_2R.off()
        led_servo_3R.off()
        led_servo_1L.off()
        led_servo_2L.off()
        led_servo_3L.off()

        key = "r"

        angle_1_new = angle_1 + Var_Angle
        time.sleep(Step_1)
        led_servo_1R.off()


    if key == 'f':
        print('f')
        led_servo_1R.off()
        led_servo_2R.off()
        led_servo_3R.off()
        led_servo_1L.on()
        led_servo_2L.off()
        led_servo_3L.off()

        key = "f"
        angle_1_new = angle_1 - Var_Angle
        time.sleep(Step_1)
        led_servo_1L.off()

    if key == 't':
        print('t')
        led_servo_1R.off()
        led_servo_2R.on()
        led_servo_3R.off()
        led_servo_1L.off()
        led_servo_2L.off()
        led_servo_3L.off()

        key = "t"
        angle_2_new = angle_2 + Var_Angle
        time.sleep(Step_2)
        led_servo_2R.off()

    if key == 'g':
        print('g')
        led_servo_1R.off()
        led_servo_2R.off()
        led_servo_3R.off()
        led_servo_1L.off()
        led_servo_2L.on()
        led_servo_3L.off()

        key = "g"

        angle_2_new = angle_2 - Var_Angle
        time.sleep(Step_2)
        led_servo_2L.off()


    if key == 'y':
        print('y')
        led_servo_1R.off()
        led_servo_2R.off()
        led_servo_3R.on()
        led_servo_1L.off()
        led_servo_2L.off()
        led_servo_3L.off()

        key = "y"
        angle_3_new = angle_3 + Var_Angle
        time.sleep(Step_3)
        led_servo_3R.off()


    if key == 'h':
        print('h')
        led_servo_1R.off()
        led_servo_2R.off()
        led_servo_3R.off()
        led_servo_1L.off()
        led_servo_2L.off()
        led_servo_3L.on()

        key = "h"

        angle_3_new = angle_3 - Var_Angle
        time.sleep(Step_3)

        led_servo_3L.off()


    if key == 'e':
        print('e')
        led_servo_1R.off()
        led_servo_2R.off()
        led_servo_3R.off()
        led_servo_1L.off()
        led_servo_2L.off()
        led_servo_3L.off()
        led_servo_4.toggle()

        key = "e"

        time.sleep(Step_4)

    theta_1 = angle_2_new
    theta_2 = angle_3_new
    theta_3 = angle_1_new

    l = 15.49  # cm

    pos_x = (7.8 * np.cos(theta_1) + 8.8 * np.cos(theta_1 + theta_2) + l * np.cos(theta_1 + theta_2) + 0.9 * np.cos(
            theta_1 + theta_2)) * np.sin(theta_3)
    pos_y =  (7.8 * np.cos(theta_1) + 8.8 * np.cos(theta_1 + theta_2) + l * np.cos(theta_1 + theta_2) + 0.9 * np.cos(
            theta_1 + theta_2)) * np.cos(theta_3)
    pos_z =  7.8 * np.sin(theta_1) + 8.8 * np.sin(theta_1 + theta_2) + l * np.sin(theta_1 + theta_2) + 0.9 * np.sin(
             theta_1 + theta_2)

    msg = str(pos_x) + ";" + str(pos_y) + ";" + str(pos_z) + ";" + str(theta_1) + ";" + str(theta_2) + ";" + str(theta_3)

    save_text = open("posicion_actual.txt", 'w')
    save_text.write(msg)
    save_text.close()

    return False


def inicio():
    print("hola estoy en incio")

    while not rospy.is_shutdown():
        key = input("ingrese una orden: ")

        # listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        # listener.start()
        # listener.join()
        on_press(key)
        rate.sleep()


if __name__ == '__main__':
    # Se inicializan los LEDs

    led_servo_1R = LED(21)
    led_servo_2R = LED(20)
    led_servo_3R = LED(16)
    led_servo_1L = LED(1)
    led_servo_2L = LED(7)
    led_servo_3L = LED(8)
    led_servo_4 = LED(12)

    # Se inicializan nodos

    pub = rospy.Publisher('turtlebot_position', String, queue_size=10)
    rospy.init_node('robot_teleop_pub', anonymous=True)
    rate = rospy.Rate(10)

    # Se inicializa el tiempo
    t0 = time.time()

    # Se inicializan parámetros de odometria estimados
    x_est = np.array([0])
    y_est = np.array([0])
    z_est = np.array([0])

    # Se inicializan parámetros del brazo
    Step_1 = 0.7  # Tiempo entre pulsos (s).
    Step_2 = 0.7  # Tiempo entre pulsos (s).
    Step_3 = 0.7  # Tiempo entre pulsos (s).
    Step_4 = 0.7  # Tiempo entre pulsos (s).

    Var_Angle = 5

    angle_1 = 90
    angle_2 = 30
    angle_3 = 130
    angle_4 = 120

    angle_1_new = angle_1
    angle_2_new = angle_2
    angle_3_new = angle_3
    angle_4_new = angle_4

    # Orden previa a tener en cuenta para calculos odométricos
    ord_prev = "p"

    inicio()