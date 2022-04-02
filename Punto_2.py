#!/usr/bin/env python3
import rospy
import numpy as np
from gpiozero import LED
from std_msgs.msg import String
import pickle
from scipy import integrate
import pickle
import time
import matplotlib.pyplot as plt

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

def send_info_arduino(data):
    msg = data.data

    #Se guarda info cada vez que se actualiza
    file = open('datos_actuales.pkl', 'wb')
    pickle.dump(msg, file)
    file.close()

    info = msg.split(",")

    vel_lineal = float(info[0])
    vel_angular = float(info[1])
    key = info[2]

    print("Vel_lineal: ", vel_lineal, "\n Vel_angular: ", vel_angular, "\n Pressed key: ", key)

    graficar(key)
    on_press(key)

def posicion_estmiada(Vr, Vl):
    global x_est
    global y_est
    global theta_est
    global t_est
    global vr_list_est
    global vl_list_est
    global l
    global rw
    global t0

    dt = abs(time.time() - t_est[-1] - t0)

    # geometria del robot
    l = 15.49 #cm

    #Se actualiza la lista del tiempo
    t_est = np.append(t_est, t[-1]+dt)

    #Se actualiza la lista de las velocidades
    vr_list_est = np.append(vr_list_est, Vr )
    vl_list_est = np.append(vl_list_est, Vl)

    #Se actualizan los thetas
    theta_est = np.append(theta_est, theta_est[-1] + integrate.simpson(vr_list_est-vl_list_est, t_est)/l)

    #Se actualizan los valores odométricos
    x_est = np.append(x_est,  x_est[-1] + 0.5* (Vr+Vl) * np.cos(theta_est[-1]) * dt)
    y_est = np.append(y_est,  y_est[-1] + 0.5* (Vr+Vl) * np.sin(theta_est[-1]) * dt)

def posicion_real(Vr, Vl):
    global x
    global y
    global theta
    global t
    global vr_list
    global vl_list
    global l
    global rw
    global t0

    dt = abs(time.time() - t[-1] - t0)

    #Se actualiza la lista del tiempo
    t = np.append(t, t[-1]+dt)

    #Se actualiza la lista de las velocidades
    vr_list = np.append(vr_list, Vr )
    vl_list = np.append(vl_list, Vl)

    #Se actualizan los thetas
    theta = np.append(theta, theta[-1] + integrate.simpson(vr_list-vl_list, t)/l)

    #Se actualizan los valores odométricos
    x = np.append(x,  x[-1] + 0.5* (Vr+Vl) * np.cos(theta[-1]) * dt)
    y = np.append(y,  y[-1] + 0.5* (Vr+Vl) * np.sin(theta[-1]) * dt)

def calcular_velocidad_ruedas(v_lineal, v_angular):
    global l
    global rw

    #La estimación de la velocidad de las ruedas se basa en modelo planteado en https://www.redalyc.org/pdf/849/84903803.pdf

    a = np.array([[0.5, 0.5], [1/(2*l), -(1/(2*l))]])
    vel_robot = np.array([[v_lineal],[v_angular]])
    vel_ruedas = np.dot(np.linalg.inv(a),vel_robot)

    Vr = vel_ruedas[0]
    Vl = vel_ruedas[1]

    return Vr, Vl


def graficar(key):
    global ord_prev

    #Se obtienen velocidades de encoders
    Vr_real = 40 #Acá el cod de Lau
    Vl_real = 40

    #Se cargan los datos de las velocidades en topico de ros
    file = open('datos_actuales.pkl', 'rb')
    msg = pickle.load(file)
    file.close()

    #Se obtiene el valor teórico de velocidad lineal y angular
    info = msg.split(",")
    v_lineal = float(info[0])
    v_angular = float(info[1])

    if ord_prev=="w":
        v_lineal_final = v_lineal
        v_angular_final = 0
    elif ord_prev=="a":
        v_lineal_final = 0
        v_angular_final = -v_angular
    elif ord_prev=="d":
        v_lineal_final = 0
        v_angular_final = v_angular
    elif ord_prev=="s":
        v_lineal_final = -v_lineal
        v_angular_final = 0
    else:
        v_lineal_final = 0
        v_angular_final = 0

    #Se estima la velocidad de cada una de las ruedas
    Vr_est = calcular_velocidad_ruedas(v_lineal_final, v_angular_final)[0]
    Vl_est = calcular_velocidad_ruedas(v_lineal_final, v_angular_final)[1]

    #Se actualizan valores tanto reales como estimados
    posicion_real(Vr_real, Vl_real)
    posicion_estmiada(Vr_est, Vl_est)

    ord_prev = key
    print("orden ejecutada en el tiempo: ", ord_prev)

    #Se grafica y compara
    global x
    global x_est
    global y
    global y_est

    try:
        plt.figure()
        plt.plot(x, y, legend="Real")
        plt.plot(x_est, y_est, legend="Estimado")
        plt.title("Ubicación del robot")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
    except:
        print("No se pudo graficar por ausencia de GUI \n Real")
        print(x)
        print(y)
        print("\n Estimado \n")
        print(x_est)
        print(y_est)


def inicio():
    print("Inicializando subscriber...")
    input("Presione [ENTER] Para continuar...")

    while True:
        # Se lee la info del nodo
        rospy.init_node('robot_interface', anonymous=True)
        rospy.Subscriber("turtlebot_cmdVel", String, send_info_arduino)
        rospy.spin()

if __name__=='__main__':
    led_up = LED(23) #Verde
    led_down = LED(24) #Naranja
    led_right = LED(25) #Blanco
    led_left = LED(22) #Morado

    #Se inicializa el tiempo
    t0 = time.time()

    #Se inicializan parámetros de odometria estimados
    x_est = np.array([0])
    y_est = np.array([0])
    theta_est = np.array([0])
    t_est = np.array([0])
    vr_list_est = np.array([0])
    vl_list_est = np.array([0])

    #Se inicializan parámetros de odometria basado en encoders
    x = np.array([0])
    y = np.array([0])
    theta = np.array([0])
    t = np.array([0])
    vr_list = np.array([0])
    vl_list = np.array([0])

    # geometria del robot
    l = 15.49 #cm
    rw = 6.64 / 2 #cm

    #Orden previa a tener en cuenta para calculos odométricos
    ord_prev = "p"

    inicio()