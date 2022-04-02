#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import String
import time

def inicio():
    global Input_velocidad_lineal
    global Input_velocidad_angular
    global saving
    global file_name
    global t0

    count = 0

    print("Inicializando control manual de robot...")
    input("Presione [ENTER] Para continuar...")
    key = "init"

    # Se inicializa el nodo donde se enviará la info - Topico
    pub = rospy.Publisher('turtlebot_cmdVel', String, queue_size=10)
    rospy.init_node('robot_teleop_pub', anonymous=True)
    rate = rospy.Rate(10)

    list_time = np.array([0])
    list_orders = np.array(['p'])

    while not key == "exit":
        key = input("Enter a new order (W,S,A,D): ")

        t_global = time.time()
        t_local = abs(t0 - t_global)

        list_time = np.append(list_time, t_local)
        list_time = list_time.astype("str")
        list_orders = np.append(list_orders, key)
        list_orders = list_orders.astype("str")

        if saving:
            times = ";".join(list_time)
            orders = ";".join(list_orders)
            to_file = times + "/"+orders + "/"

            #Se guarda el archivo
            output_file = open(file_name+".txt", 'w')
            output_file.write(to_file)
            output_file.close()

        msg = Input_velocidad_lineal + "," + Input_velocidad_angular + "," + key

        # Se envia la información al nodo
        rospy.loginfo(msg)
        pub.publish(msg)

        rate.sleep()

        if not key == "exit":
            print("Ejecutando orden ", key)
            print("Esperando nueva orden...")
        else:
            print("Finalizando codigo...")

if __name__=='__main__':

    Input_velocidad_lineal = input('Ingresar Velocidad lineal entre 0 y 70:')
    Input_velocidad_angular = input('Ingresar Velocidad angular entre 0 y 180:')

    t0 = time.time()
    answer = input("Desea guardar el recorrido a realizar? y/n: ")

    if answer == "y":
        saving = True
        file_name = input("Ingrese el nombre del archivo de texto a utilizar (sin extensión txt): ")
    else:
        saving = False
        file_name = None

    inicio()