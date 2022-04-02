#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def inicio():
    global Input_velocidad_lineal
    global Input_velocidad_angular

    print("Inicializando control manual de robot...")
    input("Presione [ENTER] Para continuar...")
    key = "init"

    # Se inicializa el nodo donde se enviará la info - Topico
    pub = rospy.Publisher('turtlebot_cmdVel', String, queue_size=10)
    rospy.init_node('robot_teleop_pub', anonymous=True)
    rate = rospy.Rate(10)

    while not key == "exit":
        key = input("Enter a new order (W,S,A,D): ")

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

    Input_velocidad_lineal = input('Ingresar Velocidad lineal entre 0 y 83.77 [cm/s]: ')
    Input_velocidad_angular = input('Ingresar Velocidad angular entre 0 y 5.4 [rad/s]: ')

    inicio()