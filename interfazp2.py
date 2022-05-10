#!/usr/bin/env python3
import numpy as np
from tkinter import *
from tkinter import Tk, Frame, Button, Label, ttk, filedialog
from mpl_toolkits.mplot3d import Axes3D
import rospy
import matplotlib.pyplot as plt
plt.style.use('dark_background')
from matplotlib.animation import FuncAnimation
from std_msgs.msg import String
import shutil
import pyautogui
# Crea la biblioteca necesaria para el lienzo
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
import threading


x_data = []
y_data = []
z_data = []

def interfaz():
    rospy.init_node('robot_manipulator_interface', anonymous=True)
    rospy.Subscriber("robot_manipulator_position", String, callback)
    t = threading.Thread(target=plot,daemon=True)
    t.start()

def callback(msg):
	datos=msg.data
	global x_data,y_data,z_data
	#data_str = str(datos)
	numbers_str = datos.split(",")
	datosx = numbers_str[0]
	datosy = numbers_str[1]
	datosz = numbers_str[2]

	x_data.append(datosx)
	y_data.append(datosy)
	z_data.append(datosz)

	print("---------------  ")
	print("x_data",x_data[-1])
	print("y_data",y_data[-1])
	print("z_data",z_data[-1])

def plot():
    ani = FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()


def animate(i):
    plt.plot(x_data, y_data)
    plt.xlim([-50,50])
    plt.ylim([-50,50])
    print("animation")


if __name__ == '__main__':
    interfaz()
    rospy.spin()
