#!/usr/bin/env python3
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
plt.style.use('dark_background')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from tkinter import Tk, Frame, Button, Label, ttk
from tkinter import *
import numpy as np
import rospy
import tty
import sys
import termios
from std_msgs.msg import String
import shutil
import threading
import pyautogui

# variables robot
x_data1 = []
y_data1 = []
datosx1 = 0
datosy1 = 0


# variables manipulador
x_data = []
y_data = []
z_data = []
VelDeseada=""
PosDeseada=""

datosx = 0
datosy = 0
datosz = 0

#Creacion de ventana de interfaz
ventana = Tk()
ventana.geometry('970x650')
ventana.wm_title('Interfaz Robot')
ventana.minsize(width=1300, height=700)
ventana.configure(background='#1F1F1F')
# Crea un contenedor para interfaz robot
frame1 = Frame(ventana, bg='#1F1F1F')
frame1.pack(side="left", expand=1,fill='both')
fig1 = plt.figure(edgecolor='green')
ax1 = fig1.add_subplot(111)
canvas = FigureCanvasTkAgg(fig1, master=frame1)
canvas.get_tk_widget().pack(padx=5, pady=5, expand=1, fill='both')

# Crea un contenedor par amanipulador, el fondo cuando no hay lienzo
frame2 = Frame(ventana, bg='#1F1F1F')
frame2.pack(side="right",expand=1, fill='both')
fig = plt.figure(edgecolor='green')
ax = Axes3D(fig)
canvas2 = FigureCanvasTkAgg(fig, master=frame2)
canvas2.get_tk_widget().pack(padx=5, pady=5, expand=1, fill='both')


#----------------funciones botonoes robot---------
def guardar():
	recorrido="recorrido antiguo"
	nombre_recorrido = filedialog.asksaveasfilename(defaultextension='.txt')
	np.savetxt(nombre_recorrido,recorrido,fmt="%s")


def abrir():
	global Abierto_flag
	global name_open
	Abrir_flag = True
	filename = filedialog.askopenfilename(initialdir = "catkin_ws/src/",
                                           title = "Select a File",
                                           filetypes = (("Text files",
                                                         "*.txt*"),
                                                        ("all files",
                                                         "*.*")))
	name_str = filename.split("/")
	n=len(name_str)
	print(name_str)
	name_open=name_str[n-1]
	nameAbrir=name_open
	print(nameAbrir)
	rospy.loginfo(nameAbrir)
	pubname.publish(nameAbrir)


def guardar_imagen():
	x, y = ventana.winfo_rootx(), ventana.winfo_rooty()
	w, h = ventana.winfo_width(), ventana.winfo_height()
	screenshot=pyautogui.screenshot(region=(x, y, w, h))
	file_path = filedialog.asksaveasfilename(defaultextension='.png')
	screenshot.save(file_path)

#-------------funciones botones manipulador--------
def WindowPosicion():
	newWindowP = Toplevel(ventana)
	newWindowP.title("Posición deseada")
	newWindowP.geometry("350x350")
	label1 = Label(newWindowP, text ="Ingrese las coordenadas deseadas")
	label1.place(x=90, y=40)
	Label(newWindowP, text ="x:").place(x=80, y=100)
	Label(newWindowP, text ="y:").place(x=80, y=175)
	Label(newWindowP, text ="z:").place(x=80, y=250)
	# Crear caja de texto.
	entryx = ttk.Entry(newWindowP)
	entryx.place(x=100, y=100)
	entryy = ttk.Entry(newWindowP)
	entryy.place(x=100, y=175)
	entryz = ttk.Entry(newWindowP)
	entryz.place(x=100, y=250)

	def ClosePos():
		#coordenadas input
		global VelDeseada
		Coordenadax=str(entryx.get())
		Coordenaday=str(entryy.get())
		Coordenadaz=str(entryz.get())
		PosDeseada= Coordenadax+","+Coordenaday+","+Coordenadaz
		PosDes=PosDeseada
		print(PosDes)
		rospy.loginfo(PosDes)
		pubVel.publish(PosDes)
		newWindowP.destroy()

	Button(newWindowP, text="Enviar", command=ClosePos).place(x=150,y=300)

def WindowVelocidad():
	newWindowV = Toplevel(ventana)
	newWindowV.title("Velocidad de cada motor")
	newWindowV.geometry("400x350")
	label1 = Label(newWindowV, text ="Ingrese la velocidad deseada")
	label1.place(x=80, y=50)
	Label(newWindowV, text ="Motor Base:").place(x=70, y=110)
	Label(newWindowV, text ="Motor Link 1:").place(x=70, y=160)
	Label(newWindowV, text ="Motor Link 2:").place(x=70, y=210)
	Label(newWindowV, text ="Motor Garra:").place(x=70, y=260)

	# Crear caja de texto.
	entry1 = ttk.Entry(newWindowV)
	entry1.place(x=160, y=110)
	entry2 = ttk.Entry(newWindowV)
	entry2.place(x=160, y=160)
	entry3 = ttk.Entry(newWindowV)
	entry3.place(x=160, y=210)
	entry4 = ttk.Entry(newWindowV)
	entry4.place(x=160, y=260)

	def CloseVel():
	    #velocidades input
		Velocidad1=str(entry1.get())
		Velocidad2=str(entry2.get())
		Velocidad3=str(entry3.get())
		Velocidad4=str(entry4.get())
		VelDeseada= Velocidad1+","+Velocidad2+","+Velocidad3+","+Velocidad4
		VelDes=VelDeseada
		print(VelDes)
		rospy.loginfo(VelDes)
		pubVel.publish(VelDes)
		newWindowV.destroy()

	Button(newWindowV, text="Enviar", command=CloseVel).place(x=200,y=300)

##---------callbacks------##
def callback1(msg):
	datos=msg.data
	global datosx1
	global datosy1
	data_str = str(datos)
	numbers_str1 = datos.split(",")
	datosx1 = float(numbers_str[0])
	datosy1 = float(numbers_str[1])

def callback(msg):
	datos=msg.data
	global datosx
	global datosy
	global datosz
	numbers_str = datos.split(",")
	datosx = float(numbers_str[0])
	datosy = float(numbers_str[1])
	datosz = float(numbers_str[2])

###-----plotear graficas----
def animate1(frame):
	global x_data1
	global y_data1
	global datosx1
	global datosy1
	x_data1.append(datosx1)
	y_data1.append(datosy1)

	print("---------------  ")
	print("x_data",x_data1[-1])
	print("y_data",y_data1[-1])
	ax1.plot(x_data1,y_data1)

def animate2(frame):
	global ax
	global x_data
	global y_data
	global z_data
	global datosx
	global datosy
	global datosz
	x_data.append(datosx)
	y_data.append(datosy)
	z_data.append(datosz)
	ax.plot(x_data, y_data, z_data)
	#
	print("---------------  ")
	print("x_data",x_data[-1])
	print("y_data",y_data[-1])
	print("z_data",z_data[-1])

#-------animacion tiempo real-------
def plot():
	ani1=animation.FuncAnimation(fig1,animate1,interval=1000)
	ani=animation.FuncAnimation(fig,animate2,interval=1000)
	canvas.draw()
	canvas2.draw()


#Botones taller 2
Button(frame1, text="Guardar recorrido",width=16,fg="white",background="black",command=guardar).pack(padx=40,side="left")
Button(frame1, text="Guardar Imagen",width=16,fg="white",background="black",command=guardar_imagen).pack(side="left")
Button(frame1, text="Cargar recorrido antiguo",width=25,fg="white",background="black",command=abrir).pack(padx=10,side="left")

#Botones taller 3
Button(frame2, text="Especificar velocidades",width=19,fg="white",background="black",command=WindowVelocidad).pack(padx=40,side="right")
Button(frame2, text="Posición deseada",width=18,fg="white",background="black",command=WindowPosicion).pack(padx=10,side="right")




if __name__ == '__main__':
	pubname = rospy.Publisher('robot_name', String, queue_size=10)
	pubPos = rospy.Publisher('robot_Pos', String, queue_size=10)
	pubVel = rospy.Publisher('robot_Vel', String, queue_size=10)
	rospy.init_node('robot_interface', anonymous=True)
	rospy.Subscriber("robot_position", String, callback1)
	rospy.Subscriber("robot_manipulator_position", String, callback)
	plot()
	ventana.mainloop()
	rospy.spin()
