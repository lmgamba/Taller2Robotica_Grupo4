#!/usr/bin/env python3
import numpy as np
from tkinter import *
from tkinter import Tk, Frame, Button, Label, ttk, filedialog
from mpl_toolkits.mplot3d import Axes3D
import rospy
import matplotlib.pyplot as plt
plt.style.use('dark_background')
import matplotlib.animation as animation
from std_msgs.msg import String
import shutil
import pyautogui
# Crea la biblioteca necesaria para el lienzo
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
import threading


x_data = []
y_data = []
z_data = []

VelDeseada=""
PosDeseada=""

ventana = Tk()
ventana.geometry('400x150')
ventana.wm_title('Interfaz Manipulador')
ventana.minsize(width=400, height=200)
ventana.configure(background='#1F1F1F')
# Crea un contenedor, el fondo cuando no hay lienzo
frame1 = Frame(ventana, bg='#1F1F1F')
frame1.pack(expand=1, fill='both')
# fig = plt.figure(edgecolor='green')
# ax = fig.add_subplot(projection='3d')
# canvas = FigureCanvasTkAgg(fig, master=frame1)
# canvas.get_tk_widget().pack(padx=5, pady=5, expand=1, fill='both')



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
		print(PosDeseada)
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
		print(VelDeseada)
		VelDes=VelDeseada
		print(VelDes)
		rospy.loginfo(VelDes)
		pubVel.publish(VelDes)
		newWindowV.destroy()

	Button(newWindowV, text="Enviar", command=CloseVel).place(x=200,y=300)


def guardar_imagen():
	x, y = ventana.winfo_rootx(), ventana.winfo_rooty()
	w, h = ventana.winfo_width(), ventana.winfo_height()
	screenshot=pyautogui.screenshot(region=(x, y, w, h))
	file_path = filedialog.asksaveasfilename(defaultextension='.png')
	screenshot.save(file_path)


# Mostrar lienzo
Button(frame1, text="Especificar velocidades",width=19,fg="white",background="black",command=WindowVelocidad).pack(pady=10,side="bottom")
Button(frame1, text="Guardar Imagen",width=16,fg="white",background="black",command=guardar_imagen).pack(side="bottom")
Button(frame1, text="Posición deseada",width=18,fg="white",background="black",command=WindowPosicion).pack(pady=10,side="bottom")




def interfaz():
	a=1
	# while not rospy.is_shutdown():
	#
	# 	rospy.loginfo(PosDeseada)
	# 	pubPos.publish(PosDeseada)

if __name__ == '__main__':
	pubPos = rospy.Publisher('robot_Pos', String, queue_size=10)
	pubVel = rospy.Publisher('robot_Vel', String, queue_size=10)
	rospy.init_node('robot_manipulator_interface', anonymous=True)
	interfaz()
	ventana.mainloop()
