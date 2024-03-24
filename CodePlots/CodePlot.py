import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
from tkinter import Tk, Frame, StringVar, Label, Button
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

isReceiving = False
isRun = True
datos = 0.0
muestraD = 100
data = collections.deque([0] * muestraD, maxlen=muestraD)
xmin = 0
xmax = muestraD
ymin = 0
ymax = 200  # Ajusta el rango según tus necesidades

try:
    arduino = serial.Serial("COM6", 9600, timeout=1)
except Exception as e:
    print("Error de conexión con el puerto:", e)

def Iniciar():
    global thread
    thread = Thread(target=DatosA)
    print("Iniciando hilo...")
    thread.start()

def DatosA():
    global datos
    try:
        time.sleep(1)
        arduino.reset_input_buffer()
        print("Hilo iniciado correctamente.")
        while isRun:
            lectura = arduino.readline().decode('utf-8').strip()
            if lectura:
                datos = float(lectura)
                print("Distancia recibida:", datos)
    except Exception as e:
        print("Error en el hilo:", e)

def Salir():
    global isRun, thread
    isRun = False
    if thread:
        thread.join()
    arduino.close()
    time.sleep(1)
    raiz.destroy()
    raiz.quit()
    print("Proceso finalizado")


def plotData(self, muestraD, lines):
    data.append(datos)
    lines.set_data(range(muestraD), data)
    labelx.set("Distancia: {:.2f} cm".format(datos))

thread = Thread(target=DatosA)

# Configuración de la ventana y lienzo matplotlib
fig = plt.figure(facecolor="0.55", figsize=(6, 4), clear=True, dpi=100)
ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
plt.title("Gráfica de Distancia", color='red', size=16, family="Tahoma")
ax.set_xlabel("Muestras")
ax.set_ylabel("Distancia (cm)")
lines = ax.plot([], [], 'r')[0]

def Limpiar():
    fig.clf()

raiz = Tk()
raiz.protocol("WM_DELETE_WINDOW", Salir)
raiz.config(bg="black")
raiz.title("Grafica de Distancia")
raiz.geometry("738x402")
raiz.resizable(1, 1)

lienzo = FigureCanvasTkAgg(fig, master=raiz)
lienzo._tkcanvas.grid(row=0, column=0, padx=1, pady=1)
frame = Frame(raiz, width=130, height=402, bg="#7003FC")
frame.grid(row=0, column=1, padx=1, pady=2)
frame.grid_propagate(False)
frame.config(relief="sunken")
frame.config(cursor="heart")
labelx = StringVar(raiz, "Distancia: 0.00 cm")

label = Label(frame, textvariable=labelx, bg="#5CFE05", fg="black", font="Helvetica 13 bold", width=11, justify="center")
label.grid(row=0, column=0, padx=5, ipady=8, pady=10)
Iniciar_btn = Button(frame, command=Iniciar, text="Iniciar", bg="blue", fg="white", font="Helvetica 14 bold", width=9, justify="center")
Iniciar_btn.grid(row=1, column=0, padx=5, pady=5)
terminar = Button(frame, command=Salir, text="Terminar", bg="blue", fg="white", font="Helvetica 14 bold", width=9)
terminar.grid(row=2, column=0, padx=5, pady=5)
limpiar = Button(frame, command=Limpiar, text="Limpiar", bg="blue", fg="white", font="Helvetica 14 bold", width=9, justify="center")
limpiar.grid(row=3, column=0, padx=5, pady=5)
salir = Button(frame, command=Salir, width=9, text="SALIR", bg="red", font="Helvetica 14 bold", justify="center")
salir.grid(row=4, column=0, padx=5, pady=125)

anim = animation.FuncAnimation(fig, plotData, fargs=(muestraD, lines), interval=100, blit=False, save_count=muestraD)
plt.show()
raiz.mainloop()



