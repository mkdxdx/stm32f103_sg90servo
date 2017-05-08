# this script will display sensor image on tkinter canvas

import serial
from tkinter import *
import math

port = 'COM30'
baud = 9600

ser = serial.Serial(port,baud)

def formcmd(x,y):
    return "X{0},Y{1};".format(x,y)

def sendcmd(cmd):
    ser.write(bytearray(cmd,'utf-8'))

master = Tk()

frm = Frame(master, width = 255, height = 255)
frm.pack()

value = StringVar()
lab = Label(master,textvariable=value)
lab.pack()
lab2= Label(master,text="Drag mouse from center to send servo axis velocity")
lab2.pack()



def evproc(ev):
    x = ev.x - 127
    y = ev.y - 127
    value.set(formcmd(x,y))
    sendcmd(formcmd(x,y))


frm.bind("<B1-Motion>",evproc)


mainloop()
ser.close()

