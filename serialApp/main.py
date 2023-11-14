from tkinter import *
import serial.tools.list_ports
import threading

def connectMenuInit():
    global root, connectButton, refreshButton
    root = Tk()
    root.title("Brazo Robot ED3")
    root.geometry("1280x720")
    root.config(bg="white")

    portLabel = Label(root, text="Puerto(s) serie disponible(s):", bg="white")
    portLabel.grid(column=1, row=2, padx=10, pady=10) 

    portBaudRate = Label(root, text="Baud rate:", bg="white")
    portBaudRate.grid(column=1, row=3, padx=10, pady=10)   

    refreshButton = Button(root, text="Actualizar", height=1, width=10, command=updateSerialPorts)
    refreshButton.grid(column=3, row=2)

    connectButton = Button(root, text="Conectar", height=1, width=10, state="disabled", command=connection)
    connectButton.grid(column=3, row=3)
    baudRateSelect()
    updateSerialPorts()

def connectCheck(args):
    if "-" in selectedBaudRate.get() or "-" in selectedCom.get():
        connectButton["state"] = "disable"
    else:
        connectButton["state"] = "active"

def baudRateSelect():
    global selectedBaudRate, dropBaudRate
    selectedBaudRate = StringVar()
    bauds = ["-",
             "9600",
             "38400",
             "115200"
             ]
    selectedBaudRate.set(bauds[0])
    dropBaudRate = OptionMenu(root, selectedBaudRate, *bauds)
    dropBaudRate.config(width=20)
    dropBaudRate.grid(column=2,row=3,padx=50)

def updateSerialPorts():
    global selectedCom, dropCom
    ports = serial.tools.list_ports.comports()
    coms = [com[0] for com in ports]
    #print(coms)
    coms.insert(0,"-")
    try:
        dropCom.destroy()
    except:
        pass
    selectedCom = StringVar()
    selectedCom.set(coms[0])
    dropCom = OptionMenu(root, selectedCom, *coms, command= connectCheck)
    dropCom.config(width=20)
    dropCom.grid(column=2, row=2, padx=50)
    connectCheck(0)

def readSerial():
    global serialData
    while serialData:
        data = ser.readline()
        if len(data)>0:
            try:
                info = (data.decode('utf8'))
                print(info)
            except:
                pass

def connection():
    global ser, serialData
    if connectButton["text"] in "Desconectar":
        serialData = False
        connectButton["text"] = "Conectar"
        refreshButton["state"] = "active"
        dropBaudRate["state"] = "active"
        dropBaudRate["state"] = "active"
    else:
        serialData = True
        connectButton["text"] = "Desconectar"
        refreshButton["state"] = "disable"
        dropBaudRate["state"] = "disable"
        dropBaudRate["state"] = "disable"
        port = selectedCom.get()
        baud = selectedBaudRate.get()
        #print(port,baud)
        try:
            ser = serial.Serial(port,baud,timeout=0)
            print("Se pudo crear el objeto puerto serie")
        except:
            print("error con serial")
            pass
        t1 = threading.Thread(target=readSerial)
        t1.daemon = True
        t1.start()


connectMenuInit()

root.mainloop()
