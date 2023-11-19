import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports

class SerialApp:
    def __init__(self, root):
        self.root = root
        self.port = None

        self.ports = ttk.Combobox(root)
        self.ports.pack()

        self.baudrates = ttk.Combobox(root, values=[9600, 14400, 19200, 38400, 57600, 115200])
        self.baudrates.pack()

        self.connect_button = tk.Button(root, text="Connect", command=self.connect)
        self.connect_button.pack()

        self.entry = tk.Entry(root)
        self.entry.pack()

        self.send_button = tk.Button(root, text="Send", command=self.send)
        self.send_button.pack()

        self.output = tk.Text(root)
        self.output.pack()

        self.update_ports()

    def update_ports(self):
        self.ports['values'] = [port.device for port in serial.tools.list_ports.comports()]

    def connect(self):
        if self.port:
            self.port.close()
        self.port = serial.Serial(self.ports.get(), self.baudrates.get())

    def send(self):
        if self.port:
            self.port.write(self.entry.get().encode())

    def receive(self):
        if self.port:
            while self.port.in_waiting:
                self.output.insert(tk.END, self.port.read(self.port.in_waiting).decode())

root = tk.Tk()
app = SerialApp(root)
root.mainloop()