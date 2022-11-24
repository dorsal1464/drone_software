import client
import server
import socket
import arduino
import time
from PyQt5 import uic, QtWidgets
import sys

if __name__ == '__main__':
    c = input("type: ")
    if str(c) == 'c':
        app = QtWidgets.QApplication(sys.argv)  # Create an instance of QtWidgets.QApplication
        ui = client.EntryWindow()
        # i = client.DroneController(ui)
        # i.init('127.0.0.1')
        # i.start()
        app.exec()  # Start the application
    if str(c) == 's':
        i = server.DroneServer()
        i.init('127.0.0.1')
        i.start()
    if str(c) == 'a':
        a = arduino.ArduinoSerialComm('COM3')
        while True:
            a.write_data([0,[1,2,3,4]])
            print(a.get_data())
            time.sleep(1/250)
            a.write_data([0,[1500,2000,1300,1400]])
            print(a.get_data())
            time.sleep(1/250)
