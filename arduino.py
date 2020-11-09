import serial
import time


class ArduinoSerialComm:
    def __init__(self, port):
        self.port = port
        self.dummy = False
        try:
            self.ard = serial.Serial(port, 9600)
        except Exception as e:
            print("port " + port + " is closed, dummy mode activated")
            self.dummy = True
        # delay to establish connection
        time.sleep(2)

    def get_data(self):
        if self.dummy:
            return "dummy"
        raw_data = self.ard.readline()
        return raw_data

    def fail_safe(self):
        print('executing failsafe')

    def write_data(self, data):
        if self.dummy:
            return 1
        if type(data) == list:
            # expecting a list as follows: [mode, vector]
            tmp = str(data[0])
            for inp in data[1]:
                tmp += ' ' + str(inp)
            self.ard.write(tmp.encode('utf-8'))

        if type(data) == str:
            self.ard.write(data.encode('utf-8'))
        self.ard.write('\n'.encode('utf-8'))
