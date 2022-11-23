# this code will run on the drone

import time
import struct
import socket
import threading
import imagezmq
import cv2 as cv
import pickle
from arduino import ArduinoSerialComm
from udp_utils import udpBroadcaster

# communicating via wifi
HOST = '0.0.0.0'
TELEMETRY_PORT = 8001
CONTROL_PORT = 8000
CAM_PORT = 5556
CAMERA_ID = 0
TIMEOUT = 60.0
FPS = 60.0


# security is assumed via password protected wifi


class DroneServer:
    def __init__(self):
        # telemetry and control will use tcp
        self.tel_sock = udpBroadcaster(TELEMETRY_PORT)
        self.con_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cam_sock = imagezmq.ImageSender()
        self.master = None
        self.telemetry_clients = list()
        self.arduino_controller = ArduinoSerialComm('COM3')
        self.threads = list()
        self.done = False
        self.inst_dict = dict()
        self.inst_dict[0] = self.move
        self.host = ''

    def init(self, host=HOST):
        self.host = host
        self.con_sock.bind((host, CONTROL_PORT))
        self.con_sock.listen(1)
        # self.tel_sock.listen(1)
        # wait for a connection on control port
        self.master = self.con_sock.accept()
        # wait for telemetry connection - at least one
        # self.telemetry_clients.append(self.tel_sock.accept())
        # setup image socket
        self.cam_sock = imagezmq.ImageSender('tcp://'+str(self.master[1][0])+':'+str(CAM_PORT), False)
        # after connection was established, set timeout
        self.con_sock.settimeout(TIMEOUT)

    def start(self):
        self.done = False
        print("starting threads...")
        # start receiving / sending data
        self.threads.append(threading.Thread(target=self.control_updown))
        self.threads.append(threading.Thread(target=self.telemetry_up))
        self.threads.append(threading.Thread(target=self.camera_up))
        # self.threads.append(threading.Thread(target=self.add_telemetry))
        for t in self.threads:
            print(t)
        for t in self.threads:
            t.start()

    def add_telemetry(self):
        while not self.done:
            conn, addr = self.tel_sock.accept()
            self.telemetry_clients.append(tuple([conn, addr]))

    def camera_up(self):
        vc = cv.VideoCapture(CAMERA_ID)
        vc.set(cv.CAP_PROP_FRAME_WIDTH, 1270)
        vc.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        vc.set(cv.CAP_PROP_FRAME_COUNT, 30)
        while not self.done:
            ret, frame = vc.read()
            if ret:
                try:
                    self.cam_sock.send_image('', frame)
                except ConnectionError:
                    print("error!")
                    self.cam_sock.close()
                    return 1
                except Exception as e:
                    print(e)
        self.cam_sock.close()

    def telemetry_up(self):
        while not self.done:
            # send telemetry as soon as arduino has the data
            data = self.arduino_controller.get_data()
            # send to each telemetry client
            try:
                print(self.tel_sock.send(data.encode()))
            except Exception as e:
                print(e)

            for tel in self.telemetry_clients:
                try:
                    tel[0].send(pickle.dumps(data))
                    tel[0].recv(256)
                except ConnectionError as e:
                    # remove dead connection
                    self.telemetry_clients.remove(tel)
                    tel[0].close()
            time.sleep(1.0 / 2)

# control packet structure:
# COMMAND: int of command id
# VECTOR: control vector - (pitch, roll, yaw, throttle)
# ARGS: if command is not 0 (move) includes parameters such as gps waypoint, alttiude hold etc.

    def safe_recv(self):
        length = int.from_bytes(self.master[0].recv(4), 'little')
        msg = bytes()
        while len(msg) < length:
            msg += self.master[0].recv(256)
        return msg

    def control_updown(self):
        # always expecting input, timeout is set to TIMEOUT
        msg = dict()
        while not self.done:
            try:
                msg = self.safe_recv()
                msg = pickle.loads(msg)
                self.master[0].send('recv'.encode())
            except TimeoutError:
                # client not responding, activate failsafe
                self.arduino_controller.fail_safe()
                self.master[0].close()
                print("timeout error")
                self.done = True
                # restart
                self.cleanup()
            except socket.error as e:
                # client not responding, activate failsafe
                self.arduino_controller.fail_safe()
                self.master[0].close()
                print("socket error - " + str(e))
                self.done = True
                # restart...
                saviour = threading.Thread(target=self.cleanup)
                saviour.start()
            except pickle.PickleError:
                # flush the buffer, we got only a part of the message
                print('pickled!')
                self.con_sock.recv(256)
            if 'COMMAND' in msg.keys() and 'VECTOR' in msg.keys() and 'ARGS' in msg.keys():
                if msg['COMMAND'] in self.inst_dict:
                    # deal with specific function, might be a blocking function or not...
                    self.inst_dict[msg['COMMAND']](msg['VECTOR'], msg['ARGS'])
                else:
                    print("unknown command - ignore")
            else:
                print("invalid command - ignore")

    def move(self, vector, args):
        self.arduino_controller.write_data([0, vector])

    def cleanup(self):
        print("restarting...")
        # cancel blocking
        self.con_sock.settimeout(None)
        # wait for a connection on control port
        self.master = self.con_sock.accept()
        # wait for telemetry connection - at least one
        # self.telemetry_clients.append(self.tel_sock.accept())
        # setup image socket
        self.cam_sock = imagezmq.ImageSender('tcp://' + str(self.master[1][0]) + ':' + str(CAM_PORT), False)
        # wait for threads to finish execution
        for t in self.threads:
            t.join()
        # clear threads - all have stopped
        self.threads.clear()
        self.start()
