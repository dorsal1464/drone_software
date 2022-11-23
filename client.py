# this code will run on the computer controlling the drone

import os
import socket
import threading
import imagezmq
import cv2 as cv
import pickle
import keyboard
import time
import asyncio
import websockets
from PyQt5 import uic, QtWidgets, QtCore, QtGui
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from udp_utils import udpConsumer


# communicating via wifi
TELEMETRY_PORT = 8001
CONTROL_PORT = 8000
CAM_PORT = 5556
TIMEOUT = 60.0
REFRESH_RATE = 250   # Hz
GPS_REFRESH_RATE = 1

GPS = [31.936086, 34.7801967]


# security is assumed via password protected wifi


class DroneController:
    def __init__(self, uiwindow):
        # telemetry and control will use tcp
        self.tel_sock = udpConsumer(TELEMETRY_PORT)
        self.con_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tel_pair = ('', TELEMETRY_PORT)
        self.cam_sock = None
        self.threads = list()
        self.done = False
        self.ui: UiWindow = uiwindow

    def init(self, host):
        self.con_sock.connect((host, CONTROL_PORT))
        # self.tel_sock.connect((host, TELEMETRY_PORT))
        self.cam_sock = imagezmq.ImageHub('tcp://'+host+':'+str(CAM_PORT), False)
        # after connection was established, set timeout
        self.con_sock.settimeout(TIMEOUT)
        return self

    def start(self):
        # start receiving / sending data
        self.threads.append(threading.Thread(target=self.control_updown))
        self.threads.append(threading.Thread(target=self.updateGPS))
        # self.threads.append(threading.Thread(target=self.update_telemetry))
        tl = TelThread(self)
        tl.changeText.connect(self.ui.set_text)
        self.threads.append(tl)

        th = CamThread(p=self.cam_sock)
        th.changePixmap.connect(self.ui.set_image)
        th.changeFPS.connect(self.ui.set_fps)
        self.threads.append(th)

        for t in self.threads:
            t.start()
        print("threads started...")

    # control packet structure:
    # COMMAND: int of command id
    # VECTOR: control vector - (pitch, roll, yaw, throttle)
    # ARGS: if command is not 0 (move) includes parameters such as gps waypoint, alttiude hold etc.
    def safe_send(self, msg):
        length = int.to_bytes(len(msg), 4, 'little')
        self.con_sock.send(length)
        self.con_sock.sendall(msg)

    def control_updown(self):
        # always expecting input, timeout is set to TIMEOUT
        throttle = 1000
        while not self.done:
            # sample keyboard inputs
            # pitch, roll, yaw, throttle
            pitch, roll, yaw, throttle = 1500, 1500, 1500, throttle
            if keyboard.is_pressed('w'):
                pitch += 250
            if keyboard.is_pressed('s'):
                pitch -= 250
            if keyboard.is_pressed('d'):
                pitch += 250
            if keyboard.is_pressed('a'):
                roll -= 250
            if keyboard.is_pressed('q'):
                yaw -= 250
            if keyboard.is_pressed('e'):
                yaw += 250
            if keyboard.is_pressed('shift'):
                throttle += 5
                throttle = min(2000, throttle)
            if keyboard.is_pressed('ctrl'):
                throttle -= 5
                throttle = max(1000, throttle)
            # assemble args control vector
            args = [pitch, roll, yaw, throttle]
            # by default, we have a move command
            command = 0
            # may be adjusted in the future...
            if self.ui.auto_hover.isChecked():
                command = 0
            elif self.ui.hold_alt.isChecked():
                command = 1
            elif self.ui.race_mode.isChecked():
                command = 2
            msg = {'COMMAND': command, 'VECTOR': args, 'ARGS': []}
            try:
                self.safe_send(pickle.dumps(msg))
                self.con_sock.recv(16)
            except socket.error as e:
                # lost contact with drone, abort (for now)
                print('lost connection - '+str(e))
                self.done = True
            # sleep for some time, the flight controller has a certain refresh rate...
            time.sleep(1 / REFRESH_RATE)
        self.con_sock.close()

    def updateGPS(self):
        global GPS
        asyncio.set_event_loop(asyncio.new_event_loop())

        async def handler(websocket, path):
            # d = await websocket.recv()
            await websocket.send(str(GPS[0]) + ":" + str(GPS[1]))
            GPS[0] += 0.00001
            self.ui.lan_coor.display(GPS[0])
            self.ui.lon_coor.display(GPS[1])

        server = websockets.serve(handler, "localhost", 7070)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()


class UiWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi('main.ui', self)
        self.auto_hover: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, 'AutoHover')
        self.camera_feed: QtWidgets.QLabel = self.findChild(QtWidgets.QLabel, 'CameraFeed')
        self.camera_feed.setFixedSize(960, 540)
        self.race_mode = self.findChild(QtWidgets.QPushButton, 'RaceMode')
        self.hold_alt = self.findChild(QtWidgets.QPushButton, 'HoldAlt')
        self.telemetry_box: QtWidgets.QTextBrowser = self.findChild(QtWidgets.QTextBrowser, "Telemetry")
        self.battery: QtWidgets.QProgressBar = self.findChild(QtWidgets.QProgressBar, 'BatteryLevel')
        self.lan_coor = self.findChild(QtWidgets.QLCDNumber, 'LanCoor')
        self.lon_coor = self.findChild(QtWidgets.QLCDNumber, 'LonCoor')
        self.browser: QWebEngineView = self.findChild(QWebEngineView, "WebView")
        # self.browser.load(QUrl("file:///" + os.getcwd().replace("\\", "/") + "/web/index.html"))
        self.browser.load(QUrl.fromLocalFile(os.getcwd() + "\\web\\index.html"))
        # self.browser.load(QUrl("file:///C:/Users/Dor/OneDrive%20-%20Bar-Ilan%20University/Documents/drone_software-master/test.html"))
        self.fps: QtWidgets.QLabel = self.findChild(QtWidgets.QLabel, 'fps')
        print(self.fps)
        self.show()
        print("fine")

    @QtCore.pyqtSlot(QtGui.QImage)
    def set_image(self, image):
        self.camera_feed.setPixmap(QtGui.QPixmap.fromImage(image))

    @QtCore.pyqtSlot(str)
    def set_text(self, text):
        # self.telemetry_box.clear()
        self.telemetry_box.append(text)

    @QtCore.pyqtSlot(int)
    def set_fps(self, fps):
        self.fps.setText(str(fps) + ' FPS')


class CamThread(QtCore.QThread):
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)
    changeFPS = QtCore.pyqtSignal(int)

    def __init__(self, p: imagezmq.ImageHub = None):
        super().__init__()
        self.p = p

    def run(self):
        fps = 0
        cpu_time = time.process_time()
        while True:
            try:
                ret, frame = self.p.recv_image()
                rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                bytesPerLine = ch * w
                convertToQtFormat = QtGui.QImage(rgb.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
                pixmap = convertToQtFormat.scaled(960, 540, QtCore.Qt.KeepAspectRatio)
                self.changePixmap.emit(pixmap)
                if time.process_time() - cpu_time <= 1:
                    fps += 1
                else:
                    cpu_time = time.process_time()
                    self.changeFPS.emit(fps)
                    fps = 0
                    continue
            except Exception as e:
                print(e)


class TelThread(QtCore.QThread):
    changeText = QtCore.pyqtSignal(str)

    def __init__(self, t):
        self.tel_sock: socket.socket = t.tel_sock
        super().__init__()

    def run(self):
        while self.tel_sock is not None:
            try:
                tel, _ = self.tel_sock.recv(2048)
                if tel == b"dummy":
                    tel = tel.decode('utf-8')
                else:
                    tel = pickle.loads(tel)
                self.changeText.emit(tel)
            except socket.error:
                self.tel_sock.close()
            except Exception as e:
                print(e)
        self.tel_sock.close()
