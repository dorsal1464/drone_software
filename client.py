# this code will run on the computer controlling the drone

import socket
import threading
import imagezmq
import cv2 as cv
import pickle
import keyboard
import time
from PyQt5 import uic, QtWidgets, QtCore, QtGui

# communicating via wifi
TELEMETRY_PORT = 8001
CONTROL_PORT = 8000
CAM_PORT = 5556
TIMEOUT = 60.0
REFRESH_RATE = 250   # Hz


# security is assumed via password protected wifi


class DroneController:
    def __init__(self, uiwindow):
        # telemetry and control will use tcp
        self.tel_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.con_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cam_sock = None
        self.threads = list()
        self.done = False
        self.ui: UiWindow = uiwindow

    def init(self, host):
        self.con_sock.connect((host, CONTROL_PORT))
        self.tel_sock.connect((host, TELEMETRY_PORT))
        self.cam_sock = imagezmq.ImageHub('tcp://'+host+':'+str(CAM_PORT), False)
        # after connection was established, set timeout
        self.con_sock.settimeout(TIMEOUT)
        return self

    def start(self):
        # start receiving / sending data
        self.threads.append(threading.Thread(target=self.control_updown))
        tl = TelThread(self.tel_sock)
        tl.changeText.connect(self.ui.set_text)
        self.threads.append(tl)
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


class UiWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi('main.ui', self)
        self.auto_hover: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, 'AutoHover')
        self.camera_feed: QtWidgets.QLabel = self.findChild(QtWidgets.QLabel, 'CameraFeed')
        self.camera_feed.setFixedSize(960, 540)
        self.race_mode = self.findChild(QtWidgets.QPushButton, 'RaceMode')
        self.hold_alt = self.findChild(QtWidgets.QPushButton, 'HoldAlt')
        self.telemetry_box: QtWidgets.QTextBrowser = self.findChild(QtWidgets.QTextBrowser)
        self.battery: QtWidgets.QProgressBar = self.findChild(QtWidgets.QProgressBar, 'BatteryLevel')
        self.lan_coor = self.findChild(QtWidgets.QLCDNumber, 'LanCoor')
        self.lon_coor = self.findChild(QtWidgets.QLCDNumber, 'LonCoor')
        self.fps: QtWidgets.QLabel = self.findChild(QtWidgets.QLabel, 'FPS')
        self.show()
        print("fine")

    @QtCore.pyqtSlot(QtGui.QImage)
    def set_image(self, image):
        self.camera_feed.setPixmap(QtGui.QPixmap.fromImage(image))

    @QtCore.pyqtSlot(str)
    def set_text(self, text):
        self.telemetry_box.clear()
        self.telemetry_box.setPlainText(text)

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
        cpu_time = time.clock()
        while True:
            try:
                ret, frame = self.p.recv_image()
                rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                bytesPerLine = ch * w
                convertToQtFormat = QtGui.QImage(rgb.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
                pixmap = convertToQtFormat.scaled(960, 540, QtCore.Qt.KeepAspectRatio)
                self.changePixmap.emit(pixmap)
                if time.clock() - cpu_time <= 1:
                    fps += 1
                else:
                    cpu_time = time.clock()
                    self.changeFPS.emit(fps)
                    fps = 0
            except Exception as e:
                print(e)


class TelThread(QtCore.QThread):
    changeText = QtCore.pyqtSignal(str)

    def __init__(self, t):
        self.tel_sock = t
        super().__init__()

    def run(self):
        while self.tel_sock is not None:
            try:
                tel = self.tel_sock.recv(2048)
                self.tel_sock.send(b'recv')
                tel = pickle.loads(tel)
                self.changeText.emit(tel)
            except socket.error:
                self.tel_sock.close()
        self.tel_sock.close()
