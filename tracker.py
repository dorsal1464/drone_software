import cv2 as cv
import numpy as np
from main import Tracker


def get_xy(event,x,y,flags,param):
    global i1, locked, track_window
    if event == cv.EVENT_LBUTTONDBLCLK:
        print(x, y)
        w, h = 100, 50  # simply hardcoded the values
        track_window = (x, y, w, h)
        locked = True


if __name__ == '__main__':
    print("test:")
    i1 = cv.imread("samples/1.jpg")
    i2 = cv.imread("samples/2.jpg")
    i3 = cv.imread("samples/3.jpg")
    i4 = cv.imread("samples/4.jpg")

    locked = False
    backSub = cv.createBackgroundSubtractorKNN()
    capture = cv.VideoCapture(0)
    track_window = (0,0,0,0)
    cv.namedWindow('i1')
    cv.setMouseCallback('i1', get_xy)
    while 1:
        ret, frame = capture.read()
        fgMask = backSub.apply(frame)
        cv.imshow('FG Mask', fgMask)
        cv.imshow('i1', frame)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
