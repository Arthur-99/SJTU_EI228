import cv2
import numpy as np
import serial
import serial.tools.list_ports

# GUI
import color
import destination

# functions
from detect import *
from comBluetooth import *
from rotate import *
from thin import *
from park import *



cap = cv2.VideoCapture(1)
port = serial.Serial("COM4", 9600, timeout = 1)
comBlutooth('P', port)


width, height = 480, 320

oldOp = 'A'

while cap.isOpened():
    _, img = cap.read()

    cv2.imshow("origin",img)

    th_b = color.Thresh_blue(img)
    th_g = color.Thresh_green(img)
    th_r = color.Thresh_red(img)
    # cv2.imshow("th_b",th_b)
    # cv2.imshow("th_g", th_g)
    # cv2.imshow("th_r", th_r)

    if cv2.waitKey(1)==ord('q'):
        comBlutooth('P', port)
        comBlutooth('S', port)
        break


    # 透视变换
    corners = detect_corners(img)
    if corners is None:
        cv2.imshow("origin", img)
        # cv2.imshow("th_b", th_b)
        # cv2.imshow("th_g", th_g)
        # cv2.imshow("th_r", th_r)
        if cv2.waitKey(1) == ord('q'):
            comBlutooth('P', port)
            comBlutooth('S', port)
            break
        continue

    corners = sorted_corners(corners)

    pts1 = np.float32(corners)
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    img = cv2.warpPerspective(img, M, (width, height))

    
    # detect小车车
    car = detect_car(img)
    print("car is detected: ",car!=None)
    if car is None:
        cv2.imshow("origin", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # wait(1) 以实现在查找失败时，瞬间进入下一循环继续查找
            comBlutooth('P', port)
            comBlutooth('S', port)
            break
        continue

    _, head, _, tail = car


    # park小车车
    if park(head, destination.des, 70) == 'P':
        comBlutooth('P', port)
        comBlutooth('S', port)
        break

    
    # rotate小车车
    th = thin(img)
    cv2.imshow('thin', th)


    op, l, m, r = rotate(th, 40,22, tail, head, oldOp)
    oldOp = op
    comBlutooth(op, port)


    cv2.circle(img, tuple(head), 3, (0, 255, 0), -1)
    cv2.circle(img, tuple(tail), 3, (255, 225, 0), -1)
    cv2.circle(img, tuple(l), 5, (127, 0, 127), -1)
    cv2.circle(img, tuple(m), 5, (0, 127, 127), -1)
    cv2.circle(img, tuple(r), 5, (127, 127, 0), -1)
    cv2.imshow("l & m & r & head & tail", img)


    if cv2.waitKey(1) == ord('q'):
        comBlutooth('P', port)
        comBlutooth('S', port)
        break



port.close()
cap.release()
cv2.destroyAllWindows()
