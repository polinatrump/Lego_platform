import os
import cv2
import numpy as np

class RobotControl():
    def __init__(self):
        self.last_error = 0
        self.lower_red1 = np.array([0, 50, 50])
        self.upper_red1 = np.array([7, 255, 255])
        self.lower_red2 = np.array([170, 50, 50])
        self.upper_red2 = np.array([180, 255, 255])


    def frame_processing(self, frame, ser):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)  # создание маски согласно цвету
        mask1 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask0 + mask1
        x_m = []
        y_m = []
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) != 0:
            max_cntr = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_cntr)
            black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)  # ---black in RGB
            max_cntr_area = cv2.rectangle(black, (x,y),(x+w+10,y+h+10), (0, 255, 0), -1)
            aim_area = cv2.bitwise_and(frame, max_cntr_area, mask=mask)
            imgwidth, imgheight = frame.shape[:2]
            width = 30
            try:
                for j in range(0, imgwidth, width):
                    black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
                    grid = cv2.rectangle(black, (0, j), (0 + imgheight, j + width), (0, 255, 0), -1)
                    fin = cv2.bitwise_and(frame, grid, mask=mask)
                    fin = cv2.bitwise_and(fin, aim_area)

                    fin = cv2.cvtColor(fin, cv2.COLOR_BGR2GRAY)
                    cnts_of_segment, _ = cv2.findContours(fin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    max_cntr = max(cnts_of_segment, key=cv2.contourArea)
                    M = cv2.moments(max_cntr)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                    x_m.append(cX)
                    y_m.append(cY)
            except Exception as e:
                print("error")

                if self.last_error <= 75:
                    print('поворот влево (от ошибки)')
                    print('last error --- ', self.last_error)
                    msg = 'ST0+00250-00100E'
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if self.last_error > 75:
                    print('поворот вправо (от ошибки)')
                    print('last error --- ', self.last_error)
                    msg = 'ST0-00100+00250E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)


            if x_m:
                x_point = x_m[0]
                print('x_point --- ', x_point)
                y_point = y_m[0]
                cv2.circle(frame, (x_point, y_point), 10, (0, 255, 0), 1)

                if 55 < x_point <= 95:
                    print('Вперед')
                    self.last_error = x_point
                    msg = 'ST0+00300+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if 95 <= x_point < 120:
                    print('Вправо мягко')
                    self.last_error = x_point
                    msg = 'ST0+00200+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if 120 <= x_point:
                    print('Вправо резко')
                    self.last_error = x_point
                    msg = 'ST0+00100+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if 30 < x_point <= 55:
                    print('Влево мягко')
                    self.last_error = x_point
                    msg = 'ST0+00300+00200E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if x_point <= 30:
                    print('Влево резко')
                    self.last_error = x_point
                    msg = 'ST0+00300+00100E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

        else:
            if self.last_error <= 75:
                print('поворот влево')
                print('last error --- ', self.last_error)
                msg = 'ST0+00250-00100E'
                print('msg --- ', msg)
                msg = msg.encode('utf-8')
                ser.write(msg)

            if self.last_error > 75:
                print('поворот вправо')
                print('last error --- ', self.last_error)
                msg = 'ST0-00100+00250E'
                print('msg --- ', msg)
                msg = msg.encode('utf-8')
                ser.write(msg)