import serial
import cv2
import numpy as np
import time

frame_rate = 5
prev = 0
# t_sleep = 0.05

# Установка чтения с порта:
ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200

# Уменьшение разрешения фрейма:
cap = cv2.VideoCapture(0)
percent = 23.4375
w = int(640 * percent / 100)
h = int(480 * percent / 100)
dim = (w, h)

# Параметры для удаления дисторсии:
K = np.array([[669.53947377, 0., 316.57731188],
              [0., 650.21491053, 291.96812174 ],
              [0., 0., 1.]])
d = np.array([-0.4077693,   0.29706739, -0.00737684, -0.00562703, -0.29700514 ])
newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)
mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)


class RobotControl():
    def __init__(self):
        self.last_error = 0
        self.last_error_massiv = []
        self.last_x_min = 0

    def frame_processing(self, frame):
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
                    black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)  # ---black in RGB
                    grid = cv2.rectangle(black, (0, j), (0 + imgheight, j + width), (0, 255, 0),
                                         -1)  # ---the dimension of the ROI
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
                    print('поворот влево (от ошибки) 1 вар')
                    print('last error --- ', self.last_error)
                    print('x min --- ', self.last_x_min)
                    msg = 'ST0+00250-00100E'
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if self.last_error > 75:
                    print('поворот вправо (от ошибки) 1 вар')
                    print('last error --- ', self.last_error)
                    print('x min --- ', self.last_x_min)
                    msg = 'ST0-00100+00250E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)


            if x_m:
                print(x_m)
                print(y_m)
                x_point = x_m[0]
                print('x_point --- ', x_point)
                x_min = x_m[-1]
                print('x_min', x_min)
                self.last_x_min = x_min
                y_point = y_m[0]
                # print('y_point --- ', y_point)
                cv2.circle(frame, (x_point, y_point), 10, (0, 255, 0), 1)
                number_of_points = len(x_m)
                # print('number of points --- ', number_of_points)
                frame_center = frame.shape[1] / 2
                # print('frame center --- ', frame_center)

                if 55 < x_point <= 95:
                    print('Вперед')
                    self.last_error = x_point
                    self.last_error_massiv.append(self.last_error)
                    msg = 'ST0+00300+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)
                    answer = ser.readline()
                    e = int(answer[0])
                    print(e)
                    print(answer)

                if 95 <= x_point < 120:
                    print('Вправо мягко')
                    self.last_error = x_point
                    self.last_error_massiv.append(self.last_error)
                    # выравниваемся вправо
                    msg = 'ST0+00200+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if 120 <= x_point:
                    print('Вправо резко')
                    self.last_error = x_point
                    self.last_error_massiv.append(self.last_error)
                    # выравниваемся вправо
                    msg = 'ST0+00100+00300E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if 30 < x_point <= 55:
                    print('Влево мягко')
                    self.last_error = x_point
                    self.last_error_massiv.append(self.last_error)
                    msg = 'ST0+00300+00200E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

                if x_point <= 30:
                    print('Влево резко')
                    self.last_error = x_point
                    self.last_error_massiv.append(self.last_error)
                    msg = 'ST0+00300+00100E'
                    print('msg --- ', msg)
                    msg = msg.encode('utf-8')
                    ser.write(msg)

        else:
            if self.last_error <= 75:
                print('поворот влево')
                print('last error --- ', self.last_error)
                msg = 'ST0+00250-00100E'
                msg = msg.encode('utf-8')
                ser.write(msg)

            if self.last_error > 75:
                print('поворот вправо')
                print('last error --- ', self.last_error)
                msg = 'ST0-00100+00250E'
                print('msg --- ', msg)
                msg = msg.encode('utf-8')
                ser.write(msg)






RC = RobotControl()

# Установка цвета дорожки
lower_red = np.array([0,50,50])
upper_red = np.array([7,255,255])
lower_red1= np.array([170,50,50])
upper_red1 = np.array([180,255,255])

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    massiv = []
    time_elapsed = time.time() - prev

    if time_elapsed > 1. / frame_rate:
        prev = time.time()
        ret, frame = cap.read()
        start_time = time.time()
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA) # уменьшаем разрешение
        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR) # избавляемся от дисторсии
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(hsv, lower_red, upper_red) # создание маски согласно цвету
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask = mask0 + mask1
        print('FRAME SHAPE ', frame.shape) # проверить размер фрейма (должен быть (112, 150, 3))
        RC.frame_processing(frame) # функция обработки фрейма
        print("--- %s seconds ---" % (time.time() - start_time))
        cv2.imshow('original', frame)
        if cv2.waitKey(1) == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()