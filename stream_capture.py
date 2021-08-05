import serial
import cv2
import numpy as np
import time #####
from procfile import RobotControl

frame_rate = 5
prev = 0

# Установка чтения с порта:
ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200

cap = cv2.VideoCapture(0)
RC = RobotControl()

# Уменьшение разрешения фрейма:
percent = 23.4375
w = int(640 * percent / 100)
h = int(480 * percent / 100)
dim = (w, h)

# Параметры для удаления дисторсии камеры:
K = np.array([[669.53947377, 0., 316.57731188],
              [0., 650.21491053, 291.96812174 ],
              [0., 0., 1.]])
d = np.array([-0.4077693,   0.29706739, -0.00737684, -0.00562703, -0.29700514 ])
newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)
mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)



if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    time_elapsed = time.time() - prev # для уменьшения частоты кадров

    if time_elapsed > 1. / frame_rate:
        prev = time.time()
        ret, frame = cap.read()
        start_time = time.time()
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA) # уменьшаем разрешение фото
        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR) # избавляемся от дисторсии
        RC.frame_processing(frame, ser) # функция обработки фрейма
        cv2.imshow('original', frame)
        if cv2.waitKey(1) == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()