import cv2
import numpy as np
from library_of_functions import RobotControl
import time

RC = RobotControl()
prev = 0
frame_rate = 5
lower_UV = np.array([90, 50, 240])
upper_UV = np.array([120, 140, 255])
while True:
    RC.turn_on_the_UV()
    time_elapsed = time.time() - prev  # для уменьшения частоты кадров
    if time_elapsed > 1. / frame_rate:
        prev = time.time()
        frame = RC.camera_reading()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_UV, upper_UV)  # создание маски согласно цвету
        x_m = []
        y_m = []
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(len(cnts))
        if len(cnts) != 0:
            max_cntr = max(cnts, key=cv2.contourArea)
            aim_area = RC.line_area_selection(frame, max_cntr, mask) # выделение области линии на изображении
            imgwidth, imgheight = frame.shape[:2]
            width = 30
            try:
                for j in range(0, imgwidth, width):
                    aim_area_part = RC.splitting_frame_into_parts(frame, j, imgheight, width, mask) # разделение кадра на участки
                    final_aim = cv2.bitwise_and(aim_area_part, aim_area) # выделение на каждом участке области целевой линии
                    final_aim_gray = cv2.cvtColor(final_aim, cv2.COLOR_BGR2GRAY)
                    cnts_of_segment, _ = cv2.findContours(final_aim_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    max_cntr = max(cnts_of_segment, key=cv2.contourArea)
                    M = cv2.moments(max_cntr)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                    x_m.append(cX)
                    y_m.append(cY)
            except Exception as e:
                print('Поворот при потере линии (от ошибки)')
                if RC.UV_last_error <= 75:
                    RC.UV_wheel_movement('SHARPLY_LEFT', RC.UV_last_error)
                    print('SHARPLY_LEFT')
                if RC.UV_last_error > 75:
                    RC.UV_wheel_movement('SHARPLY_RIGHT', RC.UV_last_error)
                    print('SHARPLY_RIGHT')

            if x_m:
                x_point = x_m[0] # ориентируемся по этой точке
                y_point = y_m[0]
                cv2.circle(frame, (x_point, y_point), 10, (0, 255, 0), 1) # обводим целевую точку на кадре
                if 55 < x_point < 95:
                    RC.UV_wheel_movement('FORWARD', x_point)
                    print('FORWARD')
                elif 95 <= x_point < 115:
                    RC.UV_wheel_movement('SOFTLY_RIGHT', x_point)
                    print('SOFTLY_RIGHT')
                elif 115 <= x_point:
                    RC.UV_wheel_movement('RIGHT', x_point)
                    print('RIGHT')
                elif 35 < x_point <= 55:
                    RC.UV_wheel_movement('SOFTLY_LEFT', x_point)
                    print('SOFTLY_LEFT')
                if x_point <= 35:
                    RC.UV_wheel_movement('LEFT', x_point)
                    print('LEFT')
        else:
            print('Поворот при потере линии (так как на фрейме нет контура)')
            if RC.UV_last_error <= 75:
                RC.UV_wheel_movement('SHARPLY_LEFT', RC.UV_last_error)
                print('SHARPLY_LEFT')
            if RC.UV_last_error > 75:
                RC.UV_wheel_movement('SHARPLY_RIGHT', RC.UV_last_error)
                print('SHARPLY_RIGHT')
        cv2.imshow('original', frame)
        if cv2.waitKey(1) == ord('q'):
            break

RC.cap.release()
cv2.destroyAllWindows()
