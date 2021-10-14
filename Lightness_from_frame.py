import numpy as np
import time
import cv2
import imutils
from Robot_test_library_functions import RobotControl
RC = RobotControl()
prev = 0
frame_rate = 5

# Уменьшение разрешения фрейма:
cap = cv2.VideoCapture(2) # нижняя камера
cap2 = cv2.VideoCapture(1) # верхняя камера

def analysis_lighting(img, thrshld):
    is_light = np.mean(img) > thrshld
    print(np.mean(img)) # вывести уровень света на кадре
    return 'light' if is_light else 'dark'

while True:
    time_elapsed = time.time() - prev
    if time_elapsed > 1. / frame_rate:
        prev = time.time()

        ret, frame = cap.read()
        frame = imutils.rotate(frame, angle=180) # перевернуть изображение
        lighting = analysis_lighting(frame, 80)
        print(lighting)
        cv2.imshow('frame_Lower', frame)

        ret2, frame2 = cap2.read()
        frame2 = imutils.rotate(frame2, angle=270) # перевернуть изображение
        cv2.imshow('frame_Upper', frame2)

        if cv2.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()