import pygame
import sys
from Robot_test import RobotControl

RC = RobotControl()

# Управление осуществляется на стрелках
# До if else это все для создания картинки. Она нужно для работы с кнопками в этой библиотеке, как видеопоток нужен waitKey()
FPS = 60
W = 150  # ширина экрана
H = 150  # высота экрана
WHITE = (255, 255, 255)
BLUE = (0, 70, 225)

sc = pygame.display.set_mode((W, H))
clock = pygame.time.Clock()

# координаты и радиус круга
x = W // 2
y = H // 2
r = 50

movement = 'STOP'

while True:
    for i in pygame.event.get():
        if i.type == pygame.QUIT:
            sys.exit()
    sc.fill(WHITE)
    pygame.draw.circle(sc, BLUE, (x, y), r)
    pygame.display.update()

    keys = pygame.key.get_pressed()

    # Чтобы смоделировать поступление координат в этот код, сделала так, что определенные значения
    # координат подаются нажатием клавиш.
    if keys[pygame.K_LEFT]: # этой клавишей подаются координаты, соответствующие повороту
                                       # влево вокруг своей оси
        X = 100
        Y = 800
    elif keys[pygame.K_RIGHT]: # этой клавишей подаются координаты, соответствующие повороту
                                       # вправо вокруг своей оси
        X = 800
        Y = 400
    elif keys[pygame.K_UP]:  # этой клавишей подаются значения соответствующие движению вперед
        X = 543
        Y = 20
    elif keys[pygame.K_DOWN]: # этой клавишей подаются координаты, соответствующие движению назад
        X = 475
        Y = 865
    elif keys[pygame.K_a]: # этой клавишей подаются координаты, соответствующие движению назад
        X = 786 # должно быть вправо
        Y = 786
    elif keys[pygame.K_b]:  # этой клавишей подаются координаты, соответствующие движению назад
        X = 1023  # должно быть влево
        Y = 1023
    elif keys[pygame.K_c]:  # этой клавишей подаются координаты, соответствующие движению назад
        X = 255  # должно быть влево
        Y = 255
    else: # если ни одна кнопка не зажата, подаются значения координат, соответствующие остановке робота
        X = 511
        Y = 511

    # в соответствии с значением координат подается сигнал на колеса
    if 255 < X < 767 and 255 < Y < 767:
        RC.set_wheels_movement('STOP')
    elif X >= 767:
        RC.set_wheels_movement('RIGHT')
    elif X <= 255:
        RC.set_wheels_movement('LEFT')
    elif Y <= 255:
        RC.set_wheels_movement('FORWARD')
    elif Y >= 767:
        RC.set_wheels_movement('BACK')

