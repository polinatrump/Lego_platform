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

    # при нажатии определенной клавиши, выбирается соответствующий тип движения
    if keys[pygame.K_LEFT]:
        movement = 'LEFT'
    elif keys[pygame.K_RIGHT]:
        movement = 'RIGHT'
    elif keys[pygame.K_UP]:
        movement = 'FORWARD'
    elif keys[pygame.K_DOWN]:
        movement = 'BACK'
    else: # если ничего не нажато - остановка
        movement = 'STOP'

    # в соответствиии с указанным движением подается сигнал на колеса
    if movement == 'LEFT':
        RC.set_wheels_movement('LEFT')
    elif movement == 'RIGHT':
        RC.set_wheels_movement('RIGHT')
    elif movement == 'FORWARD':
        RC.set_wheels_movement('FORWARD')
    elif movement == 'BACK':
        RC.set_wheels_movement('BACK')
    elif movement == 'STOP':
        RC.set_wheels_movement('STOP')

