from library_of_functions import RobotControl
RC = RobotControl()

while True:
    IR_values = RC.data_reading_IR() # ученик  запрашивает функцию для чтения данных с ИК
    threshold = 170 # пороговое начение, сигнализирующее о том, что линия под ИК датчиком
    s1, s2, s3, s4, s5, s6, s7, s8 = int(IR_values[2:5]), int(IR_values[5:8]), int(IR_values[8:11]), int(IR_values[11:14]), int(
        IR_values[14:17]), int(IR_values[17:20]), int(IR_values[20:23]), int(IR_values[23:26])
    print(s1, s2, s3, s4, s5, s6, s7, s8)
    if (s8 > threshold) and (s5 > threshold or s4 > threshold):
        RC.IR_wheel_movement('LEFT')
    elif (s1 > threshold) and (s5 > threshold or s4 > threshold):
        RC.IR_wheel_movement('RIGHT')
    elif s4 > threshold or s5 > threshold:
        RC.IR_wheel_movement('FORWARD')
    elif s1 > threshold or s2 > threshold or s3 > threshold:
        RC.IR_wheel_movement('RIGHT')
    elif s6 > threshold or s7 > threshold or s8 > threshold:
        RC.IR_wheel_movement('LEFT')
    else:
        RC.make_IR_last_wheel_movement()
