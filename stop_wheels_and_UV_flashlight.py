import serial

ser = serial.Serial('/dev/ttyUSB0',)
ser.baudrate = 115200

def encode_and_send_msg(msg: str):
    """
    отправка сообщения на serial
    :param msg: str
    :return:
    """
    msg = msg.encode('utf-8')
    ser.write(msg)

def stop_driving():
    """
    остановка движения колес
    :return:
    """
    msg_wheels = 'ST0+00000+00000E'
    encode_and_send_msg(msg_wheels)

def turn_off_UV():
    """
    выключение фонарика
    :return:
    """
    msg_UV = 'SU-000000000000E'
    encode_and_send_msg(msg_UV)

stop_driving()
turn_off_UV()
