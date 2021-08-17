import numpy as np

class RobotControl():
    def __init__(self):
        self.frame = []
        self.IR = 'SI010010010010010010010010E'
        self.IR_last_msg = 'ST0+00000+00000E'
        self.read_data_4_US = ['SUF030E\r\n', 'SUL030E\r\n', 'SUB030E\r\n', 'SUR030E\r\n']
        self.UV_last_error = 0
        self.data_4_US = []
        self.ser = serial.Serial('/dev/ttyUSB0')
        self.ser.baudrate = 115200
        self.cap = cv2.VideoCapture(0)  # замена на VideoStream
        # Уменьшение разрешения фрейма:
        percent = 23.4375
        w = int(640 * percent / 100)
        h = int(480 * percent / 100)
        self.dim = (w, h)
        # Параметры для удаления дисторсии камеры:
        K = np.array([[669.53947377, 0., 316.57731188],
                      [0., 650.21491053, 291.96812174],
                      [0., 0., 1.]])
        d = np.array([-0.4077693, 0.29706739, -0.00737684, -0.00562703, -0.29700514])
        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w, h), 0)
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)

    def data_reading_IR(self):  # функция считывания данных с ИК датчика
        try:
            US_IR = self.ser.readline()
            US_IR = US_IR.decode()
            if US_IR[:2] == 'SI':
                self.IR = US_IR
                print('self IR', self.IR)
        except Exception as e:
            pass
        return self.IR

    def IR_wheel_movement(self, moving):
        wheel_movements_dict = {'LEFT': 'ST0+00250-00250E', 'RIGHT': 'ST0-00250+00250E', 'FORWARD': 'ST0+00300+00300E'}
        msg = wheel_movements_dict.get(moving)
        self.IR_last_msg = msg
        print('msg --- ', msg)
        msg = msg.encode('utf-8')
        self.ser.write(msg)

    def make_last_wheel_movement(self):
        msg = self.IR_last_msg
        msg = msg.encode('utf-8')
        self.ser.write(msg)

    def US_reading(self):
        try:
            US_IR = self.ser.readline()
            US_IR = US_IR.decode()
            if US_IR[:2] == 'SU':
                self.data_4_US.append(US_IR)
                if len(self.data_4_US) == 4:
                    self.read_data_4_US = self.data_4_US
                    self.data_4_US = []
        except Exception as e:
            pass
        return self.read_data_4_US

    def US_wheel_movement(self, moving):
        wheel_movements_dict = {'LEFT': 'ST0+00250+00050E', 'RIGHT': 'ST0-00000+00250E', 'FORWARD': 'ST0+00300+00300E'}
        msg = wheel_movements_dict.get(moving)
        print('msg --- ', msg)
        msg = msg.encode('utf-8')
        self.ser.write(msg)

    def turn_on_the_UV(self):  # функция включения УФ фонарика
        msg_UV = 'SU+000000000000E'
        msg_UV = msg_UV.encode('utf-8')
        self.ser.write(msg_UV)

    def camera_reading(self):
        ret, frame = self.cap.read()
        print(self.dim)
        frame = cv2.resize(frame, self.dim, interpolation=cv2.INTER_AREA)  # уменьшаем разрешение фото
        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)  # избавляемся от дисторсии
        return frame

    def line_area_selection(self, frame, max_cntr, mask):
        x, y, w, h = cv2.boundingRect(max_cntr)
        black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)  # ---black in RGB
        max_cntr_area = cv2.rectangle(black, (x, y), (x + w + 10, y + h + 10), (0, 255, 0), -1)
        aim_area = cv2.bitwise_and(frame, max_cntr_area, mask=mask)
        return aim_area

    def splitting_frame_into_parts(self, frame, j, imgheight, width, mask):
        black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
        grid = cv2.rectangle(black, (0, j), (0 + imgheight, j + width), (0, 255, 0), -1)
        aim_area_part = cv2.bitwise_and(frame, grid, mask=mask)
        return aim_area_part

    def UV_wheel_movement(self, moving, x_point):
        wheel_movements_dict = {'SHARPLY_LEFT': 'ST0+00250-00100E', 'SHARPLY_RIGHT': 'ST0-00100+00250E',
                                'FORWARD': 'ST0+00300+00300E',
                                'SOFTLY_RIGHT': 'ST0+00200+00300E', 'RIGHT': 'ST0+00100+00300E',
                                'SOFTLY_LEFT': 'ST0+00300+00200E',
                                'LEFT': 'ST0+00300+00100E'}
        msg = wheel_movements_dict.get(moving)
        self.UV_last_error = x_point
        msg = msg.encode('utf-8')
        self.ser.write(msg)
