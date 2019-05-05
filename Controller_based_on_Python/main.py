# -*- coding: utf-8 -*-
import serial
import re
import sys
import easygui
import config
from threading import Thread
from PyQt5 import QtWidgets
from demo import Ui_MainWindow
import lane_detection
import time

cruise = False
collision = False
lane = False


class demo(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(demo, self).__init__(parent)
        self.setupUi(self)
        condition_display = Thread(target=self.condition)
        condition_display.start()
        self.cruise.stateChanged.connect(self.checkbox_state)
        self.collision.stateChanged.connect(self.checkbox_state)
        self.lane.stateChanged.connect(self.checkbox_state)
        self.push_button1.clicked.connect(self.connect)
        self.bln.clicked.connect(self.change)

    def change(self):
        config.set_name('0')

    def connect(self):
        config.set_name('1')
        ll = Thread(target=lane_detection.run)
        ll.start()

    def checkbox_state(self):
        global cruise, collision, lane
        if self.cruise.isChecked():
            cruise = True
        else:
            cruise = False
        if self.collision.isChecked():
            collision = True
        else:
            collision = False
        if self.lane.isChecked():
            lane = True
        else:
            lane = False

    def condition(self):
        global cruise
        global collision
        global lane
        ser = serial.Serial()
        ser.baudrate = 9600  # set baudrate
        ser.port = 'COM6'
        velocity = 0
        voltage = 0
        servo = 0
        way = 1
        check = check_port()
        if check:
            ser.open()
        while check:
            data = ser.read()
            if data == b'A':
                data = ser.read_until(b'}')  # read motor condition
                data = data.decode('utf-8')
                condition = re.findall('-?\d+', data)  # split the data
                left_speed = condition[0]
                right_speed = condition[1]
                voltage = condition[2]
                servo = int(condition[3])-5
                velocity = int(left_speed) + int(right_speed)
            if cruise:
                speed = self.speed_input.text()
                cruise_control = 1
            else:
                cruise_control = 0
                speed = 0
            if collision:
                collision_prevention = 1
            else:
                collision_prevention = 0
            if lane:
                lane_keeping = 1
                way = config.get_angle()
                traffic = config.get_traffic()
                if traffic == 'red':
                    speed = 0
                    cruise_control = 1
                    print('stop')
            else:
                lane_keeping = 0
            output = "{" + str(cruise_control) + "," + str(speed) + "," + str(collision_prevention) + "," \
                     + str(lane_keeping) + ","+str(way)+"}"
            ser.write(output.encode())
            self.speed.display(velocity)
            self.battery.display(voltage)
            self.servo.display(servo)
            time.sleep(0.05)


def check_port():
    try:
        ser = serial.Serial()
        ser.baudrate = 9600  # set baudrate
        ser.port = 'COM6'
        ser.open()
        ser.close()
        return True
    except:
        easygui.msgbox("Failed to open the port!", title="Warning", ok_button="OK")
        return False


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    my_demo = demo()
    my_demo.show()
    sys.exit(app.exec_())
