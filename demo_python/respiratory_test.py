# -*- coding: utf-8 -*-
# @File   		: respiratory_test.py
# @Description	: sample and compare respiratory motion data and surrogate date
# @Date   		: 2024/04/12 16:14:44
# @Author	    : zxliao, zhixiangleo@163.com

import laser_rangefinder
import serial_communication
import respiratory_force_sensor
import time
import sys
import keyboard
import csv
from threading import Thread
import threading
import signal

def signal_handler(signal, frame):
    # 终止所有正在运行的线程
    for thread in threading.enumerate():
        if thread.is_alive():
            thread.do_run = False
    print('程序已结束')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class RespiratoryTest(object):
    def __init__(self,laser,servo,rp_sensor) -> None:
        self.laser = laser
        self.servo = servo
        self.rp_sensor = rp_sensor
        self.distance_data = []
        self.respiratory_force = []
        self.needle_force = []
        self.record_button = False
        keyboard.on_press_key('t',self.start_record)
        keyboard.on_press_key('p',self.stop_record)
        keyboard.on_press_key('z',self.quit)

    def quit(self,x):
        sys.exit()

    def start_record(self,x):
        if self.record_button:
            print('cannot record new data while recording!')
        else:
            self.record_button = True
            print('start recording...')

    def stop_record(self,x):
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/respiratorymotion_data_'+t+'.txt'
        with open(filename,'w',newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.distance_data)
            csvfile.close()
        print('create data file '+filename)
        filename = './data/needleforce_data_'+t+'.txt'
        with open(filename,'w',newline='') as csvfile:
            writer  =csv.writer(csvfile)
            writer.writerows(self.needle_force)
            csvfile.close()
        print('create data file '+filename)
        filename = './data/respiratoryforce_data_'+t+'.txt'
        with open(filename,'w',newline='') as csvfile:
            writer  =csv.writer(csvfile)
            writer.writerows(self.respiratory_force)
            csvfile.close()
        print('create data file '+filename)

        print('complete recording!')
        self.distance_data.clear()
        self.respiratory_force.clear()
        self.needle_force.clear()
        self.record_button = False
        

    def read_distance_data(self):
        while True:
            dis = self.laser.read_sensor_value()
            if self.record_button:
                self.distance_data.append([time.time(),dis])

    def read_respiratory_force(self):
        while True:
            rp_force = self.rp_sensor.read_sensor_value()
            if self.record_button:
                self.respiratory_force.append([time.time(),rp_force])

    def read_needle_force(self):
        while True:
            analog_input = self.servo.read_analog_input()
            ai_value = int.from_bytes(analog_input,'big')
            if self.record_button:
                self.needle_force.append([time.time(),ai_value])

    def run(self):
        thread_distance = Thread(target=self.read_distance_data)
        thread_respiratory_force = Thread(target=self.read_respiratory_force)
        thread_needle_force = Thread(target=self.read_needle_force)
        thread_distance.start()
        thread_respiratory_force.start()
        thread_needle_force.start()

        # while True:
        #     time.sleep(1)


if __name__=='__main__':
    laser = laser_rangefinder.laser_rangefinder(9600)
    servo = serial_communication.SerialModbus(port='COM11',bps=9600,timex=0.1)
    rp_sensor = respiratory_force_sensor.respiratory_sensor(com='COM12',bps=9600)
    rp_test = RespiratoryTest(laser,servo,rp_sensor)
    rp_test.run()
