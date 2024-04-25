# -*- coding: utf-8 -*-
# @File   		: respiratory_force_sensor.py
# @Description	: do something
# @Date   		: 2024/04/15 15:00:05
# @Author	    : zxliao, zhixiangleo@163.com

import serial
import time
import matplotlib.pyplot as plt
from threading import Thread

POINTS = 300

class respiratory_sensor(object):
    def __init__(self,com,bps) -> None:
        try:
            self.ser = serial.Serial(com,bps,timeout=0.1)
        except Exception as e:
            print("---error---:",e)
            raise Exception(e)
        self.len_bytes = 7
        self.data_initial = False
        self.sensor_value = None
        self.ttick = None
        self.t0 = None

        self.fig,self.ax = plt.subplots()
        self.ax.set_autoscaley_on(True)
        self.ax.grid(True)
        self.plt_data = [None]*POINTS
        self.time = [None]*POINTS
        self.line_data, = self.ax.plot(range(POINTS),self.plt_data,label='force')
        # self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.on_timer)
        
    def read_sensor_data(self)->bytes:
        if self.data_initial:
            return self.ser.read(self.len_bytes)
        else:
            while True:
                data=self.ser.read(1)
                if data==b'\xff':
                    data = self.ser.read(1)
                    if data==b'\xff':
                        data = self.ser.read(self.len_bytes-2)
                        self.data_initial = True
                        break
                self.t0 = time.time()
            return self.ser.read(self.len_bytes)
    
    def read_sensor_value(self)->int:
        value_validation = False
        while not value_validation:
            data = self.read_sensor_data()
            data_type = int.from_bytes(data[2:3],'big')
            if data_type==1:
                value_validation = True
        self.ttick = time.time()-self.t0

        return int.from_bytes(data[3:5],'big')

    def run(self):
        while True:
            self.sensor_value = self.read_sensor_data()
            # print(data[0:2])
            # print(data[2])
            # print(data[3:5])
            # print(data[5:7])
            # print(self.read_sensor_value())
            time.sleep(0.01)

    def on_timer(self):
        self.plt_data = self.plt_data[1:]+[self.sensor_value]
        # self.time = self.time[1:]+[self.ttick]
        # self.line_data.set_ydata(self.plt_data)
        # self.data_fig.set_xdata(self.time)
        # self.ax.draw_artist(self.line_data)
        # self.ax.figure.canvas.draw()

    def plot(self):
        thread_read = Thread(target=self.run)
        thread_read.daemon = True
        thread_read.start()

        self.timer.start()
        plt.show()



if __name__=='__main__':
    rp_sensor = respiratory_sensor(com='COM12',bps=9600)
    # rp_sensor.run()
    rp_sensor.plot()



