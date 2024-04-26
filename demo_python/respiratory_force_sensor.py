# -*- coding: utf-8 -*-
# @File   		: respiratory_force_sensor.py
# @Description	: do something
# @Date   		: 2024/04/15 15:00:05
# @Author	    : zxliao, zhixiangleo@163.com

import serial
import time
import matplotlib.pyplot as plt
from threading import Thread
from robot_mqtt_client import RobotMQTTClient
import json

POINTS = 200

def in_range(value,range)->bool:
    if value>=range[0] and value<=range[1]:
        return True
    else:
        return False
class RespiratorySensor(object):
    def __init__(self,com:str,bps=9600) -> None:
        try:
            self.ser = serial.Serial(com,bps,timeout=0.1)
            print(f'{com} is opened')
        except Exception as e:
            print("---error---:",e)
            raise Exception(e)
        self.len_bytes = 7
        self.data_initial = False
        self.sensor_value = None
        self.ttick = None
        self.t0 = time.time()
        self.time_max = 10

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
            self.sensor_value = self.read_sensor_value()
            time.sleep(0.01)

    def on_timer(self,ax):
        self.plt_data = self.plt_data[1:]+[self.sensor_value]
        self.time = self.time[1:]+[self.ttick]
        self.line_data.set_xdata(self.time)
        self.line_data.set_ydata(self.plt_data)
        if None in self.time:
            xdata = [t for t in self.time if t is not None]
            if len(xdata)==1:
                pass
            else:
                ax.set_xlim([min(xdata),max(xdata)])
        else:
            ax.set_xlim([min(self.time),max(self.time)])
        if None in self.plt_data:
            ydata = [y for y in self.plt_data if y is not None]
            if len(ydata)==1:
                pass
            else:
                ax.set_ylim([min(ydata),max(ydata)])
        else:
            ax.set_ylim([min(self.plt_data),max(self.plt_data)])
        ax.draw_artist(self.line_data)
        ax.figure.canvas.draw()

    def plot(self):
        thread_read = Thread(target=self.run)
        thread_read.daemon = True
        thread_read.start()

        self.plt_data = [None]*POINTS
        self.time = [None]*POINTS
        fig,ax = plt.subplots()
        ax.set_xlim([0,self.time_max])
        ax.set_ylim([30970,31160])
        ax.set_autoscale_on(False)
        ax.grid(True)
        self.line_data, = ax.plot(self.time,self.plt_data,label='force')
        # self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        timer = fig.canvas.new_timer(interval=50)
        timer.add_callback(self.on_timer,ax)
        timer.start()
        plt.show()

    def respiratory_control(self):
        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callabck_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

        thread_read = Thread(target=self.run)
        thread_read.daemon = True
        thread_read.start()

        pub_data = {"name":"respiratory","motion":False}
        # range = [30970,31160]
        range = [0,30850]
        # range = [31160,50000]
        while self.sensor_value is None:
            time.sleep(0.02)
        self.sensor_value_last = self.sensor_value
        while True:
            if in_range(self.sensor_value,range) and not in_range(self.sensor_value_last,range):
                pub_data['motion'] = True
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            if in_range(self.sensor_value_last,range) and not in_range(self.sensor_value,range):
                pub_data['motion'] = False
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            self.sensor_value_last = self.sensor_value
            time.sleep(0.005)

    def on_topic_robot_info(self,client,userdata,msg):
        pass

if __name__=='__main__':
    rp_sensor = RespiratorySensor(com='COM12',bps=9600)
    # rp_sensor.run()
    # rp_sensor.plot()
    rp_sensor.respiratory_control()


