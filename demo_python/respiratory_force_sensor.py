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
from dynamic_graph import DynamicGraph
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
            print(f'respiratory force sensor {com} is opened')
        except Exception as e:
            print("---error---:",e)
            raise Exception(e)
        self.len_bytes = 7
        self.data_initial = False
        self.sensor_value = 1
        self.pre_sensor_value = 0
        self.ttick = 0.01
        self.pre_ttick = 0.1
        self.sensor_vel = 0
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
        self.sensor_value = int.from_bytes(data[3:5],'big')

        self.sensor_vel = (self.sensor_value-self.pre_sensor_value)/(self.ttick-self.pre_ttick)
        self.pre_sensor_value = self.sensor_value
        self.pre_ttick = self.ttick
        return int.from_bytes(data[3:5],'big')
    
    def read_sensor_vel(self)->float:
        return self.sensor_vel

    def run(self):
        while True:
            self.read_sensor_value()
            # print(self.sensor_vel)
            time.sleep(0.01)

    def plot(self):
        graph = DynamicGraph()
        graph.add_plt_data(15,self.read_sensor_value,'band_force')
        graph.add_plt_data(15,self.read_sensor_vel,'band_vel')
        graph.plot()


    def respiratory_control(self):
        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callabck_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

        thread_read = Thread(target=self.run)
        thread_read.daemon = True
        thread_read.start()

        f_max = 33990.0
        f_min = 33270.0
        f_range = f_max-f_min
        pub_data = {"name":"respiratory","motion":False}
        # range = [30970,31160]
        range_exp = [f_min+0.5*f_range,f_min+0.5*f_range]
        range_inhale = [f_max-0.5*f_range,f_max-0.5*f_range]
        expiration = True
        while self.sensor_value is None:
            time.sleep(0.02)
        self.sensor_value_last = self.sensor_value
        while True:
            if expiration:
                if self.sensor_value<range_exp[0] and self.pre_sensor_value>range_exp[0]:
                    pub_data['motion'] = True
                    self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
                if self.sensor_value>range_exp[1] and self.pre_sensor_value<range_exp[1]:
                    pub_data['motion'] = False
                    self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            else:
                if self.sensor_value>range_inhale[0] and self.pre_sensor_value<range_inhale[0]:
                    pub_data['motion'] = True
                    self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
                if self.sensor_value<range_inhale[1] and self.pre_sensor_value>range_inhale[1]:
                    pub_data['motion'] = False
                    self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            self.sensor_value_last = self.sensor_value
            time.sleep(0.005)

    def on_topic_robot_info(self,client,userdata,msg):
        pass

if __name__=='__main__':
    rp_sensor = RespiratorySensor(com='COM3',bps=9600)
    # rp_sensor.run()
    # rp_sensor.plot()
    rp_sensor.respiratory_control()


