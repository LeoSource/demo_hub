# -*- coding: utf-8 -*-
# @File   		: respiratory_force_sensor.py
# @Description	: do something
# @Date   		: 2024/04/15 15:00:05
# @Author	    : zxliao, zhixiangleo@163.com

import serial
import time

class respiratory_sensor(object):
    def __init__(self,com,bps) -> None:
        try:
            self.ser = serial.Serial(com,bps,timeout=0.1)
        except Exception as e:
            print("---error---:",e)
            raise Exception(e)
        self.len_bytes = 7
        self.data_initial = False
        
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
                
        return int.from_bytes(data[3:5],'big')

    def run(self):
        while True:
            # data = self.read_sensor_data()
            # print(data[0:2])
            # print(data[2])
            # print(data[3:5])
            # print(data[5:7])
            print(self.read_sensor_value())
            time.sleep(0.01)




if __name__=='__main__':
    rp_sensor = respiratory_sensor(com='COM12',bps=9600)
    rp_sensor.run()

