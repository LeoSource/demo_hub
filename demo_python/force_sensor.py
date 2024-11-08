# !/bin/python3

import time
import serial
import libscrc
import numpy as np
import keyboard
from robot_mqtt_client import RobotMQTTClient
import json

class force_sensor(object):
    def __init__(self,com,bps=9600) -> None:
        self.ser = serial.Serial(com,bps,timeout=0.1)
        print('波特率：',self.ser.baudrate)
        # print('校验位：',self.ser.parity)
        # print('停止位：',self.ser.stopbits)
        # print('读超时设置：',self.ser.timeout)
        # print('软件流控：',self.ser.xonxoff)
        # print('硬件流控：',self.ser.rtscts)
        # print('硬件流控：',self.ser.dsrdtr)
        # print('字符间隔超时：',self.ser.interCharTimeout)
        self.force_data = []
        self.record_button = False
        keyboard.on_press_key('t',self.start_record)
        keyboard.on_press_key('p',self.stop_record)
        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.add_callback_response_robot(self.on_topic_response_robot)
        self.mqtt_client.client.loop_start()

    def on_topic_robot_info(self,client,userdata,msg):
        pass
        # j = json.loads(msg.payload)
        # self.robot_state = j["robot_state"]
        # self.jp = np.array(j["position_joint"])
        # # self.jp[2:4] *= D2R
        # self.rpy = np.array(j["rpy"])

    def on_topic_response_robot(self,client,userdata,msg):
        pass
        # j = json.loads(msg.payload)
        # if "auxiliary_joint" in j:
        #     q_aux = j["auxiliary_joint"]
        #     q_all = np.hstack((self.jp,q_aux))
        #     self.jpos.append(q_all)

    def start_record_rt(self):
        pub_data = {"record":"start"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)

    def stop_record_rt(self):
        pub_data = {"record":"stop"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)

    def read_sensor_value(self,num_channel:int)->int:
        rec_data = self.read_sensor_data(num_channel)
        sensor_value = []
        for idx in range(num_channel):
            sensor_value.append(int.from_bytes(rec_data[4*idx+3:4*idx+7],'big',signed=True))
        return sensor_value
    
    def read_sensor_data(self,num_channel:int)->bytes:
        address = b'\x01'
        func = b'\x03'
        register_add = b'\x00\x0b'
        register_num = (2*num_channel).to_bytes(2,'big')
        send_data = address+func+register_add+register_num
        send_data += libscrc.modbus(send_data).to_bytes(2,'little')
        self.ser.write(send_data)
        len_bytes = 5+4*num_channel
        return self.ser.read(len_bytes)

    def start_record(self,x):
        self.record_button = True
        self.start_record_rt()
        print('start recording...')

    def stop_record(self,x):
        self.stop_record_rt()
        self.record_button = False
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/force_data_'+t+'.txt'
        np.savetxt(filename,self.force_data,delimiter=' ')
        print('create data file '+filename)
        print('complete recording!')
        self.force_data.clear()

    def run(self):
        while True:
            fdata = self.read_sensor_value(1)
            fdata.insert(0,time.time())
            if self.record_button:
                self.force_data.append(fdata)
            time.sleep(0.001)
            # print(fdata)
            # time.sleep(1)
 
if __name__=='__main__':
    fs_sensor = force_sensor('COM8')
    fs_sensor.run()
