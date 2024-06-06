# -*- coding: utf-8 -*-
# @File   		: orientation_projection.py
# @Description	: sample/validate orientation between motor and auxiliary model
# @Date   		: 2024/06/06 14:06:49
# @Author	    : zxliao, zhixiangleo@163.com


from robot_mqtt_client import RobotMQTTClient
import json
import time
import math

pi = math.pi
r2d = 180/math.pi
d2r = math.pi/180

class OrientationFit(object):
    def __init__(self) -> None:
        self.robot_state = None

        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

    def on_topic_robot_info(self,client,userdata,msg):
        j = json.loads(msg.payload)
        self.robot_state = j["robot_state"]

    def sample(self):
        time.sleep(2)
        while not self.mqtt_client.client.is_connected():
            print('connecting to robot with mqtt...')
            time.sleep(1)

        if self.robot_state==10:
            print('robot is ready to start to test')
        else:
            print('robot is not ready!!')
            return
        
        self.start_record()
        self.send_traj([[0,0],[10*d2r,0],[-10*d2r,0],[0,0]],1)
        self.send_traj([[20*d2r,10*d2r],[-20*d2r,10*d2r],[0,0]],2)
        self.send_traj([[30*d2r,5*d2r],[-30*d2r,5*d2r],[0,0]],3)
        self.send_traj([[0,10*d2r],[0,-10*d2r],[0,0]],4)
        self.send_traj([[10*d2r,20*d2r],[10*d2r,-20*d2r],[0,0]],5)
        self.send_traj([[5*d2r,30*d2r],[5*d2r,-30*d2r],[0,0]],6)
        self.send_traj([[15*d2r,-15*d2r],[-15*d2r,15*d2r],[0,0]],7)
        self.send_traj([[15*d2r,15*d2r],[-15*d2r,-15*d2r],[0,0]],8)
        self.stop_record()
        print('complete sample!!')


    def validata(self):
        time.sleep(2)
        while not self.mqtt_client.client.is_connected():
            print('connecting to robot with mqtt...')
            time.sleep(1)

        if self.robot_state==10:
            print('robot is ready to start to test')
        else:
            print('robot is not ready!!')
            return
        
        

        

    def send_traj(self,via_pos,idx):
        pub_data = {"name":"motion","slave":{
            "pos":via_pos,"type":"rpy","relative":False,
            "rcm-constraint":False,"joint-space":False}}
        self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
        time.sleep(0.5)
        while self.robot_state!=10:
            time.sleep(0.01)
        print(f'complete test trajectory {idx}')
        time.sleep(2)

    def start_record(self):
        pub_data = {"record":"start"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)

    def stop_record(self):
        pub_data = {"record":"stop"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)


if __name__=='__main__':
    of = OrientationFit()
    of.sample()

