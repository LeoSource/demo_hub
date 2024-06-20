# -*- coding: utf-8 -*-
# @File   		: orientation_projection.py
# @Description	: sample/validate orientation between motor and auxiliary model
# @Date   		: 2024/06/06 14:06:49
# @Author	    : zxliao, zhixiangleo@163.com

import numpy as np
from robot_mqtt_client import RobotMQTTClient
import spatialmath.base as sm
import matplotlib.pyplot as plt
import json
import time
import math
import random

pi = math.pi
r2d = 180/math.pi
d2r = math.pi/180

def parse_direction(dir):
    zn = dir/np.linalg.norm(dir)
    alpha = -math.asin(zn[1])
    beta = math.asin(zn[0]/math.cos(alpha))
    return sm.roty(beta)@sm.rotx(alpha)

def random_rpy():
    h = 1
    cone_angle = 30*d2r
    max_r = math.tan(cone_angle)*h
    r = random.uniform(0,max_r)
    theta = random.uniform(0,2*pi)
    dir = np.array([r*math.cos(theta),r*math.sin(theta),h])
    dir = dir/np.linalg.norm(dir)
    rot_mat = parse_direction(dir)
    rpy = sm.tr2rpy(rot_mat,"zyx")
    return rpy[0:2]

class OrientationFit(object):
    def __init__(self) -> None:
        self.robot_state = None
        self.rpy = None

        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

    def on_topic_robot_info(self,client,userdata,msg):
        j = json.loads(msg.payload)
        self.robot_state = j["robot_state"]
        self.rpy = np.array(j["rpy"])

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
        rpy_list = []
        num = 100
        for idx in range(num):
            rpy = random_rpy()
            self.send_traj([rpy.tolist()],idx+1)
            rpy_list.append(np.hstack((self.rpy[0:2],rpy)))
        rpy_np = np.array(rpy_list)
        alpha_error = np.abs(rpy_np[:,0]-rpy_np[:,2])
        beta_error = np.abs(rpy_np[:,1]-rpy_np[:,3])
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = 'rpy_data_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print(f'maximum Rx error:{alpha_error.max()*r2d}degree, mean Rx error:{alpha_error.mean()*r2d}degree')
        print(f'maximum Ry error:{beta_error.max()*r2d}degree, mean Ry error:{beta_error.mean()*r2d}degree')
        print('save data successfully!')

        fig1 = plt.figure()
        plt.plot(range(num),alpha_error,label='alpha_error')
        plt.plot(range(num),beta_error,label='beta_error')
        plt.grid(True)
        plt.legend(loc='upper right')
        plt.show()
        
    def send_traj(self,via_pos,idx:int):
        pub_data = {"name":"motion","slave":{
            "pos":via_pos,"type":"rpy","relative":False,
            "rcm-constraint":False,"joint-space":False,
            "vel_ratio":4.0,"acc_ratio":6.0}}
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

def analy_validate_data(filename):
    td = np.loadtxt(filename)
    num = td.shape[0]
    alpha_error = np.abs(td[:,0]-td[:,2])
    beta_error = np.abs(td[:,1]-td[:,3])
    # alpha_error[11] = 0
    # alpha_error[27] = 0
    # beta_error[27] = 0
    # alpha_error[26] = 0
    # beta_error[26] = 0
    print(f'maximum Rx error:{alpha_error.max()*r2d}degree, mean Rx error:{alpha_error.mean()*r2d}degree')
    print(f'maximum Ry error:{beta_error.max()*r2d}degree, mean Ry error:{beta_error.mean()*r2d}degree')

    fig1 = plt.figure()
    plt.plot(range(num),alpha_error,label='alpha_error')
    plt.plot(range(num),beta_error,label='beta_error')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.show()



if __name__=='__main__':
    of = OrientationFit()
    # of.sample()
    of.validata()
    # analy_validate_data("rpy_data_2024-0607-173626.txt")
