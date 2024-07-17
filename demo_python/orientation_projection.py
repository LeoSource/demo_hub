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

def random_max_rpy(cone_angle):
    h = 1
    # cone_angle = 30*d2r
    max_r = math.tan(cone_angle)*h
    r = random.uniform(0,max_r)
    theta = random.uniform(0,2*pi)
    dir = np.array([r*math.cos(theta),r*math.sin(theta),h])
    dir = dir/np.linalg.norm(dir)
    rot_mat = parse_direction(dir)
    rpy = sm.tr2rpy(rot_mat,"zyx")
    return rpy[0:2]

def random_rpy(cone_angle):
    h = 1
    r = h*math.tan(cone_angle)
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
        self.jps = np.array(j["position_joint"])

    def sample(self):
        if not self.is_robot_ready():
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
        if not self.is_robot_ready():
            return
        rpy_list = []
        num = 100
        for idx in range(num):
            rpy = random_max_rpy(30*d2r)
            self.send_traj([rpy.tolist()],idx+1)
            rpy_list.append(np.hstack((self.rpy[0:2],rpy)))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/rpy_data_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np


    def sample_rot_err(self):
        if not self.is_robot_ready():
            return
        self.send_traj([[0,0]],0)
        deg_list = [5,10,20,30]
        rpy_list = []
        num = 100
        for deg in deg_list:
            for idx in range(num):
                rpy = random_rpy(deg*d2r)
                self.send_traj([rpy.tolist()],idx+1)
                rpy_list.append(np.hstack((self.rpy[0:2],rpy,self.jps[2:4])))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_roterr_data_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np


    def sample_rx_err(self):
        if not self.is_robot_ready():
            return
        self.send_traj([[0,0]],0)
        rpy_list = []
        num = 100
        for idx in range(num):
            alpha = random.uniform(-30*d2r,30*d2r)
            self.send_traj([[alpha,0]],idx+1)
            rpy_list.append(np.hstack((self.rpy[0:2],np.array([alpha,0]),self.jps[2:4])))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_roterr_data_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np

        
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

    def is_robot_ready(self):
        time.sleep(2)
        while not self.mqtt_client.client.is_connected():
            print('connecting to robot with mqtt...')
            time.sleep(1)
        if self.robot_state==10:
            print('robot is ready to start to test')
            return True
        else:
            print('robot is not ready!!')
            return False

def analy_validate_data(vdata,title=None):
    num = vdata.shape[0]
    alpha_error = vdata[:,2]-vdata[:,0]
    beta_error = vdata[:,3]-vdata[:,1]
    theta = np.zeros(num)
    for idx in range(num):
        rot_fdb = sm.rpy2r(np.append(vdata[idx,0:2],0),'zyx')
        rot_cmd = sm.rpy2r(np.append(vdata[idx,2:4],0),'zyx')
        theta[idx],v = sm.tr2angvec(rot_cmd@rot_fdb.T)
    print(f'mean Rx error:{np.abs(alpha_error).mean()*r2d:.2f}degree,'
          f'maximum Rx error:{np.abs(alpha_error).max()*r2d:.2f}degree,'
          f'minimum Rx error:{np.abs(alpha_error).min()*r2d:.2f}degree')
    print(f'mean Ry error:{np.abs(beta_error).mean()*r2d:.2f}degree,'
          f'maximum Ry error:{np.abs(beta_error).max()*r2d:.2f}degree,'
          f'minimum Ry error:{np.abs(beta_error).min()*r2d:.2f}degree')
    print(f'mean theta error:{theta.mean()*r2d:.2f}degree,'
          f'maximum theta error:{theta.max()*r2d:.2f}degree,'
          f'mimium theta error:{theta.min()*r2d:.2f}degree')
    # plt.figure()
    # plt.subplot(311)
    # plt.plot(range(num),alpha_error*r2d,label='alpha_error')
    # plt.plot(range(num),beta_error*r2d,label='beta_error')
    # plt.plot(range(num),theta*r2d,label='theta')
    # plt.grid(True)
    # plt.subplot(312)
    # plt.plot(range(num),alpha_error*r2d)
    # plt.plot(range(num),vdata[:,0]*r2d)
    # plt.subplot(313)
    # plt.plot(range(num),beta_error*r2d)
    # plt.plot(range(num),vdata[:,1]*r2d)
    plt.figure()
    plt.plot(range(num),alpha_error*r2d,label='alpha_error')
    plt.plot(range(num),beta_error*r2d,label='beta_error')
    plt.plot(range(num),theta*r2d,label='theta')
    plt.grid(True)
    plt.legend(loc='upper right')
    if title is not None:
        plt.title(title)

def analy_validate_file(filename):
    td = np.loadtxt(filename)
    analy_validate_data(td)
    plt.show()

def analy_roterr_file(filename):
    td = np.loadtxt(filename)
    analy_validate_data(td[0:100,:],'5degree error')
    analy_validate_data(td[100:200,:],'10degree error')
    analy_validate_data(td[200:300,:],'20degree error')
    analy_validate_data(td[300:400,:],'30degree error')
    plt.show()


if __name__=='__main__':
    # of = OrientationFit()
    # of.sample()
    # of.validata()
    # of.sample_rot_err()
    analy_roterr_file('./data/sample_roterr_data_2024-0717-102958.txt')
    # analy_validate_data("rpy_data_2024-0607-173626.txt")
