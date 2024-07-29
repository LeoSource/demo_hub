# -*- coding: utf-8 -*-
# @File   		: needle_display.py
# @Description	: display flexible needle real-time with mqtt
# @Date   		: 2024/06/05 09:42:36
# @Author	    : zxliao, zhixiangleo@163.com

from robot_mqtt_client import RobotMQTTClient
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import json
import time
import math


def frame_gen():
    while True:
        yield

def axis_equal_matlab(ax):
    x_range = ax.get_xlim()[1]-ax.get_xlim()[0]
    y_range = ax.get_ylim()[1]-ax.get_ylim()[0]
    z_range = ax.get_zlim()[1]-ax.get_zlim()[0]
    ax.set_box_aspect([x_range,y_range,z_range])

class NeedleGraph(object):
    def __init__(self,mqtt_client) -> None:
        self.pos_needle = None
        self.mqtt_client = mqtt_client
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()
        self.idx = 1
        self.dir = 1.0

    def on_topic_robot_info(self,client,userata,msg):
        j = json.loads(msg.payload)
        p1 = np.array(j["position_vertex"])
        p2 = np.array(j["position_needle"])
        self.pos_needle = np.array([p1,p2])
        self.pos_needle = self.pos_needle.T
        # self.flex_needle = np.array(j["flexible_neede"])
        flex_needle_list = j["flexible_needle"]
        self.flex_needle = np.array(flex_needle_list)
        self.flex_needle = self.flex_needle.T
        self.pos_needle[:,0:1] = self.flex_needle[:,0:1]

    def animate_sim(self):
        self.pos_needle = np.array([[0,0,0],[1,2,3]],dtype=np.float64)
        self.pos_needle = self.pos_needle.T
        self.p1 = np.array([[0,0,0],[1,2,0]],dtype=np.float64)
        self.p1 = self.p1.T

        fig = plt.figure()
        ax = plt.subplot(111,projection='3d')
        self.line2, = ax.plot(self.pos_needle[0,:],self.pos_needle[1,:],self.pos_needle[2,:],'bo-',linewidth=3)
        self.line1, = ax.plot(self.p1[0,:],self.p1[1,:],self.p1[2,:],'go--',linewidth=2)
        ax.set(xlabel='X',ylabel='Y',zlabel='Z')
        ax.view_init(azim=-90,elev=90)
        ax.set_proj_type('ortho')  # 设置为正交投影，以减少透视失真
        ax.set_xlim([0,4])
        fig.tight_layout()

        ani = FuncAnimation(fig, self.update, frames=frame_gen, interval=50, blit=True)

        plt.show()

    def animate(self):
        while not self.mqtt_client.client.is_connected():
            time.sleep(1)

        fig = plt.figure()
        self.ax = plt.subplot(111,projection='3d')
        self.line1, = self.ax.plot(self.pos_needle[0,:],self.pos_needle[1,:],self.pos_needle[2,:],'bo--',linewidth=2)
        self.line2, = self.ax.plot(self.flex_needle[0,:],self.flex_needle[1,:],self.flex_needle[2,:],'go-',linewidth=3)
        self.line3, = self.ax.plot(self.flex_needle[0,[0,3]],self.flex_needle[1,[0,3]],self.flex_needle[2,[0,3]],'ro--',linewidth=1)
        self.ax.set(xlabel='X',ylabel='Y',zlabel='Z')
        self.ax.set_proj_type('ortho')
        # self.ax.autoscale(True)
        self.ax.set_xlim([0.0,0.12])
        self.ax.set_ylim([0.09,0.15])
        self.ax.set_zlim([-0.1,0.06])
        # self.ax.set_box_aspect([1,1,1])
        # self.ax.set_aspect('equal',adjustable='datalim')
        fig.tight_layout()
        ani = FuncAnimation(fig,self.update,frames=frame_gen,interval=50,blit=True)
        plt.show()

    def update(self,frame):
        self.line1.set_data(self.pos_needle[0,:],self.pos_needle[1,:])
        self.line1.set_3d_properties(self.pos_needle[2,:])
        self.line2.set_data(self.flex_needle[0,:],self.flex_needle[1,:])
        self.line2.set_3d_properties(self.flex_needle[2,:])
        self.line3.set_data(self.flex_needle[0,[0,3]],self.flex_needle[1,[0,3]])
        self.line3.set_3d_properties(self.flex_needle[2,[0,3]])
        self.ax.axis('equal')
        self.ax.set(xlim=(0.0,0.12),ylim=(0.09,0.15),zlim=(-0.1,0.06))
        return [self.line1,self.line2,self.line3]

    def update_sim(self,frame):
        if self.idx%20==0:
            self.dir = -1*self.dir
        self.pos_needle[0,1] = self.pos_needle[0,1]+0.1*self.dir
        self.idx = self.idx+1
    
        self.line2.set_data(self.pos_needle[0,:], self.pos_needle[1,:])
        self.line2.set_3d_properties(self.pos_needle[2,:])
        self.line1.set_data(self.p1[0,:],self.p1[1,:])
        self.line1.set_3d_properties(self.p1[2,:])
        self.line3.set_data(self.flex_needle[0,[0,3]],self.flex_needle[1,[0,3]])
        self.line3.set_3d_properties(self.flex_needle[2,[0,3]])
        return [self.line1,self.line2,self.line3]

    def display(self):
        while not self.mqtt_client.client.is_connected():
            print('connecting to robot with mqtt...')
            time.sleep(1)
        time.sleep(2)
        while True:
            p12 = self.flex_needle[:,1]-self.flex_needle[:,0]
            p23 = self.flex_needle[:,2]-self.flex_needle[:,1]
            theta = math.acos(p12.dot(p23)/np.linalg.norm(p12)/np.linalg.norm(p23))
            # print(np.linalg.norm(self.pos_needle[:,1]-self.pos_needle[:,0]))
            # print(np.linalg.norm(p23))
            print(np.linalg.norm(self.flex_needle[:,2]-self.flex_needle[:,3]))
            print(180/math.pi*theta)
            print(self.flex_needle)
            print(self.pos_needle)
            print()
            time.sleep(2)

if __name__=='__main__':
    mqtt_client = RobotMQTTClient('192.168.2.242')
    needle = NeedleGraph(mqtt_client)
    needle.display()
    # needle.animate()
    # needle = NeedleGraph(1)
    # needle.plot()
