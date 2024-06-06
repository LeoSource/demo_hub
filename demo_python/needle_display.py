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


def frame_gen():
    while True:
        yield

class NeedleGraph(object):
    def __init__(self,mqtt_client) -> None:
        self.pos_needle = None
        self.mqtt_client = mqtt_client
        # self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        # self.mqtt_client.client.loop_start()
        self.idx = 1
        self.dir = 1.0

    def on_topic_robot_info(self,client,userata,msg):
        j = json.loads(msg.payload)
        p1 = np.array(j["position_vertex"])
        p2 = np.array(j["position_needle"])
        self.pos_needle = np.array([p1,p2])
        self.pos_needle = self.pos_needle.T

    def animate(self):
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

    def update(self,frame):
        if self.idx%20==0:
            self.dir = -1*self.dir
        self.pos_needle[0,1] = self.pos_needle[0,1]+0.1*self.dir
        self.idx = self.idx+1
    
        self.line2.set_data(self.pos_needle[0,:], self.pos_needle[1,:])
        self.line2.set_3d_properties(self.pos_needle[2,:])
        self.line1.set_data(self.p1[0,:],self.p1[1,:])
        self.line1.set_3d_properties(self.p1[2,:])
        return [self.line1,self.line2]



if __name__=='__main__':
    # mqtt_client = RobotMQTTClient('192.168.2.242')
    # needle = NeedleGraph(mqtt_client)
    needle = NeedleGraph(1)
    # needle.plot()
    needle.animate()
