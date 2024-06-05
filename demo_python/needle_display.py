# -*- coding: utf-8 -*-
# @File   		: needle_display.py
# @Description	: display flexible needle real-time with mqtt
# @Date   		: 2024/06/05 09:42:36
# @Author	    : zxliao, zhixiangleo@163.com

from robot_mqtt_client import RobotMQTTClient
import matplotlib.pyplot as plt
import numpy as np
import json

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

    def plot(self):
        self.pos_needle = np.array([[0,0,0],[1,2,3]],dtype=np.float64)
        self.pos_needle = self.pos_needle.T
        self.p1 = np.array([[0,0,0],[1,2,0]],dtype=np.float64)
        self.p1 = self.p1.T

        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        ax.set(xlabel='X',ylabel='Y',zlabel='Z')
        ax.view_init(azim=-90,elev=90)

        timer = fig.canvas.new_timer(100)
        timer.add_callback(self.__on_timer,ax)
        timer.start()
        plt.show()

    def __on_timer(self,ax):
        if self.idx%20==0:
            self.dir = -1*self.dir
        self.pos_needle[0,1] = self.pos_needle[0,1]+0.5*self.dir
        self.idx = self.idx+1
        print(self.pos_needle[:,1])
        if self.pos_needle is not None:
            ax.plot(self.pos_needle[0,:],self.pos_needle[1,:],self.pos_needle[2,:],'bo-',linewidth=3)
            ax.plot(self.p1[0,:],self.p1[1,:],self.p1[2,:],'go--',linewidth=1)
            plt.draw()


if __name__=='__main__':
    # mqtt_client = RobotMQTTClient('192.168.2.242')
    # needle = NeedleGraph(mqtt_client)
    needle = NeedleGraph(1)
    needle.plot()


    # p1 = np.array([0,0,0])
    # p2 = np.array([1,2,3])
    # pos = np.array([p1,p2])
    # pos = pos.T
    # print(pos[0,:])
    # print(pos[1,:])
    # fig = plt.figure()
    # ax = fig.add_subplot(111,projection='3d')
    # ax.plot(pos[0,:],pos[1,:],pos[2,:],'bo-',linewidth=3)
    # ax.set_xlim(-1,1)
    # ax.set_ylim(-2,2)
    # ax.set_zlim(-3,3)
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plt.show()
