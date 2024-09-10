# -*- coding: utf-8 -*-
# @File   		: gear_ratio_calibration.py
# @Description	: do something
# @Date   		: 2024/09/06 16:45:58
# @Author	    : zxliao, zhixiangleo@163.com

import numpy as np
from robot_mqtt_client import RobotMQTTClient
import json
import time

class GearRatioCalibration(object):
    def __init__(self):
        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.add_callback_response_robot(self.on_topic_response_robot)
        self.jpos = []
        self.mqtt_client.client.loop_start()
        time.sleep(1)

    def on_topic_robot_info(self,client,userdata,msg):
        j = json.loads(msg.payload)
        self.robot_state = j["robot_state"]
        self.rpy = np.array(j["rpy"])
        self.jp = np.array(j["position_joint"])
        self.jp_aux = None

    def on_topic_response_robot(self,client,userdata,msg):
        j = json.loads(msg.payload)
        q_aux = j["auxiliary_joint"]
        q_all = np.hstack((self.jp,q_aux))
        self.jpos.append(q_all)

    def calibrate_master_joint3(self):
        q0 = self.jp[7]
        jpmax = np.array([8*np.pi/180., 8*np.pi/180., q0+0.06])
        jpmin = np.array([-8*np.pi/180., -8*np.pi/180., q0+0.002])
        q = np.array([0,0,jpmin[2]])
        self._send_master_traj(q,0,False)
        qseg = np.linspace(jpmin[2]+0.005,jpmax[2]-0.005,5)
        for idx,qm3 in enumerate(qseg):
            q[2] = qm3
            self._send_master_traj(q,idx,True)
        q = np.array([0,0,jpmax[2]])
        self._send_master_traj(q,0,False)
        for idx,qm3 in enumerate(reversed(qseg)):
            q[2] = qm3
            self._send_master_traj(q,idx,True)

        jpos_np = np.array(self.jpos)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/calibrate_master_ratio_'+t+'.txt'
        np.savetxt(filename,jpos_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return jpos_np
    
    def calibrate_slave_joint5(self):
        jpmax = np.array([0.045,0.058,0.349,1.1,0.1])
        jpmin = np.array([0.002,0.002,-1.08,-0.349,0.002])
        q = self.jp[0:5].copy()
        q[4] = 0.005
        self._send_slave_traj(q,0,False)
        q[4] = 0.005
        self._send_slave_traj(q,0,False)
        qseg = np.linspace(jpmin[4]+0.01,jpmax[4]-0.01,5)
        for idx,qs5 in enumerate(qseg):
            q[4] = qs5
            self._send_slave_traj(q,idx,True)
        q[4] = jpmax[4]-0.005
        self._send_slave_traj(q,0,False)
        for idx,qs5 in enumerate(reversed(qseg)):
            q[4] = qs5
            self._send_slave_traj(q,idx,True)

        jpos_np = np.array(self.jpos)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/calibrate_slave_ratio_'+t+'.txt'
        np.savetxt(filename,jpos_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return jpos_np


    def _send_master_traj(self,via_pos,idx:int,with_aux:bool):
        pub_data = {"name":"motion","master":{"pos":[via_pos.tolist()],"relative":False}}
        self._send_traj(pub_data,idx,with_aux)

    def _send_slave_traj(self,via_pos,idx:int,with_aux:bool):
        pub_data = {"name":"motion","slave":{"pos":[via_pos.tolist()],"relative":False,"type":"joint",
                                             "rcm-constraint":False,"joint-space":True,
                                             "vel_ratio":3.0,"acc_ratio":3.0}}
        self._send_traj(pub_data,idx,with_aux)

    def _send_traj(self,pub_data,idx:int,with_aux:bool):
        self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
        time.sleep(1)
        while self.robot_state!=10:
            time.sleep(0.01)
        print(f'complete test trajectory {idx}')
        time.sleep(2)
        if with_aux:
            self.mqtt_client.pub_request_robot(json.dumps({"name":"auxiliary_joint"}))
            time.sleep(1)

def analy_calib_data(tdata,midx:int,aidx:int):
    num = int(tdata.shape[0]/2)
    dataf = tdata[:num,:]
    datab = np.flip(tdata[num:,:],axis=0)
    dis_fm,dis_fa = [],[]
    dis_bm,dis_ba = [],[]
    for idx in range(dataf.shape[0]-1):
        dis_fm.append(dataf[idx+1,midx]-dataf[idx,midx])
        dis_bm.append(datab[idx+1,midx]-datab[idx,midx])
        dis_fa.append(dataf[idx+1,aidx]-dataf[idx,aidx])
        dis_ba.append(datab[idx+1,aidx]-datab[idx,aidx])
    gear_ratio_f = np.abs(np.array(dis_fm))/np.abs(np.array(dis_fa))
    gear_ratio_b = np.abs(np.array(dis_bm))/np.abs(np.array(dis_ba))
    r = 0.5*(gear_ratio_f.mean()+gear_ratio_b.mean())
    print(f'gear ratio is {r}')
    return r

if __name__=='__main__':
    # calib = GearRatioCalibration()
    # calib.calibrate_master_joint3()
    # calib.calibrate_slave_joint5()
    tdata = np.loadtxt('./data/calibrate_slave_ratio_2024-0910-135600.txt')
    # analy_calib_data(tdata,midx=7,aidx=15)
    analy_calib_data(tdata,midx=4,aidx=12)
