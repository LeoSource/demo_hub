# -*- coding: utf-8 -*-
# @File   		: robot_operation_manager.py
# @Description	: manage and control the robot operation
# @Date   		: 2024/09/11 10:37:49
# @Author	    : zxliao, zhixiangleo@163.com

import numpy as np
from robot_mqtt_client import RobotMQTTClient
import spatialmath.base as smb
import spatialmath as sm
import json
import time
import matplotlib.pyplot as plt
import math
import random

PI = np.pi
R2D = 180.0/np.pi
D2R = np.pi/180.0


def parse_direction(direction):
    zn = direction/np.linalg.norm(direction,axis=0)
    alpha = -np.arcsin(zn[1,:])
    beta = np.arcsin(zn[0,:]/np.cos(alpha))
    return (sm.SO3.Ry(beta)*sm.SO3.Rx(alpha)).R

def random_max_rpy(cone_angle,num_points=1):
    h = 1
    max_r = math.tan(cone_angle)*h
    r = np.random.uniform(0,max_r,num_points)
    theta = np.random.uniform(0,2*PI,num_points)
    direction = np.array([r*np.cos(theta),r*np.sin(theta),h*np.ones(r.shape)])
    direction /= np.linalg.norm(direction,axis=0)
    alpha = -np.arcsin(direction[1,:])
    beta = np.arcsin(direction[0,:]/np.cos(alpha))
    rot_mat = sm.SO3.Ry(beta)*sm.SO3.Rx(alpha)
    rpy = rot_mat.rpy(rot_mat,'zyx')
    if rpy.ndim==1:
        return rpy[0:2]
    else:
        return rpy[:,0:2]

def random_rpy(cone_angle,num_points=1):
    h = 1
    r = h*math.tan(cone_angle)
    theta = np.random.uniform(0,2*PI,num_points)
    direction = np.array([r*np.cos(theta),r*np.sin(theta),h*np.ones(theta.shape)])
    direction = direction/np.linalg.norm(direction)
    rot_mat = parse_direction(direction)
    rpy = smb.tr2rpy(rot_mat,"zyx")
    return rpy[0:2]



class RobotOperationManager:
    def __init__(self,ip_address):
        self.mqtt_client = RobotMQTTClient(ip_address)
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.add_callback_response_robot(self.on_topic_response_robot)
        self.mqtt_client.client.loop_start()
        self.jpos = []
        time.sleep(1)

    def on_topic_robot_info(self,client,userdata,msg):
        j = json.loads(msg.payload)
        self.robot_state = j["robot_state"]
        self.jp = np.array(j["position_joint"])
        # self.jp[2:4] *= D2R
        self.rpy = np.array(j["rpy"])

    def on_topic_response_robot(self,client,userdata,msg):
        j = json.loads(msg.payload)
        if "auxiliary_joint" in j:
            q_aux = j["auxiliary_joint"]
            q_all = np.hstack((self.jp,q_aux))
            self.jpos.append(q_all)

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

    def run_random_joint(self):
        jpmin = np.array([0.005,0.005,-0.8,-0.1,0.005])
        jpmax = np.array([0.05,0.05,0.1,0.8,0.08])
        loop = 0
        while True:
            q = jpmin+(jpmax-jpmin)*np.random.rand(5)
            self._send_slave_joint_traj(q,loop,False)
            loop += 1

    def sample_master_gear_ratio(self,jidx:int):
        if not self.is_robot_ready():
            return
        q_ready = np.array([0,0,self.jp[7]])
        q0 = self.jp[jidx+5]
        jpmax = np.array([8*D2R, 8*D2R, q0+0.06])
        jpmin = np.array([-8*D2R, -8*D2R, q0+0.002])
        q = self.jp[5:].copy()
        q[jidx] = jpmin[jidx]
        self._send_master_traj(q,0,False)
        if jidx==0:
            qseg = np.linspace(jpmin[0]+0.5*D2R,jpmax[0]-0.5*D2R,5)
        elif jidx==1:
            qseg = np.linspace(jpmin[1]+0.5*D2R,jpmax[1]-0.5*D2R,5)
        elif jidx==2:
            qseg = np.linspace(jpmin[2]+0.005,jpmax[2]-0.005,5)
        else:
            print('joint idx is out of range!')
            return
        for idx,qm in enumerate(qseg):
            q[jidx] = qm
            self._send_master_traj(q,idx,True)
        q[jidx] = jpmax[jidx]
        self._send_master_traj(q,0,False)
        for idx,qm in enumerate(reversed(qseg)):
            q[jidx] = qm
            self._send_master_traj(q,idx,True)
        self._send_master_traj(q_ready,0,False)

        jpos_np = np.array(self.jpos)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_master_ratio'+str(jidx)+'_'+t+'.txt'
        np.savetxt(filename,jpos_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return jpos_np
    
    def sample_slave_gear_ratio(self,jidx:int):
        if not self.is_robot_ready():
            return
        q_ready = np.array([0.023,0.026,-0.5819,0.5819,0.005])
        jpmax = np.array([0.045,0.058,0.349,1.1,0.095])
        jpmin = np.array([0.002,0.002,-1.08,-0.349,0.005])
        q = self.jp[0:5].copy()
        q[jidx] = jpmin[jidx]
        self._send_slave_joint_traj(q,0,False)
        if jidx==0:
            qseg = np.linspace(jpmin[0]+0.005,jpmax[0]-0.005,5)
        elif jidx==1:
            qseg = np.linspace(jpmin[1]+0.005,jpmax[1]-0.005,5)
        elif jidx==4:
            qseg = np.linspace(jpmin[4]+0.005,jpmax[4]-0.01,5)
        else:
            print('joint idx is out of range!')
            return
        for idx,qs in enumerate(qseg):
            q[jidx] = qs
            self._send_slave_joint_traj(q,idx,True)
        q[jidx] = jpmax[jidx]
        self._send_slave_joint_traj(q,0,False)
        for idx,qs in enumerate(reversed(qseg)):
            q[jidx] = qs
            self._send_slave_joint_traj(q,idx,True)
        self._send_slave_joint_traj(q_ready,0,False)

        jpos_np = np.array(self.jpos)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_slave_ratio'+str(jidx)+'_'+t+'.txt'
        np.savetxt(filename,jpos_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return jpos_np

    def record_specific_rot(self):
        if not self.is_robot_ready():
            return
        self.start_record()
        self._send_slave_rpy_traj([[0,0],[10*D2R,0],[-10*D2R,0],[0,0]],1)
        self._send_slave_rpy_traj([[20*D2R,10*D2R],[-20*D2R,10*D2R],[0,0]],2)
        self._send_slave_rpy_traj_traj([[30*D2R,5*D2R],[-30*D2R,5*D2R],[0,0]],3)
        self._send_slave_rpy_traj([[0,10*D2R],[0,-10*D2R],[0,0]],4)
        self._send_slave_rpy_traj([[10*D2R,20*D2R],[10*D2R,-20*D2R],[0,0]],5)
        self._send_slave_rpy_traj([[5*D2R,30*D2R],[5*D2R,-30*D2R],[0,0]],6)
        self._send_slave_rpy_traj([[15*D2R,-15*D2R],[-15*D2R,15*D2R],[0,0]],7)
        self._send_slave_rpy_traj([[15*D2R,15*D2R],[-15*D2R,-15*D2R],[0,0]],8)
        self.stop_record()
        print('complete sample!!')

    def sample_random_rot(self):
        if not self.is_robot_ready():
            return
        rpy_list = []
        num = 100
        for idx in range(num):
            rpy = random_max_rpy(30*D2R)
            self._send_slave_rpy_traj([rpy.tolist()],idx+1)
            rpy_list.append(np.hstack((self.rpy[0:2],rpy,self.jp[2:4])))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_random_rot_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np
    
    def sample_concentric_rot(self):
        if not self.is_robot_ready():
            return
        self._send_slave_rpy_traj([[0,0]],0)
        deg_list = [5,10,20,30]
        rpy_list = []
        num = 100
        for deg in deg_list:
            for idx in range(num):
                rpy = random_rpy(deg*D2R)
                self._send_slave_rpy_traj([rpy.tolist()],idx+1)
                rpy_list.append(np.hstack((self.rpy[0:2],rpy,self.jp[2:4])))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_concentric_rot_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np

    def sample_rotx(self):
        if not self.is_robot_ready():
            return
        self._send_slave_rpy_traj([[0,0]],0)
        rpy_list = []
        num = 100
        for idx in range(num):
            alpha = random.uniform(-30*D2R,30*D2R)
            self._send_slave_rpy_traj([[alpha,0]],idx+1)
            rpy_list.append(np.hstack((self.rpy[0:2],np.array([alpha,0]),self.jp[2:4])))
        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_rotx_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np
    
    def sample_backlash_rot(self):
        if not self.is_robot_ready():
            return
        self._send_slave_rpy_traj([[0,0]],0)
        num_points = 10
        rpy_list = []
        for idx in range(num_points):
            while True:
                rpy = random_max_rpy(30*D2R,3)
                rpy_sort = np.sort(rpy,axis=0)
                alpha_diff = np.diff(rpy_sort[:,0])
                beta_diff = np.diff(rpy_sort[:,1])
                if np.random.rand()>0.5:
                    rpy_sort[:,1] = reversed(rpy_sort[:,1])
                theta = np.zeros(len(rpy))
                for idx,py in enumerate(rpy_sort):
                    rmat = smb.roty(py[1])@smb.rotx(py[0])
                    theta[idx],_ = smb.tr2angvec(rmat)
                if (theta<30*D2R).all() and (alpha_diff>6*D2R).all() and (beta_diff>6*D2R).all():
                    break
            for loop in range(4):
                rpy_send = rpy_sort[0] if loop%2==0 else rpy_sort[2]
                self._send_slave_rpy_traj([rpy_send.tolist()],loop+1)
                self._send_slave_rpy_traj([rpy_sort[1].tolist()],loop+1)
                rpy_list.append(np.hstack((self.rpy[0:2],rpy_sort[1],self.jp[2:4])))

        rpy_np = np.array(rpy_list)
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/sample_backlash_rot_'+t+'.txt'
        np.savetxt(filename,rpy_np,delimiter=' ',fmt='%.8f')
        print('save data successfully!')
        return rpy_np

    def sample_backlash_slave(self,jidx:int,is_prismatic:bool,sort=True):
        self._sample_backlash_joint(jidx,is_prismatic,'slave',sort)

    def sample_backlash_master(self,jidx:int,is_prismatic:bool,sort=True):
        self._sample_backlash_joint(jidx,is_prismatic,'master',sort)

    def _sample_backlash_joint(self,jidx:int,is_prismatic:bool,part:str,sort=True):
        if not self.is_robot_ready():
            return
        threshold_diff = 0.01 if is_prismatic else 5*D2R
        num_points = 100
        if part=='slave':
            q_ready = np.array([0.023,0.026,-0.5819,0.5819,0.005])
            jpmax = np.array([0.045,0.058,0.349,1.1,0.095])
            jpmin = np.array([0.005,0.005,-1.08,-0.349,0.005])
            q = self.jp[0:5].copy()
            jpos_cmd = np.zeros((num_points*2,5)) if sort else np.zeros((num_points,5))
        elif part=='master':
            q_ready = np.array([0,0,self.jp[7]])
            q0 = self.jp[jidx+5]
            jpmax = np.array([8*D2R, 8*D2R, q0+0.06])
            jpmin = np.array([-8*D2R, -8*D2R, q0+0.002])
            q = self.jp[5:].copy()
            jpos_cmd = np.zeros((num_points*2,3)) if sort else np.zeros((num_points,3))
        self._send_joint_traj(q_ready,0,False,part)
        for idx in range(num_points):
            if sort:
                while True:
                    qseg = jpmin[jidx]+(jpmax[jidx]-jpmin[jidx])*np.random.rand(3)
                    qseg = np.sort(qseg)
                    qdelta = np.diff(qseg)
                    if (qdelta>threshold_diff).all():
                        break
                q[jidx] = qseg[0]
                self._send_joint_traj(q,idx,False,part)
                q[jidx] = qseg[1]
                self._send_joint_traj(q,idx,True,part)
                jpos_cmd[2*idx] = q
                q[jidx] = qseg[2]
                self._send_joint_traj(q,idx,False,part)
                q[jidx] = qseg[1]
                self._send_joint_traj(q,idx,True,part)
                jpos_cmd[2*idx+1] = q
            else:
                while True:
                    q[jidx] = jpmin[jidx]+(jpmax[jidx]-jpmin[jidx])*np.random.rand()
                    if abs(q[jidx]-self.jp[jidx])>threshold_diff:
                        break
                self._send_joint_traj(q,idx,True,part)
                jpos_cmd[idx] = q


        self._send_joint_traj(q_ready,0,False,part)
        postfix = 'sort' if sort else 'nosort'
        jpos_np = np.hstack((np.array(self.jpos),jpos_cmd))
        filename = 'sample_backlash_'+part+str(jidx)+'_'+postfix
        self._save_data(jpos_np,filename)
        return jpos_np

    def start_record(self):
        pub_data = {"record":"start"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)

    def stop_record(self):
        pub_data = {"record":"stop"}
        self.mqtt_client.pub_record_data(json.dumps(pub_data),qos=2)
        time.sleep(1)

    def _send_joint_traj(self,via_pos,idx:int,with_aux:bool,part:str):
        if part=='slave':
            self._send_slave_joint_traj(via_pos,idx,with_aux)
        elif part=='master':
            self._send_master_traj(via_pos,idx,with_aux)

    def _send_slave_joint_traj(self,via_pos,idx:int,with_aux:bool):
        pub_data = {"name":"motion","slave":{"pos":[via_pos.tolist()],"relative":False,"type":"joint",
                                             "rcm-constraint":False,"joint-space":True,
                                             "vel_ratio":4.0,"acc_ratio":8.0}}
        self._send_traj(pub_data,idx,with_aux)

    def _send_slave_rpy_traj(self,via_pos,idx:int):
        pub_data = {"name":"motion","slave":{
            "pos":via_pos,"type":"rpy","relative":False,
            "rcm-constraint":False,"joint-space":False,
            "vel_ratio":3.0,"acc_ratio":6.0}}
        self._send_traj(pub_data,idx,False)

    def _send_master_traj(self,via_pos,idx:int,with_aux:bool):
        pub_data = {"name":"motion","master":{"pos":[via_pos.tolist()],"relative":False,
                                              "vel_ratio":3.0,"acc_ratio":6.0}}
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

    def _save_data(self,data,filename):
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        file = './data/'+filename+'_'+t+'.txt'
        np.savetxt(file,data,delimiter=' ',fmt='%.8f')
        print('save data successfully!')



def analy_calib_ratio_data(tdata,midx:int,aidx:int):
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
    ratio = np.hstack((gear_ratio_f,gear_ratio_b))
    r = ratio.mean()
    vr = max(ratio.max()-1.,1.-ratio.min())
    ra = ratio.max()-ratio.min()
    print(f'mean gear ratio is {r:.6f}')
    print(f'maximum variation of gear ratio is {vr:.6f}')
    print(f'maximum range of gear ratio is {ra:.6f}')
    return r

def analy_calib_ratio_file(filename,midx:int,aidx:int):
    if isinstance(filename,str):
        tdata = np.loadtxt(filename)
    elif isinstance(filename,list):
        tdata = np.vstack([np.loadtxt(f) for f in filename])
    analy_calib_ratio_data(tdata,midx,aidx)
    plt.figure()
    plt.plot(tdata[:,midx],label='motor encoder')
    plt.plot(tdata[:,aidx],label='auxiliary encoder')
    plt.legend()
    plt.grid()
    plt.show()

def analy_random_rot_data(vdata,title=None):
    num = vdata.shape[0]
    alpha_error = vdata[:,2]-vdata[:,0]
    beta_error = vdata[:,3]-vdata[:,1]
    theta = np.zeros(num)
    for idx in range(num):
        rot_fdb = smb.rpy2r(np.append(vdata[idx,0:2],0),'zyx')
        rot_cmd = smb.rpy2r(np.append(vdata[idx,2:4],0),'zyx')
        theta[idx],v = smb.tr2angvec(rot_cmd@rot_fdb.T)
    print(f'mean Rx error:{np.abs(alpha_error).mean()*R2D:.2f}degree,'
          f'maximum Rx error:{np.abs(alpha_error).max()*R2D:.2f}degree,'
          f'minimum Rx error:{np.abs(alpha_error).min()*R2D:.2f}degree')
    print(f'mean Ry error:{np.abs(beta_error).mean()*R2D:.2f}degree,'
          f'maximum Ry error:{np.abs(beta_error).max()*R2D:.2f}degree,'
          f'minimum Ry error:{np.abs(beta_error).min()*R2D:.2f}degree')
    print(f'mean theta error:{theta.mean()*R2D:.2f}degree,'
          f'maximum theta error:{theta.max()*R2D:.2f}degree,'
          f'mimium theta error:{theta.min()*R2D:.2f}degree')
    plt.figure()
    plt.plot(range(num),alpha_error*R2D,label='alpha_error')
    plt.plot(range(num),beta_error*R2D,label='beta_error')
    plt.plot(range(num),theta*R2D,label='theta')
    plt.grid(True)
    plt.legend(loc='upper right')
    if title is not None:
        plt.title(title)

def analy_random_rot_file(filename):
    td = np.loadtxt(filename)
    analy_random_rot_data(td)
    plt.show()

def analy_concentric_rot_file(filename):
    td = np.loadtxt(filename)
    analy_random_rot_data(td[0:100,:],'5degree error')
    analy_random_rot_data(td[100:200,:],'10degree error')
    analy_random_rot_data(td[200:300,:],'20degree error')
    analy_random_rot_data(td[300:400,:],'30degree error')
    plt.show()

def analy_backlash_rot_data(vdata):
    num_points = vdata.shape[0]
    alpha = vdata[:,0]
    beta = vdata[:,1]
    alpha_err = alpha[0::2]-alpha[1::2]
    beta_err = beta[0::2]-beta[1::2]
    plt.figure()
    plt.plot(alpha[0::2]*R2D,label='alpha+fdb')
    plt.plot(alpha[1::2]*R2D,label='alpha-fdb')
    plt.plot(vdata[0::2,2]*R2D,'--',label='alpha+cmd')
    plt.plot(vdata[1::2,2]*R2D,'--',label='alpha-cmd')
    plt.grid(True)
    plt.legend()
    plt.figure()
    plt.plot(beta[0::2]*R2D,label='beta+fdb')
    plt.plot(beta[1::2]*R2D,label='beta-fdb')
    plt.plot(vdata[0::2,3]*R2D,'--',label='beta+cmd')
    plt.plot(vdata[1::2,3]*R2D,'--',label='beta-cmd')
    plt.grid(True)
    plt.legend()
    plt.figure()
    plt.plot(alpha_err*R2D,label='alpha_err')
    plt.plot(beta_err*R2D,label='beta_err')
    plt.grid(True)
    plt.legend(loc='upper right')
    alpha_backlash = np.abs(alpha_err*R2D)
    beta_backlash = np.abs(beta_err*R2D)
    print(f'mean alpha backlash is {alpha_backlash.mean():.6f}deg')
    print(f'mean beta backlash is {beta_backlash.mean():.6f}deg')
    print(f'maximum alpha backlash is {alpha_backlash.max():.6f}deg')
    print(f'maximum beta backlash is {beta_backlash.max():.6f}deg')

def analy_backlash_joint_data(tdata,midx:int,aidx:int,is_prismatic:bool):
    unit = 'mm' if is_prismatic else 'deg'
    unit_trans = 1000 if is_prismatic else R2D
    dis = []
    pos_motor = tdata[:,midx]
    pos_aux = tdata[:,aidx]
    pos_err = np.abs(pos_aux[0::2]-pos_aux[1::2])
    b = pos_err.mean()*unit_trans
    bmax = pos_err.max()*unit_trans
    print(f'mean backlash is {b:.6f}{unit}')
    print(f'maximum backlash is {bmax:.6f}{unit}') 

def analy_backlash_rot_file(filename):
    if isinstance(filename,str):
        data_set = np.loadtxt(filename)
    elif isinstance(filename,list):
        data_set = np.vstack([np.loadtxt(file) for file in filename])
    analy_backlash_rot_data(data_set)
    plt.show()

def analy_backlash_joint_file(filename,midx:int,aidx:int,is_prismatic:bool):
    if isinstance(filename,str):
        tdata = np.loadtxt(filename)
    elif isinstance(filename,list):
        tdata = np.vstack([np.loadtxt(file) for file in filename])
    analy_backlash_joint_data(tdata,midx,aidx,is_prismatic)

def calib_gear_ratio():
    # rom = RobotOperationManager('192.168.2.242')
    # rom.sample_master_gear_ratio(2)
    # rom.sample_slave_gear_ratio(1)
    analy_calib_ratio_file('./data/calibrate_master_ratio2_2024-0911-134608.txt',midx=7,aidx=15)

def calib_rot_params():
    # rom = RobotOperationManager('192.168.2.242')
    # for _ in range(10):
        # rpy_data = rom.sample_random_rot()
    # analy_concentric_rot_file('./data/sample_roterr_data_2024-0717-102958.txt')
    analy_random_rot_file('./data/4#robot/sample_random_rot_2024-1108-125320.txt')

def calib_backlash():
    rom = RobotOperationManager('192.168.2.242')
    rom.sample_backlash_slave(4,is_prismatic=True,sort=True)
    # rom.sample_backlash_master(0,jtype='rotary')
    # rom.sample_backlash_rot()
    # analy_backlash_joint_file('./data/sample_backlash_slave1_2024-0912-134107.txt',
    #                           midx=1,aidx=9,is_prismatic=True)
    # analy_backlash_rot_file(['./data/sample_backlash_rot2024-0913-093650.txt',
    #                        './data/sample_backlash_rot_2024-0913-094411.txt'])
    # analy_backlash_rot_file(['./data/sample_repeaterr_data_2024-0729-183225.txt',
    #                       './data/sample_repeaterr_data_2024-0729-191327.txt',
    #                       './data/sample_repeaterr_data_2024-0729-181516.txt',
    #                       './data/sample_repeaterr_data_2024-0729-194525.txt'])
    # analy_backlash_rot_file(['./data/sample_repeaterr_data_2024-0801-164133.txt',
    #                       './data/sample_repeaterr_data_2024-0801-165257.txt',
    #                       './data/sample_repeaterr_data_2024-0801-170719.txt',
    #                       './data/sample_repeaterr_data_2024-0801-171958.txt',
    #                       './data/sample_repeaterr_data_2024-0801-173023.txt'])


if __name__=='__main__':
    # calib_gear_ratio()
    # calib_backlash()
    calib_rot_params()
    # rom = RobotOperationManager('192.168.2.242')
    # rom.sample_random_rot()
    # rom.run_random_joint()
