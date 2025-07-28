#!/usr/bin/env python3

import os
import pinocchio as pin
from pinocchio_common import load_a2_t2d0_flagship_left_serial_arm, load_a2_t2d0_flagship_right_serial_arm
import numpy as np
from scipy.optimize import minimize, Bounds
from scipy.linalg import svd
from numpy.linalg import qr, matrix_rank, norm
import time
import matplotlib.pyplot as plt
from tqdm import tqdm
import math

class ArmDynamicsWrapper:
    def __init__(self, arm_name:str):
        if arm_name=='left':
            self.model, self.geom_model, data, geom_data, urdf_file, robot = load_a2_t2d0_flagship_left_serial_arm(verbose=False)
        elif arm_name=='right':
            self.model, self.geom_model, data, geom_data, urdf_file, robot = load_a2_t2d0_flagship_right_serial_arm(verbose=False)
        else:
            print('wrong arm name!')

    def get_dyn_params_dep(self, samples=1000):
        nj = self.model.nq
        Z = []
        for idx in range(samples):
            jpos = np.random.uniform(self.model.lowerPositionLimit,self.model.upperPositionLimit)
            jvel = np.random.uniform(-np.ones(5),np.ones(5))
            jacc = np.random.uniform(-2*np.ones(5),2*np.ones(5))
            reg = pin.computeJointTorqueRegressor(self.model, self.model.createData(), jpos, jvel, jacc)
            Z.append(reg)
        Z = np.vstack(Z)

        Q,R = qr(Z)
        R1_diag = np.diag(R)

        dbi = []
        ddi = []
        for i, val in enumerate(R1_diag):
            if abs(val) > 1e-6:
                dbi.append(i)
            else:
                ddi.append(i)
        print(matrix_rank(R))
        print(len(dbi))
        dbn = len(dbi)
        per_mat = np.eye(nj*10)
        col_order = dbi+ddi
        per_mat = per_mat[:,col_order]
        _, Rbd = qr(Z.dot(per_mat))
        Rb = Rbd[:dbn,:dbn]
        Rd = Rbd[:dbn,dbn:]

        self.dyn_idx = dbi
        self.pb = per_mat[:,:dbn]
        self.pd = per_mat[:,dbn:]
        self.kd = np.linalg.lstsq(Rb, Rd, rcond=None)[0]  # Solve Rb * kd = Rd

    # https://zhuanlan.zhihu.com/p/549740247
    def get_dyn_params_dep1(self, samples=1000):
        Y = []
        for _ in range(samples):
            jpos = np.random.uniform(self.model.lowerPositionLimit,self.model.upperPositionLimit)
            jvel = np.random.uniform(-np.ones(5),np.ones(5))
            jacc = np.random.uniform(-2*np.ones(5),2*np.ones(5))
            reg = pin.computeJointTorqueRegressor(self.model, self.model.createData(), jpos, jvel, jacc)
            Y.append(reg)
        Y = np.vstack(Y)
        Q,R = qr(Y)
        R1_diag = np.diag(R)

        dbi = []
        ddi = []
        for i, val in enumerate(R1_diag):
            if abs(val) > 1e-8:
                dbi.append(i)
            else:
                ddi.append(i)
        self.dbi = dbi
        self.ddi = ddi
        print(matrix_rank(R))
        print(len(dbi))
        dbn = len(dbi)
        Y1 = Y[:,dbi]
        Y2 = Y[:,ddi]
        _,R = qr(np.hstack([Y1,Y2]))
        R1 = R[:dbn,:dbn]
        R2 = R[:dbn,dbn:]
        self.beta = np.linalg.inv(R1).dot(R2)

    def calc_min_regressor(self, q, qd, qdd):
        base_reg = pin.computeJointTorqueRegressor(self.model, self.model.createData(), q, qd, qdd)
        return base_reg[:,self.dyn_idx]
    
    def get_all_dyn_params(self):
        all_dyn_params = []
        for i in range(self.model.nq):
            all_dyn_params.append(self.model.inertias[i+1].toDynamicParameters())
        return np.concatenate(all_dyn_params)

    def get_min_dyn_params(self):
        all_dyn_params = self.get_all_dyn_params()
        min_dyn_params = (self.pb.T+self.kd.dot(self.pd.T)).dot(all_dyn_params)
        return min_dyn_params

    def get_min_dyn_params1(self):
        all_dyn_params = self.get_all_dyn_params()
        min_dyn_params = all_dyn_params[self.dbi]+self.beta.dot(all_dyn_params[self.ddi])
        return min_dyn_params    

    def get_max_pos(self):
        return self.model.upperPositionLimit
    
    def get_min_pos(self):
        return self.model.lowerPositionLimit

    def computeCollisions(self,q):
        pin.updateGeometryPlacements(self.model, self.model.createData(), self.geom_model, self.geom_model.createData(), q)
        return pin.computeCollisions(self.model, self.model.createData(), self.geom_model, self.geom_model.createData(), q, True)

    def rnea(self, q, qd, qdd):
        return pin.rnea(self.model, self.model.createData(), q, qd, qdd)


class FourierTrajectory:
    """
    q(t) = q0 + Σ [a_ksin(w_kt) + b_kcos(w_kt)]/w_k
    q'(t) = Σ [a_kcos(w_kt) - b_ksin(w_kt)]
    q''(t) = -Σ w_k[a_ksin(w_kt) + b_kcos(w_kt)]
    """
    def __init__(self, dof, order, w0):
        self.dof = dof
        self.order = order
        self.w0 = w0
        self.nparams = dof*(2*order+1)

    def generate_point(self, params, t):
        q = np.zeros(self.dof)
        qd = np.zeros(self.dof)
        qdd = np.zeros(self.dof)
        for j in range(self.dof):
            idx_start = j*(2*self.order+1)
            q0 = params[idx_start]
            a_coeffs = params[idx_start+1:idx_start+1+self.order]
            b_coeffs = params[idx_start+1+self.order:idx_start+1+2*self.order]
            q[j] = q0
            for k,(a,b) in enumerate(zip(a_coeffs,b_coeffs)):
                wi = self.w0*(k+1)
                q[j] += a/wi*np.sin(wi*t)+b/wi*np.cos(wi*t)
                qd[j] += a*np.cos(wi*t)-b*np.sin(wi*t)
                qdd[j] += (-a*wi*np.sin(wi*t)-b*wi*np.cos(wi*t))
        return q,qd,qdd

    def generate_traj(self, params, time_samples):
        jpos = []
        jvel = []
        jacc = []
        for t in time_samples:
            q,qd,qdd = self.generate_point(params,t)
            jpos.append(q)
            jvel.append(qd)
            jacc.append(qdd)
        return np.vstack(jpos), np.vstack(jvel), np.vstack(jacc)
    


class TrajectoryOptimizer:
    def __init__(self, robot, order, w0, num_pts, q_limit):
        self.robot = robot
        self.robot.get_dyn_params_dep1()
        self.robot.get_dyn_params_dep()
        self.fourier_traj = FourierTrajectory(dof=robot.model.nq,order=order,w0=w0)
        self.num_pts = num_pts
        self.qmax = q_limit[0]
        self.qmin = q_limit[1]
        self.qdmax = q_limit[2]
        self.qdmin = q_limit[3]
        self.qddmax = q_limit[4]
        self.qddmin = q_limit[5]


    def obj_function(self, x):
        time_samples = np.linspace(0, 10, self.num_pts)
        q,qd,qdd = self.fourier_traj.generate_traj(x,time_samples)

        obj_mat = []
        for idx in range(self.num_pts):
            reg  = self.robot.calc_min_regressor(q[idx], qd[idx], qdd[idx])
            obj_mat.append(reg)
        obj_mat = np.vstack(obj_mat)
            
        # 计算条件数
        try:
            cond = np.linalg.cond(obj_mat)
            s = svd(obj_mat, compute_uv=False)
            # print(f'current parameters: {x[:11]}')
            return cond
        except:
            # 如果矩阵奇异，返回一个大的值
            return 1e10

    def kinematics_nonlin_constraints(self, x):
        # 位置、速度、加速度上下限
        time_samples = np.linspace(0, 10, self.num_pts)
        q,qd,qdd = self.fourier_traj.generate_traj(x,time_samples)

        conqmax = []
        conqmin = []
        conqdmax = []
        conqdmin = []
        conqddmax = []
        conqddmin = []
        for idx in range(self.num_pts):
            conqmax.append(self.qmax-q[idx])
            conqmin.append(q[idx]-self.qmin)
            conqdmax.append(self.qdmax-qd[idx])
            conqdmin.append(qd[idx]-self.qdmin)
            conqddmax.append(self.qddmax-qdd[idx])
            conqddmin.append(qdd[idx]-self.qddmin)
        c = np.concatenate([conqmax,conqmin,conqdmax,conqdmin,conqddmax,conqddmin]).flatten()
        return c

    def kinematics_lin_constraints(self,x):
        # 初始速度和加速度为零
        _,qd0,qdd0 = self.fourier_traj.generate_point(x,0)
        ceq = np.concatenate([qd0,qdd0]).flatten()
        return ceq

    def collision_constraint(self,x):
        pass

    def start_optimize(self):
        nj = self.robot.model.nq
        nparams = self.fourier_traj.nparams
        x_init = 0.5*(2*np.random.rand(nparams) - np.ones(nparams))
            
        # 定义等式约束
        constraints = []
        constraints.append({
            'type': 'eq',
            'fun': self.kinematics_lin_constraints
        })
            
        # 添加不等式约束
        constraints.append({
            'type': 'ineq',
            'fun': self.kinematics_nonlin_constraints
        })
        
        # 设置优化选项
        options = {
            'maxiter': 15000,
            'disp': True
        }
        
        # 执行优化
        result = minimize(
            fun=self.obj_function,
            x0=x_init,
            method='SLSQP',
            constraints=constraints,
            options=options
        )

        return result


def generate_optimal_traj(arm_name):
    model = ArmDynamicsWrapper(arm_name)
    qmax = model.get_max_pos()-0.2*(model.get_max_pos()-model.get_min_pos())
    qmin = model.get_min_pos()+0.2*(model.get_max_pos()-model.get_min_pos())
    qdmax = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
    qdmin = -qdmax
    qddmax = np.array([np.pi, np.pi, np.pi, np.pi, np.pi])
    qddmin = -qddmax
    qlimit = [qmax,qmin,qdmax,qdmin,qddmax,qddmin]
    order_list = [3, 5]
    w0 = 2*np.pi*0.1
    opt_pts = 100
    for order in order_list:
        traj_optimizer = TrajectoryOptimizer(model, order, w0, opt_pts, qlimit)
        for traj_idx in range(10):
            result = traj_optimizer.start_optimize()
            if result.success:
                np.savetxt(f'fourier_params_order_{order}_{traj_idx}.csv', result.x, delimiter=',', fmt='%.6f')

def validate_fourier_params(arm_name, params_folder):
    model = ArmDynamicsWrapper(arm_name)
    order_3_files = []
    order_5_files = []
    for filename in os.listdir(params_folder):
        parts = filename.split('_')
        if len(parts)>=5 and parts[2]=='order':
            order = parts[3]
            file_path = os.path.join(params_folder, filename)
            if order=='3':
                order_3_files.append(file_path)
            elif order=='5':
                order_5_files.append(file_path)

    dof = 5
    t = np.linspace(0, 10, 2000)
    valid_idx = []
    for file_idx,params_file in enumerate(order_3_files):
        params = np.loadtxt(params_file, delimiter=',')
        fourier_traj = FourierTrajectory(dof=dof,order=3,w0=2*np.pi*0.1)
        q, _, _ = fourier_traj.generate_traj(params, t)
        for idx in range(q.shape[0]):
            collision = model.computeCollisions(q[idx])
            if collision:
                break
        valid_idx.append(file_idx)
    print(f'valid fourier parameters file of order 3 is: ')
    print(valid_idx)

    valid_idx.clear()
    for file_idx,params_file in enumerate(order_5_files):
        params = np.loadtxt(params_file, delimiter=',')
        fourier_traj = FourierTrajectory(dof=dof,order=5,w0=2*np.pi*0.1)
        q, _, _ = fourier_traj.generate_traj(params, t)
        for idx in range(q.shape[0]):
            collision = model.computeCollisions(q[idx])
            if collision:
                break
        valid_idx.append(file_idx)
    print(f'valid fourier parameters file of order 5 is: ')
    print(valid_idx)

def visualize(params_file):
    params = np.loadtxt(params_file, delimiter=',')
    dof = 5
    order = (len(params)/dof-1)/2
    t = np.linspace(0, 10, 500)
    fourier_traj = FourierTrajectory(dof=dof,order=int(order),w0=2*np.pi*0.1)
    q, qd, qdd = fourier_traj.generate_traj(params, t)
    # 创建绘图窗口
    nj = q.shape[1]
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 10))

    # 绘制位置
    for i in range(nj):
        axes[0].plot(t, q[:, i], label=f'Joint {i+1}')
    axes[0].set_title('Joint Positions')
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Position [rad]')
    axes[0].grid(True)
    axes[0].legend()

    # 绘制速度
    for i in range(nj):
        axes[1].plot(t, qd[:, i], label=f'Joint {i+1}')
    axes[1].set_title('Joint Velocities')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Velocity [rad/s]')
    axes[1].grid(True)
    axes[1].legend()

    # 绘制加速度
    for i in range(nj):
        axes[2].plot(t, qdd[:, i], label=f'Joint {i+1}')
    axes[2].set_title('Joint Accelerations')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Acceleration [rad/s²]')
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()

def test_min_regressor():
    arm_dyn_wrapper = ArmDynamicsWrapper('left')
    all_dyn_params = arm_dyn_wrapper.get_all_dyn_params()
    arm_dyn_wrapper.get_dyn_params_dep()
    min_dyn_params = arm_dyn_wrapper.get_min_dyn_params()

    arm_dyn_wrapper.get_dyn_params_dep1()
    min_dyn_params1 = arm_dyn_wrapper.get_min_dyn_params1()
    print(norm(min_dyn_params-min_dyn_params1))
    print(arm_dyn_wrapper.dyn_idx)
    print(arm_dyn_wrapper.dbi)

    q = np.random.uniform(arm_dyn_wrapper.get_min_pos(), arm_dyn_wrapper.get_max_pos())
    qd = np.zeros(len(q))
    qdd = np.zeros(len(q))
    tau_raw = arm_dyn_wrapper.rnea(q,qd,qdd)
    reg_min = arm_dyn_wrapper.calc_min_regressor(q,qd,qdd)
    tau_reg = reg_min@min_dyn_params
    print(tau_raw-tau_reg)
    
if __name__ == "__main__":
    # test_min_regressor()
    # generate_optimal_traj('right')
    # visualize('./left_fourier_params/fourier_params_order_3_1.csv')
    validate_fourier_params('right','./right_fourier_params')






