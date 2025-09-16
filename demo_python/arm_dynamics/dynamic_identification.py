#!/usr/bin/env python3

import os
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import pinocchio as pin
from optimal_trajectory import ArmDynamicsWrapper, FourierTrajectory


def awgn(x, snr):
    """
    Additive White Gaussian Noise
    :param x: Raw Signal (vector)
    :param snr: SNR (decibel)
    :return: Received Signal With AWGN
    """
    snr = 10 ** (snr / 10.0)
    xpower = np.sum(x ** 2) / len(x)
    npower = xpower / snr
    if isinstance(x[0], complex):
        noise = (np.random.randn(len(x)) + 1j * np.random.randn(len(x))) * np.sqrt(0.5 * npower)  # Complex Number
    else:
        noise = np.random.randn(len(x)) * np.sqrt(npower)  # Real Number
    return x + noise

def process_dyn_iden_data(pos_raw:np.ndarray, vel_raw:np.ndarray, tau_raw:np.ndarray, fs:float, avg:int):
    """
    动力学数据预处理
    
    参数:
        pos_raw: 原始位置数据 (形状: [时间点数, 关节数])
        tau_raw: 原始力矩数据 (形状: [时间点数, 关节数])
        fs: 采样频率 (Hz)
        avg: 平均窗口大小
    
    返回:
        位置、速度、加速度、力矩的降采样结果 (形状: [处理后时间点数, 关节数])
    """
    max_idx = (pos_raw.shape[0]//avg)*avg
    nj = pos_raw.shape[1]
    pos_data = pos_raw[:max_idx,:]
    vel_data = vel_raw[:max_idx,:]
    tau_data = tau_raw[:max_idx,:]
    acc_data = np.zeros_like(vel_data)
    acc_data[1:-1] = (vel_data[2:]-vel_data[:-2])*fs/2
    acc_data[0] = (vel_data[1]-vel_data[0])*fs
    acc_data[-1] = (vel_data[-1]-vel_data[-2])*fs

    N = 5
    ff_torque = 10
    B_torque,A_torque = signal.butter(N, ff_torque/(fs/2), btype='lowpass')
    ff_motion = 5
    B_motion,A_motion = signal.butter(N, ff_motion/(fs/2), btype='lowpass')

    vel_filtered = signal.filtfilt(B_motion, A_motion, vel_data, axis=0)
    acc_filtered = signal.filtfilt(B_motion, A_motion, acc_data, axis=0)
    tau_filtered = signal.filtfilt(B_torque, A_torque, tau_data, axis=0)

    def downsample(data):
        n_samples,n_joints = data.shape
        n_samples_avg = n_samples//avg
        return data[:n_samples_avg*avg].reshape(n_samples_avg, avg, n_joints).mean(axis=1)

    return downsample(pos_data),downsample(vel_filtered),downsample(acc_filtered),downsample(tau_filtered)

def generate_simulation_data(model, params_folder, params_files, fs, with_noise):
    dof = 5
    jpos,jvel,jacc,jtau = [], [], [], []
    for file_name in params_files:
        params_file = os.path.join(params_folder, file_name)
        params = np.loadtxt(params_file, delimiter=',')
        order = (len(params)/dof-1)/2
        fourier_traj = FourierTrajectory(dof=dof,order=int(order),w0=2*np.pi*0.1)
        time_samples = np.linspace(0, 10, 10*fs)
        q, qd, qdd = fourier_traj.generate_traj(params, time_samples)
        tau_tmp = []
        for idx in range(q.shape[0]):
            tau_tmp.append(model.rnea(q[idx], qd[idx], qdd[idx]))
        jpos.append(q)
        jvel.append(qd)
        jacc.append(qdd)
        jtau.append(np.array(tau_tmp))
    jpos,jvel,jacc,jtau = np.vstack(jpos), np.vstack(jvel), np.vstack(jacc), np.vstack(jtau)

    pos_snr=70
    vel_snr=40
    acc_snr=20
    tau_snr=20
    jpos_noise,jvel_noise,jacc_noise,jtau_noise = np.zeros_like(jpos),np.zeros_like(jvel),np.zeros_like(jacc),np.zeros_like(jtau)
    for jidx in range(dof):
        jpos_noise[:,jidx] = awgn(jpos[:,jidx],pos_snr)
        jvel_noise[:,jidx] = awgn(jvel[:,jidx],vel_snr)
        jacc_noise[:,jidx] = awgn(jacc[:,jidx],acc_snr)
        jtau_noise[:,jidx] = awgn(jtau[:,jidx],tau_snr)

    if with_noise:
        return jpos_noise,jvel_noise,jacc_noise,jtau_noise
    else:
        return jpos,jvel,jacc,jtau

def identify_min_dyn_params(model,jpos,jvel,jacc,jtau):
    Y = []
    for idx in range(jpos.shape[0]):
        reg = model.calc_min_regressor(jpos[idx], jvel[idx], jacc[idx])
        Y.append(reg)
    Y = np.vstack(Y)
    jtau_flat = jtau.flatten()
    assert Y.shape[0] == jtau_flat.shape[0]

    min_dyn_params = np.linalg.inv(Y.T @ Y)@Y.T@jtau_flat

    tau_pred = Y@min_dyn_params
    residuals = jtau_flat - tau_pred
    mse = np.mean(residuals**2)
    print(f'minimum dynamics parameter sets: {min_dyn_params}')
    print(f'identification error: {mse}')
    return min_dyn_params, tau_pred.reshape(-1,5)

def identify_left_arm():
    model = ArmDynamicsWrapper('left')
    model.get_dyn_params_dep()
    params_folder = './left_fourier_params'
    params_files = [
        'fourier_params_order_3_0.csv',
        'fourier_params_order_3_1.csv',
        'fourier_params_order_3_2.csv',
        'fourier_params_order_3_3.csv',
        'fourier_params_order_3_4.csv'
    ]
    fs = 250
    jpos,jvel,jacc,jtau = generate_simulation_data(model, params_folder, params_files, fs, True)
    down_avg = 2
    jpos_filt,jvel_filt,jacc_filt,jtau_filt = process_dyn_iden_data(jpos, jvel, jtau, fs, down_avg)
    min_dyn_params,tau_pred = identify_min_dyn_params(model, jpos_filt, jvel_filt, jacc_filt, jtau_filt)
    np.savetxt(f'left_min_dyn_params.csv', min_dyn_params, delimiter=',', fmt='%.10f')
    visualize(jtau, tau_pred, 10, int(10/down_avg))

def identify_right_arm():
    model = ArmDynamicsWrapper('right')
    model.get_dyn_params_dep()
    params_folder = 'right_fourier_params'
    params_files = [
        'fourier_params_order_5_0.csv',
        'fourier_params_order_5_1.csv',
        'fourier_params_order_5_2.csv',
        'fourier_params_order_5_3.csv',
        'fourier_params_order_5_4.csv'
    ]
    fs = 250
    jpos,jvel,jacc,jtau = generate_simulation_data(model, params_folder, params_files, fs, True)
    down_avg = 2
    jpos_filt,jvel_filt,jacc_filt,jtau_filt = process_dyn_iden_data(jpos, jvel, jtau, fs, down_avg)
    min_dyn_params, tau_pred = identify_min_dyn_params(model, jpos_filt, jvel_filt, jacc_filt, jtau_filt)
    np.savetxt(f'right_min_dyn_params.csv', min_dyn_params, delimiter=',', fmt='%.10f')
    visualize(jtau, tau_pred, 10, int(10/down_avg))

def validate_left_arm():
    model = ArmDynamicsWrapper('left')
    model.get_dyn_params_dep()
    params_folder = './left_fourier_params'
    params_files = ['fourier_params_order_5_5.csv',
                    'fourier_params_order_5_6.csv',
                    'fourier_params_order_5_7.csv',
                    'fourier_params_order_5_8.csv',
                    'fourier_params_order_5_9.csv'
                    ]    
    fs = 250
    jpos,jvel,jacc,jtau = generate_simulation_data(model, params_folder, params_files, fs, True)
    Y = []
    for idx in range(jpos.shape[0]):
        reg = model.calc_min_regressor(jpos[idx], jvel[idx], jacc[idx])
        Y.append(reg)
    Y = np.vstack(Y)

    min_dyn_params = np.loadtxt('left_min_dyn_params.csv',delimiter=',')
    jtau_pred = Y@min_dyn_params
    residuals = jtau.flatten() - jtau_pred
    mse = np.mean(residuals**2)
    print(f'minimum dynamics parameter sets: {min_dyn_params}')
    print(f'identification error: {mse}')
    visualize(jtau, jtau_pred.reshape(-1,5), 10, 10)

def validate_right_arm():
    model = ArmDynamicsWrapper('right')
    model.get_dyn_params_dep()
    params_folder = './right_fourier_params'
    params_files = ['fourier_params_order_5_5.csv',
                    'fourier_params_order_5_6.csv',
                    'fourier_params_order_5_7.csv',
                    'fourier_params_order_5_8.csv',
                    'fourier_params_order_5_9.csv'
                    ]    
    fs = 250
    jpos,jvel,jacc,jtau = generate_simulation_data(model, params_folder, params_files, fs, True)
    Y = []
    for idx in range(jpos.shape[0]):
        reg = model.calc_min_regressor(jpos[idx], jvel[idx], jacc[idx])
        Y.append(reg)
    Y = np.vstack(Y)

    min_dyn_params = np.loadtxt('right_min_dyn_params.csv',delimiter=',')
    jtau_pred = Y@min_dyn_params
    residuals = jtau.flatten() - jtau_pred
    mse = np.mean(residuals**2)
    print(f'minimum dynamics parameter sets: {min_dyn_params}')
    print(f'identification error: {mse}')
    visualize(jtau, jtau_pred.reshape(-1,5), 10, 10)



def visualize(jtau_sample, jtau_pred, avg_sample, avg_pred):
    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    for i in range(5):
        axes[i].plot(jtau_sample[::avg_sample, i], label='Sampled Torque', alpha=0.8)
        axes[i].plot(jtau_pred[::avg_pred, i], label='Predicted Torque', linestyle='--', alpha=0.8)
        
        axes[i].set_title(f'Joint {i+1}')
        axes[i].set_ylabel('Torque (Nm)')
        axes[i].grid(True)
        axes[i].legend()

    # 添加全局标签
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    plt.show()



if __name__ == '__main__':
    # identify_left_arm()
    validate_left_arm()
    # identify_right_arm()
    # validate_right_arm()



