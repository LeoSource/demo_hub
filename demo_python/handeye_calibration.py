# !/bin/python3

import numpy as np
import cv2
import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb

np.set_printoptions(suppress=True)

def calibrate_handineye():
    rt_cam2gripper_ref = sm.SE3.Trans([5,-6,7])*sm.SE3.Rx(10,'deg')
    
    num_sample = 20
    rt_target2cam = []
    rt_gripper2base = [sm.SE3.Rand()]
    for idx in range(num_sample):
        rt_target2cam.append(sm.SE3.Rand())

    for idx in range(num_sample-1):
        mat_B = rt_target2cam[idx+1]*rt_target2cam[idx].inv()
        mat_A = rt_cam2gripper_ref*mat_B*rt_cam2gripper_ref.inv()
        rt_gripper2base.append(rt_gripper2base[idx]*mat_A.inv())

    r_target2cam = []
    t_target2cam = []
    r_gripper2base = []
    t_gripper2base = []
    for idx in range(num_sample):
        r_target2cam.append(rt_target2cam[idx].R)
        t_target2cam.append(rt_target2cam[idx].t)
        r_gripper2base.append(rt_gripper2base[idx].R)
        t_gripper2base.append(rt_gripper2base[idx].t)

    r_cam2gripper_calc,t_cam2gripper_calc = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_HORAUD)
    
    print('refrence rotation matrix:')
    print(rt_cam2gripper_ref.R)
    print('calculation rotatin matrix:')
    print(r_cam2gripper_calc)
    print('refrence translation vector:')
    print(rt_cam2gripper_ref.t)
    print('calculation translation vector:')
    print(t_cam2gripper_calc)

def calibrate_handtoeye():
    t_cam2base_ref = sm.SE3.Trans(3,6,-10)
    r_cam2base_ref = sm.SE3.Rx(10,'deg')*sm.SE3.Ry(-30,'deg')
    rt_cam2base_ref = t_cam2base_ref*r_cam2base_ref

    num_sample = 20
    rt_target2cam = []
    for idx in range(num_sample):
        rt_target2cam.append(sm.SE3.Rand())

    rt_gripper2base = [sm.SE3.Rand()]
    for idx in range(num_sample-1):
        mat_B = rt_target2cam[idx+1]*rt_target2cam[idx].inv()
        mat_A = rt_cam2base_ref*mat_B*rt_cam2base_ref.inv()
        rt_gripper2base.append(rt_gripper2base[idx]*mat_A.inv())

    r_target2cam = []
    t_target2cam = []
    r_gripper2base = []
    t_gripper2base = []
    for idx in range(num_sample):
        r_target2cam.append(rt_target2cam[idx].R)
        t_target2cam.append(rt_target2cam[idx].t)
        r_gripper2base.append(rt_gripper2base[idx].R)
        t_gripper2base.append(rt_gripper2base[idx].t)

    r_cam2base_calc,t_cam2base_calc = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_HORAUD)
    
    print('refrence rotation matrix:')
    print(rt_cam2base_ref.R)
    print('calculation rotation matrix:')
    print(r_cam2base_calc)
    print('refrence translation vector:')
    print(rt_cam2base_ref.t)
    print('calculation translation vector:')
    print(t_cam2base_calc)


if __name__ == '__main__':
    # calibrate_handineye()
    # calibrate_handtoeye()
    r1 = smb.rpy2r([1,2,3])
    r2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
    r3 = r1*r2
    print(r3)
    # print(smb.rpy2r(1,2,3))
    # print(smb.angvec2r(0.7,[1,0,0]))