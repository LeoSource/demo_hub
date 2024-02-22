# -*- coding: utf-8 -*-
# @File   		: EvaluateTransformMatrix.py
# @Description	: evaluate transform matrix between robot and CT
# @Date   		: 2024/02/02 14:33:09
# @Author	    : zxliao, zhixiangleo@163.com

import numpy as np
import spatialmath.base as smb
import itertools
import sys

def norm_rand_vector(dim):
    res = -np.ones((dim,1))+2*np.ones((dim,1))*np.random.rand(dim,1)
    return res/np.linalg.norm(res)

def points2cs(points):
    xn = points[:,1]-points[:,0]
    xn = xn/np.linalg.norm(xn)
    zn = np.cross(points[:,2]-points[:,0],points[:,1]-points[:,0])
    zn = zn/np.linalg.norm(zn)
    yn = np.cross(zn,xn)
    rot = np.array([xn,yn,zn]).T

    return points[:,0:1].reshape(-1,1),rot

def calc_transform(pb,pct,verbose):
    num_idx = np.arange(pb.shape[1])
    # calc_idx = np.array(list(itertools.combinations(num_idx,3)))
    calc_idx = list(itertools.combinations(num_idx,3))
    pb_calc = []
    pct_calc = []
    for idx in range(len(calc_idx)):
        pb_calc.append(pb[:,calc_idx[idx]])
        pct_calc.append(pct[:,calc_idx[idx]])
    pbm = np.zeros(shape=(3,3*len(pb_calc)))
    rbm = np.zeros(shape=(3*len(pb_calc),3,3))
    pcm = np.zeros_like(pbm)
    rcm = np.zeros_like(rbm)
    for idx in range(len(pb_calc)):
        pbm[:,3*idx:3*idx+1],rbm[3*idx,:,:] = points2cs(pb_calc[idx][:,(0,1,2)])
        pbm[:,3*idx+1:3*idx+2],rbm[3*idx+1,:,:] = points2cs(pb_calc[idx][:,(2,0,1)])
        pbm[:,3*idx+2:3*idx+3],rbm[3*idx+2,:,:] = points2cs(pb_calc[idx][:,(1,2,0)])

        pcm[:,3*idx:3*idx+1],rcm[3*idx,:,:] = points2cs(pct_calc[idx][:,(0,1,2)])
        pcm[:,3*idx+1:3*idx+2],rcm[3*idx+1,:,:] = points2cs(pct_calc[idx][:,(2,0,1)])
        pcm[:,3*idx+2:3*idx+3],rcm[3*idx+2,:,:] = points2cs(pct_calc[idx][:,(1,2,0)])

    pbc = np.zeros((3,1))
    rbc = np.eye(3)
    it_times = 0
    dr = np.ones((6,1))
    while np.linalg.norm(dr)>1e-6:
        if it_times>20:
            print('calibration fail!')
            sys.exit()

        nidx = pbm.shape[1]
        pos_err = np.zeros((3,nidx))
        rot_err = np.zeros_like(pos_err)
        pose_err = np.zeros((6,nidx))
        jaco = np.zeros((6*nidx,6))
        for idx in range(nidx):
            pos_err[:,idx:idx+1] = pbm[:,idx:idx+1]-(pbc+rbc@pcm[:,idx:idx+1])
            theta,k = smb.tr2angvec(rbm[idx,:,:]@(rbc@rcm[idx,:,:]).T)
            rot_err[:,idx:idx+1] = theta*k.reshape(-1,1)
            pose_err[:,idx:idx+1] = np.concatenate((pos_err[:,idx:idx+1],rot_err[:,idx:idx+1]))
            jaco[6*idx:6*idx+6,:] = np.eye(6)

        dr = np.linalg.pinv(jaco)@pose_err.flatten(order='F').reshape(-1,1)
        pbc += dr[0:3]
        rbc = smb.angvec2r(np.linalg.norm(dr[3:6]),dr[3:6]/np.linalg.norm(dr[3:6]))@rbc
        it_times += 1
        if verbose:
            print('pose error: %f' %(np.linalg.norm(pose_err.flatten(order='F'))))
    # if verbose:
    #     print('\n')

    return pbc,rbc


def analysis_transform_error(pb,pbc_ref,rbc_ref,verbose):
    pct = []
    pct_sample = []
    for idx in range(pb.shape[1]):
        pct.append(rbc_ref.T@(pb[:,idx:idx+1]-pbc_ref))
        pct_sample.append((pct[idx]+norm_rand_vector(3)*np.random.normal(0.3,0.002)).reshape(-1,1))

    pct_sample = np.squeeze(np.array(pct_sample)).T
    pbc_sample,rbc_sample = calc_transform(pb,pct_sample,False)
    theta,k = smb.tr2angvec(rbc_ref@rbc_sample.T)

    pb_sample = np.zeros((3,pb.shape[1]))
    for idx in range(pb.shape[1]):
        pb_sample[:,idx:idx+1] = pbc_sample+rbc_sample@pct_sample[:,idx:idx+1]

    xmin = pct[1][0]+40
    xmax = pct[1][0]+120
    ymin = pct[1][1]+40
    ymax = pct[1][1]+100
    zmin = pct[1][2]+30
    zmax = pct[1][2]+100
    pmin = np.array([xmin,ymin,zmin])
    pmax = np.array([xmax,ymax,zmax])
    num_points = 1000
    p = pmin+np.random.rand(3,num_points)*(pmax-pmin)
    pbm_ref = np.zeros((3,num_points))
    pbm_sample = np.zeros_like(pbm_ref)
    pos_err = np.zeros_like(pbm_ref)
    for idx in range(num_points):
        pbm_ref[:,idx:idx+1] = pbc_ref+rbc_ref@p[:,idx:idx+1]
        pbm_sample[:,idx:idx+1] = pbc_sample+rbc_sample@p[:,idx:idx+1]
        pos_err[:,idx:idx+1] = pbm_ref[:,idx:idx+1]-pbm_sample[:,idx:idx+1]

    mean_err = np.mean(np.linalg.norm(pos_err,axis=0))
    max_err = np.max(np.linalg.norm(pos_err,axis=0))
    if verbose:
        print('position transform error: %fmm, orientation transform error: %fdeg' 
              %(np.linalg.norm(pbc_ref-pbc_sample), np.rad2deg(theta)))
        print('marker position error:')
        print(np.linalg.norm(pb-pb_sample,axis=0))
        print('mean error: %fmm, maximum error: %fmm' %(mean_err, max_err))
        
    return mean_err,max_err


if __name__ == '__main__':
    pbc_ref = np.array([80,309,-69]).reshape(3,1)
    rbc_ref = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    rbc_ref = smb.angvec2r(5,norm_rand_vector(3),unit='deg')@rbc_ref
    # pb = np.array([[158,0,0],[0,0,0],[0,0,26],[164,-11,24]]).T
    # pb = np.array([[158,-10,10],[0,0,0],[10,-10,26],[164,-11,24]]).T
    pb = np.array([[158,0,0],[0,0,0],[0,0,26],[164,-11,24],[100,5,-5],[60,5,-5]]).T

    num_test = 1000
    mean_pos_err = []
    max_pos_err = []
    for idx in range(num_test):
        mean_err,max_err = analysis_transform_error(pb,pbc_ref,rbc_ref,False)
        mean_pos_err.append(mean_err)
        max_pos_err.append(max_err)

    print('mean position error: %fmm' %(np.mean(mean_pos_err)))
    print('maximum position error: %fmm' %(np.max(max_pos_err)))