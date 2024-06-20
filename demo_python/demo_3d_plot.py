# -*- coding: utf-8 -*-
# @File   		: demo_3d_plot.py
# @Description	: plot 3d lines/points
# @Date   		: 2024/06/20 13:26:41
# @Author	    : zxliao, zhixiangleo@163.com

import matplotlib.pyplot as plt
import numpy as np

def axis_equal_matlab(ax):
    x_range = ax.get_xlim()[1]-ax.get_xlim()[0]
    y_range = ax.get_ylim()[1]-ax.get_ylim()[0]
    z_range = ax.get_zlim()[1]-ax.get_zlim()[0]
    ax.set_box_aspect([x_range,y_range,z_range])

def python_style(xdata,ydata,zdata):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d',proj_type='ortho')
    ax.plot(xdata, ydata, zdata, label='parametric curve')
    ax.legend()
    ax.set_box_aspect([1,1,1])
    ax.axis('equal')
    ax.set(xlabel='X',ylabel='Y',zlabel='Z')
    ax.set_title('python stytle')

def matlab_style(xdata,ydata,zdata):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d',proj_type='ortho')
    ax.plot(xdata, ydata, zdata, label='parametric curve')
    ax.legend()
    axis_equal_matlab(ax)
    ax.set(xlabel='X',ylabel='Y',zlabel='Z')
    ax.set_title('matlab stytle')


if __name__=='__main__':
    theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    z = np.linspace(-2, 2, 100)
    r = z**2 + 5
    x = r * np.sin(theta)
    y = r * np.cos(theta)
    python_style(x,y,z)
    matlab_style(x,y,z)
    plt.show()
