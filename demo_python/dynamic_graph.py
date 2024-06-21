# -*- coding: utf-8 -*-
# @File   		: dynamic_graph.py
# @Description	: plot data curves dynamically 
# @Date   		: 2024/04/28 15:41:08
# @Author	    : zxliao, zhixiangleo@163.com

import math
import time
import threading
import matplotlib.pyplot as plt
import keyboard

NUM_POINTS = 300
TIME_MAX = 10

colors = ('b','g','r','c','m','y','k')

def gen_sin(amp:float,freq:float,t:float,phase:float)->float:
    return amp*math.sin(2*math.pi*freq*t+phase)

t = time.time()
def gen_dynamic_sin():
    ttick = time.time()-t
    return gen_sin(1,0.5,ttick,0)

def gen_dynamic_sin1():
    ttick = time.time()-t
    return gen_sin(2,1,ttick,0)

def gen_dynamic_sin2():
    ttick = time.time()-t
    return gen_sin(10,0.4,ttick,math.pi/4)

def gen_dynamic_sin3():
    ttick = time.time()-t
    return gen_sin(5,0.8,ttick,math.pi/2)

class DynamicGraph(object):
    def __init__(self) -> None:
        self.t0 = time.time()
        self.time = None
        self.value = None
        self.read_func = None
        self.label = None
        self.interval = None
        self.pause_plot = False
        keyboard.on_press_key('space',self.__pause_draw)

    def add_plt_data(self,interval:int,func,name:str):
        if self.time is None:
            self.time = [[None]*NUM_POINTS]
            self.value = [[None]*NUM_POINTS]
            self.read_func = [func]
            self.label = [name]
            self.interval = [interval]
        else:
            self.time = self.time+[[None]*NUM_POINTS]
            self.value = self.value+[[None]*NUM_POINTS]
            self.read_func.append(func)
            self.label.append(name)
            self.interval.append(interval)

    def record_plt_data(self,index:int):
        while True:
            ret = self.read_func[index]()
            t = time.time()-self.t0
            self.time[index] = self.time[index][1:] +[t]
            self.value[index] = self.value[index][1:]+[ret]
            time.sleep(self.interval[index]/1000.0)

    def plot(self):
        thread_read = []
        for idx in range(len(self.label)):
            thread_read.append(threading.Thread(target=self.record_plt_data,args=[idx],daemon=True))
            thread_read[idx].start()

        fig,ax1 = plt.subplots()
        ax1.set_xlim([0,TIME_MAX])
        ax1.set_ylim([-1,1])
        ax1.set_autoscale_on(False)
        ax1.grid(True)

        self.line = []
        axes = []
        for idx in range(len(self.label)):
            if idx==0:
                curve, = ax1.plot(self.time[0],self.value[0],colors[0],label=self.label[0])
                ax1.tick_params(axis='y', labelcolor = colors[0])
                axes.append(ax1)
            else:
                ax2 = ax1.twinx()
                ax2.spines['right'].set_position(('outward',35*(idx-1)))
                curve, = ax2.plot(self.time[idx],self.value[idx],colors[idx],label=self.label[idx])
                ax2.tick_params(axis='y',colors=colors[idx])
                axes.append(ax2)
            self.line.append(curve)

        timer = fig.canvas.new_timer(interval=80)
        timer.add_callback(self.__on_timer,axes)
        timer.start()
        fig.legend(self.label,loc='upper left')
        # fig.subplots_adjust(right=1-(len(self.label)-2)*0.15)
        fig.subplots_adjust()
        plt.show()

    def __on_timer(self,ax):
        for idx in range(len(self.label)):
            self.line[idx].set_xdata(self.time[idx])
            self.line[idx].set_ydata(self.value[idx])
            if self.pause_plot:
                continue
            if None in self.time[idx]:
                xdata = [t for t in self.time[idx] if t is not None]
                if len(xdata)==1:
                    pass
                else:
                    pass
                    # ax[idx].set_xlim([min(xdata),max(xdata)])
            else:
                ax[idx].set_xlim([min(self.time[idx]),max(self.time[idx])])
            if None in self.value[idx]:
                ydata = [y for y in self.value[idx] if y is not None]
                if len(ydata)==1:
                    pass
                else:
                    pass
                    # ax[idx].set_ylim([min(ydata),max(ydata)])
            else:
                ax[idx].set_ylim(min(self.value[idx]),max(self.value[idx]))
            ax[idx].draw_artist(self.line[idx])
        if not self.pause_plot:
            ax[0].figure.canvas.draw()

    def __pause_draw(self,x):
        self.pause_plot = not self.pause_plot


def demo():
    dg = DynamicGraph()
    dg.add_plt_data(20,gen_dynamic_sin,'sin')
    dg.add_plt_data(40,gen_dynamic_sin1,'sin1')
    dg.add_plt_data(30,gen_dynamic_sin2,'sin2')
    dg.add_plt_data(40,gen_dynamic_sin3,'sin3')
    dg.add_plt_data(40,gen_dynamic_sin3,'sin3')

    dg.plot()

if __name__=='__main__':
    demo()
