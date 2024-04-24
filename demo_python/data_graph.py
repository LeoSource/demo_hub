import time
from threading import Thread
import psutil as p
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import math


class DataGraph(object):
    def __init__(self) -> None:
        self.t0 = time.time()
        self.value = None
        self.time_max = 10


    def read_value(self):
        t = time.time()-self.t0
        f = 0.2
        self.value = math.sin(2*math.pi*f*t)

    def read_forever(self):
        while True:
            self.read_value()
            time.sleep(0.05)

    def on_timer(self,ax):
        self.data_list = self.data_list[1:]+[self.value]
        self.time = self.time[1:]+[time.time()-self.t0]
        # if self.time[-1]>self.time_max:
        #     for idx in range(len(self.time)):
        #         if self.time[idx] is not None:
        #             self.time[idx] = self.time[idx]-(self.time[-1]-self.time_max)
        self.line_data.set_xdata(self.time)
        self.line_data.set_ydata(self.data_list)
        ax.set_xlim([min(self.time),max(self.time)])
        ax.draw_artist(self.line_data)
        ax.figure.canvas.draw()
        # ax.plot(self.time,self.data_list)
        # print(self.time[-1])


    def plot(self):
        thread_read = Thread(target=self.read_forever)
        thread_read.daemon = True
        thread_read.start()

        NUM_POINTS = 300
        fig,ax = plt.subplots()
        ax.set_ylim([-1, 1])
        ax.set_xlim([0, self.time_max])
        ax.set_autoscale_on(False)
        # self.ax.set_xticks([])
        # self.ax.set_yticks(range(0, 101, 10))
        ax.grid(True)
        self.data_list = [None]*NUM_POINTS
        self.time = [0]*NUM_POINTS
        self.line_data, = ax.plot(self.time,self.data_list)
        timer = fig.canvas.new_timer(interval=50)
        timer.add_callback(self.on_timer,ax)
        timer.start()
        plt.show()

def prepare_cpu_usage():
    t = p.cpu_times()
    if hasattr(t, 'nice'):
        return [t.user, t.nice, t.system, t.idle]
    else: 
        return [t.user, 0, t.system, t.idle]

def get_cpu_usage():
    global before

    now = prepare_cpu_usage()
    delta = [now[i] - before[i] for i in range(len(now))]
    total = sum(delta)
    before = now
    return [(100.0*dt)/(total+0.1) for dt in delta]

def OnTimer(ax):
    global user,nice,sys,idle,bg

    tmp = get_cpu_usage()

    user = user[1:] + [tmp[0]]
    nice = nice[1:] + [tmp[1]]
    sys = sys[1:] + [tmp[2]]
    idle = idle[1:] + [tmp[3]]
    # print(id(l_user))

    l_user.set_ydata(user)
    l_nice.set_ydata(nice)
    l_sys.set_ydata(sys)
    l_idle.set_ydata(idle)

    ax.draw_artist(l_user)
    ax.draw_artist(l_nice)
    ax.draw_artist(l_sys)
    ax.draw_artist(l_idle)

    ax.figure.canvas.draw()


if __name__=='__main__':
    graph = DataGraph()
    graph.plot()

    # data = [1,2,3,4]
    # for idx in range(len(data)):
    #     data[idx] -= 1
    # print(data)

    # POINTS = 300

    # fig,ax = plt.subplots()

    # ax.set_ylim([0, 100])
    # ax.set_xlim([0, POINTS])
    # ax.set_autoscale_on(False)

    # ax.set_xticks([])
    # ax.set_yticks(range(0, 101, 10))
    # ax.grid(True)

    # user = [None] * POINTS
    # nice = [None] * POINTS
    # sys = [None] * POINTS
    # idle = [None] * POINTS

    # l_user, = ax.plot(range(POINTS), user, label = 'User %')
    # l_nice, = ax.plot(range(POINTS), nice, label = 'Nice %')
    # l_sys, = ax.plot(range(POINTS), sys, label = 'Sys %')
    # l_idle, = ax.plot(range(POINTS), idle, label = 'Idle %')
    # print(id(l_user))
    # print('----')

    # ax.legend(loc = 'upper center',
    #         ncol = 4, prop = font_manager.FontProperties(size = 10))

    # # bg = fig.canvas.copy_from_bbox(ax.bbox)

    # before = prepare_cpu_usage()

    # timer = fig.canvas.new_timer(interval=100)
    # timer.add_callback(OnTimer,ax)
    # timer.start()
    # plt.show()