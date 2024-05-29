# !/bin/python3
import time
import serial.tools.list_ports
import serial
import matplotlib.pyplot as plt
from threading import Thread
from robot_mqtt_client import RobotMQTTClient
from dynamic_graph import DynamicGraph
import json


POINTS = 200

def find_lasar(bps,max_failed=10):
    num_failed = 0
    lasar_port = None
    while num_failed<max_failed:
        port_list = list(serial.tools.list_ports.comports())
        if len(port_list)==0:
            num_failed += 1
            time.sleep(0.2)
            continue
        else:
            for port in port_list:
                if port.manufacturer=='FTDI':
                    lasar_port = port
                    num_failed = 0
                    break
            if lasar_port is None:
                num_failed += 1
                time.sleep(0.2)
                continue
            else:
                break
    if lasar_port is None:
        raise Exception("未找到激光位移传感器")
    return lasar_port.name

def in_range(value,range)->bool:
    if value>=range[0] and value<=range[1]:
        return True
    else:
        return False

class LaserRangefinder(object):
    def __init__(self,com:str,bps=9600) -> None:
        try:
            lasar_port = find_lasar(bps)
            self.ser = serial.Serial(com,bps,timeout=0.05)
            print(f'laser rangefinder {com} is opened')
        except Exception as e:
            print("---error---: ",e)
            raise Exception(e)
        self.stx = b'\x02'
        self.etx = b'\x03'
        self.ack = b'\x06'
        self.nak = b'\x15'
        self.read_vaule_address = b'\x43'
        self.write_setting_address = b'\x57'
        self.read_setting_address = b'\x52'
        self.len_bytes = 6

        self.sensor_value = None
        self.t0 = time.time()
        self.time_max = 10
        self.plt_data = [None]*POINTS
        self.time = [None]*POINTS

    def get_bcc(self,dat1:bytes,dat2:bytes,dat3:bytes)->bytes:
        res = int.from_bytes(dat1,'big') ^ int.from_bytes(dat2,'big') ^ int.from_bytes(dat3,'big')
        return res.to_bytes(1,'big')

    def read_data(self,cmd:bytes,data1:bytes,data2:bytes):
        pass

    def write_data(self,cmd:bytes,data1:bytes,data2:bytes):
        bcc = self.get_bcc(cmd,data1,data2)
        res = self.stx+cmd+data1+data2+self.etx+bcc
        self.ser.write(res)#02 43 B0 01 03 F2

    def read_sensor_data(self)->bytes:
        self.write_data(self.read_vaule_address,b'\xb0',b'\x01')
        while self.ser.in_waiting!=self.len_bytes:
            continue
        rec_data = self.ser.read(self.len_bytes)
        return rec_data

    def read_sensor_value(self)->float:
        # value_validation = False
        # while not value_validation:
        rec_data = self.read_sensor_data()
        res = int.from_bytes(rec_data[2:4],'big')
        if res>32768:
            res = res-65536
        disp = 10
        res = res*disp/1000.0
        res += 100
        if res>50 and res<150:
            value_validation = True
        value_validation = True
        
        return res

    def run(self):
        while True:
            self.sensor_value = self.read_sensor_value()
            t = time.strftime('%H%M%S',time.localtime())
            # print(t,res)
            time.sleep(0.005)

    def record_plt_data(self):
        while True:
            ret = self.read_sensor_value()
            t = time.time()-self.t0
            self.time = self.time[1:]+[t]
            self.plt_data = self.plt_data[1:]+[ret]
            time.sleep(0.01)

    def plot(self):
        graph = DynamicGraph()
        graph.add_plt_data(5,self.read_sensor_value,'laser')
        graph.plot()

    def respiratory_control(self):
        self.mqtt_client = RobotMQTTClient('192.168.2.242')
        self.mqtt_client.add_callabck_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

        thread_read = Thread(target=self.run,daemon=True)
        thread_read.start()

        pub_data = {"name":"respiratory","motion":False}
        # range = [30970,31160]
        range = [0,77.6715]
        # range = [94.2585,100]
        while self.sensor_value is None:
            time.sleep(0.01)
        self.sensor_value_last = self.sensor_value
        while True:
            if in_range(self.sensor_value,range) and not in_range(self.sensor_value_last,range):
                pub_data['motion'] = True
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            if in_range(self.sensor_value_last,range) and not in_range(self.sensor_value,range):
                pub_data['motion'] = False
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
            self.sensor_value_last = self.sensor_value
            time.sleep(0.005)

    def on_topic_robot_info(self,client,userdata,msg):
        pass


if __name__=='__main__':
    lr = LaserRangefinder(com='COM4',bps=9600)
    # lr.run()
    lr.plot()
    # lr.respiratory_control()