# -*- coding: utf-8 -*-
# @File   		: respiratory_test.py
# @Description	: sample and compare respiratory motion data and surrogate date
# @Date   		: 2024/04/12 16:14:44
# @Author	    : zxliao, zhixiangleo@163.com

import laser_rangefinder
import serial_communication
import respiratory_force_sensor
from robot_mqtt_client import RobotMQTTClient
import time
import sys
import keyboard
import csv
import json
from threading import Thread

def in_range(value,range)->bool:
    if value>=range[0] and value<=range[1]:
        return True
    else:
        return False
    
def save_data_file(filename:str,data):
    with open(filename,'w',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)
        csvfile.close()
    print('create data file '+filename)

class RespiratoryTest(object):
    def __init__(self,laser,servo,rp_sensor,mqtt_client) -> None:
        self.laser = laser
        self.servo = servo
        self.rp_sensor = rp_sensor
        self.mqtt_client = mqtt_client
        self.distance_data = []
        self.respiratory_force = []
        self.needle_force = []
        self.respiratory_command = []
        self.record_button = False
        self.rp_force = None
        keyboard.on_press_key('t',self.start_record)
        keyboard.on_press_key('p',self.stop_record)
        keyboard.on_press_key('z',self.quit)
        self.mqtt_client.add_callabck_robot_info(self.on_topic_robot_info)
        self.mqtt_client.client.loop_start()

    def on_topic_robot_info(self,client,userdata,msg):
        pass

    def quit(self,x):
        sys.exit()

    def start_record(self,x):
        if self.record_button:
            print('cannot record new data while recording!')
        else:
            self.record_button = True
            print('start recording...')

    def stop_record(self,x):
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/respiratorymotion_data_'+t+'.txt'
        save_data_file(filename,self.distance_data)
        filename = './data/needleforce_data_'+t+'.txt'
        save_data_file(filename,self.needle_force)
        filename = './data/respiratoryforce_data_'+t+'.txt'
        save_data_file(filename,self.respiratory_force)
        filename = './data/respiratorycommand_data_'+t+'.txt'
        save_data_file(filename,self.respiratory_command)

        print('complete recording!')
        self.distance_data.clear()
        self.respiratory_force.clear()
        self.needle_force.clear()
        self.respiratory_command.clear()
        self.record_button = False
        
    def respiratory_control(self):
        pub_data = {"name":"respiratory","motion":False}
        # range = [30970,31160]
        # range = [0,30970]
        range = [31160,50000]
        while self.rp_force is None:
            time.sleep(0.2)
        self.rp_force_last = self.rp_force
        while True:
            if in_range(self.rp_force,range) and not in_range(self.rp_force_last,range):
                pub_data['motion'] = True
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
                if self.record_button:
                    self.respiratory_command.append([time.time(),1])
            if in_range(self.rp_force_last,range) and not in_range(self.rp_force,range):
                pub_data['motion'] = False
                self.mqtt_client.pub_hr_robot(json.dumps(pub_data))
                if self.record_button:
                    self.respiratory_command.append([time.time(),0])
            self.rp_force_last = self.rp_force
            time.sleep(0.005)

    def read_distance_data(self):
        while True:
            dis = self.laser.read_sensor_value()
            if self.record_button:
                self.distance_data.append([time.time(),dis])
            time.sleep(0.005)

    def read_respiratory_force(self):
        while True:
            self.rp_force = self.rp_sensor.read_sensor_value()
            if self.record_button:
                self.respiratory_force.append([time.time(),self.rp_force])
            time.sleep(0.01)

    def read_needle_force(self):
        while True:
            analog_input = self.servo.read_analog_input()
            ai_value = int.from_bytes(analog_input,'big')
            if self.record_button:
                self.needle_force.append([time.time(),ai_value])

    def run(self):
        thread_distance = Thread(target=self.read_distance_data)
        thread_distance.daemon = True
        thread_distance.start()

        thread_respiratory_force = Thread(target=self.read_respiratory_force)
        thread_respiratory_force.daemon = True
        thread_respiratory_force.start()

        thread_needle_force = Thread(target=self.read_needle_force)
        thread_needle_force.daemon = True
        thread_needle_force.start()

        thread_respiratory_control = Thread(target=self.respiratory_control)
        thread_respiratory_control.daemon = True
        thread_respiratory_control.start()

        while True:
            time.sleep(1)


if __name__=='__main__':
    laser = laser_rangefinder.laser_rangefinder(9600)
    servo = serial_communication.SerialModbus(port='COM4',bps=9600,timex=0.1)
    rp_sensor = respiratory_force_sensor.respiratory_sensor(com='COM12',bps=9600)
    mqtt_client = RobotMQTTClient('192.168.2.242')
    rp_test = RespiratoryTest(laser,servo,rp_sensor,mqtt_client)
    rp_test.run()
