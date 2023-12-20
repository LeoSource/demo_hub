# !/bin/python3

import time
import serial
import libscrc
import csv
from threading import Thread
import keyboard

class force_sensor(object):
    def __init__(self,com,bps=9600) -> None:
        self.ser = serial.Serial(com,bps,timeout=0.1)
        print('波特率：',self.ser.baudrate)
        print('校验位：',self.ser.parity)
        print('停止位：',self.ser.stopbits)
        print('读超时设置：',self.ser.timeout)
        print('软件流控：',self.ser.xonxoff)
        print('硬件流控：',self.ser.rtscts)
        print('硬件流控：',self.ser.dsrdtr)
        print('字符间隔超时：',self.ser.interCharTimeout)
        self.force_data = []
        self.record_button = False
        keyboard.on_press_key('t',self.start_record)
        keyboard.on_press_key('p',self.stop_record)

    def read_sensor_value(self)->int:
        func = b'\x03'
        register_add = b'\x00\x0b'
        register_num = b'\x00\x04'
        send_data = b'\x01'+func+register_add+register_num
        send_data += libscrc.modbus(send_data).to_bytes(2,'little')
        # print(send_data.hex())
        self.ser.write(send_data)
        while not self.ser.in_waiting:
            time.sleep(0.01)
        rec_data = self.ser.readline()
        # print(rec_data.hex())
        # print(rec_data[3:7].hex())

        # send_data = b'\x01'+func+b'\x00\xc9'+b'\x00\x01'
        # send_data += libscrc.modbus(send_data).to_bytes(2,'little')
        # self.ser.write(send_data)
        # while not self.ser.in_waiting:
        #     sleep(0.01)
        # rec_data = self.ser.readline()
        # print(rec_data.hex())
        return [int.from_bytes(rec_data[3:7],'big',signed=True),int.from_bytes(rec_data[7:11],'big',signed=True)]
    
    def start_record(self,x):
        self.record_button = True
        print('start recording...')

    def stop_record(self,x):
        self.record_button = False
        t = time.strftime('%Y-%m%d-%H%M%S',time.localtime())
        filename = './data/force_data_'+t+'.txt'
        with open(filename,'w',newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.force_data)
            csvfile.close()
        print('create data file '+filename)
        print('complete recording!')
        self.force_data.clear()

    def record_force(self):
        pass
            # if not self.record_button:
            #     writer.writerows(self.force_data)
            # while True:
            #     if len(self.force_data)>0:
            #         writer.writerow([self.force_data[0]])
            #         self.force_data.pop(0)
            #         time.sleep(0.001)

    def run(self):
        thread_record = Thread(target=self.record_force)
        thread_record.start()
        while True:
            force = self.read_sensor_value()
            fs = force[0]/10.0
            fm = force[1]/10.0
            if self.record_button and abs(fs)<100 and abs(fm)<100:
                self.force_data.append([time.time(),fs,fm])
            time.sleep(0.001)
            # print(fs)
            # time.sleep(1)

 
if __name__=='__main__':
    fs_sensor = force_sensor('COM3')
    fs_sensor.run()


    # while True:
    #     force = fs_sensor.read_sensor_value()/10.0
    #     print(force)
    #     time.sleep(1)
    
    # print(int.from_bytes(func,'big'))
 