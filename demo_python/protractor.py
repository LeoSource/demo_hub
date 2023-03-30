# !/bin/python3
import time
import re
import serial
import serial.tools.list_ports

def find_protractor(bps,max_failed=10):
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list)==0:
        raise Exception("无可用串口")
    else:
        timex = 0.1
        protractor_port = None
        for port in port_list:
            sensor = serial.Serial(port.name,bps,timeout=timex)
            time.sleep(0.5)
            if sensor.in_waiting:
                res_str = sensor.readline().decode()
                if ":" in res_str:
                    if ("DUAL" in res_str) or ("SING" in res_str) or ("GYRO" in res_str):
                        protractor_port = port
            sensor.close()
    if protractor_port is None:
        raise Exception("未找到角度测量仪")
    return protractor_port.name

def extract_numbers(input_string):
    return re.findall(r'[-+]?\d*\.\d+|\d+', input_string)

class protractor(object):
    def __init__(self,bps=9600) -> None:
        try:
            protractor_port = find_protractor(bps=9600)
            self.ser = serial.Serial(protractor_port,bps,timeout=0.2)
            print('波特率：',self.ser.baudrate)
            print('校验位：',self.ser.parity)
            print('停止位：',self.ser.stopbits)
            print('读超时设置：',self.ser.timeout)
            print('软件流控：',self.ser.xonxoff)
            print('硬件流控：',self.ser.rtscts)
            print('硬件流控：',self.ser.dsrdtr)
            print('字符间隔超时：',self.ser.interCharTimeout)
            if not self.ser.is_open:
                self.ser.open()
        except Exception as e:
            print("---error---: ",e)

    def read_sensor_value(self):
        res_value = None
        while res_value is None:
            self.ser.reset_input_buffer()
            rec_data = self.ser.readline()
            res_str = rec_data.decode()
            # print(res_str)
            res_list = extract_numbers(res_str)
            if "DUAL" in res_str:
                res_value = res_list[0:2]
                # print("X轴角度: ",res_list[0])
                # print("Y轴角度: ",res_list[1])
            elif "SING" in res_str:
                if "mm/M" in res_str:
                    res_value = res_list[0]
                    # print("单轴mm/M: ",res_list)
                elif "%" in res_str:
                    res_value = res_list[0]
                    # print("单轴%: ",res_list)
                else:
                    res_value = res_list[0]
                    # print("轴角度: ",res_list)
            elif "GYRO" in res_str:
                res_value = res_list[0]
                # print("GYRO: ",res_list)
        return res_value

    def run(self):
        while True:
            angle = self.read_sensor_value()
            print(angle)
            time.sleep(1)


if __name__=='__main__':
    ptr = protractor()
    ptr.run()




