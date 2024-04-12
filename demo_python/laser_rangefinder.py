# !/bin/python3
import time
import serial.tools.list_ports
import serial


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

class laser_rangefinder(object):
    def __init__(self,bps=9600) -> None:
        try:
            lasar_port = find_lasar(bps)
            self.ser = serial.Serial(lasar_port,bps,timeout=0.1)
            print('波特率：',self.ser.baudrate)
            print('校验位：',self.ser.parity)
            print('停止位：',self.ser.stopbits)
            print('读超时设置：',self.ser.timeout)
            print('软件流控：',self.ser.xonxoff)
            print('硬件流控：',self.ser.rtscts)
            print('硬件流控：',self.ser.dsrdtr)
            print('字符间隔超时：',self.ser.interCharTimeout)
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

    def get_bcc(self,dat1:bytes,dat2:bytes,dat3:bytes)->bytes:
        res = int.from_bytes(dat1,'big') ^ int.from_bytes(dat2,'big') ^ int.from_bytes(dat3,'big')
        return res.to_bytes(1,'big')

    def read_data(self,cmd:bytes,data1:bytes,data2:bytes):
        pass

    def write_data(self,cmd:bytes,data1:bytes,data2:bytes):
        bcc = self.get_bcc(cmd,data1,data2)
        res = self.stx+cmd+data1+data2+self.etx+bcc
        self.ser.write(res)

    def read_sensor_value(self):
        self.write_data(self.read_vaule_address,b'\xb0',b'\x01')
        count = 0
        while not self.ser.in_waiting:
            time.sleep(0.001)
            count = count+1
        rec_data = self.ser.readline()
        res = int.from_bytes(rec_data[2:4],'big')
        if res>32768:
            res = res-65536
        disp = 10
        res = res*disp/1000.0
        
        return res+100

    def run(self):
        while True:
            res = self.read_sensor_value()
            print(res)
            time.sleep(1)


if __name__=='__main__':
    lr = laser_rangefinder(bps=9600)
    lr.run()