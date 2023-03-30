import time

import libscrc
import serial


class SerialModbus(object):
    def __init__(self,port,bps,timex) -> None:
        try:
            self.ser = serial.Serial(port,bps,timeout=timex)
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
        self.servo_port = b'\x00'
        self.read_addr = b'\x03'
        self.write_addr = b'\x10'
        self.status_fdb_addr = b'\x46\x57'
        self.status_cmd_addr = b'\x46\x51'
        self.err_fdb_addr = b'\x46\x6e'
        self.err_cmd_addr = b'\x46\x59'
        self.pos_fdb_addr = b'\x42\xfd'
        self.pos_cmd_addr = b'\x43\xc6'
        self.vel_fdb_addr = b'\x42\xd1'
        self.vel_cmd_addr = b'\x43\xd0'

    def get_read_modbus(self,addr_register:bytes,num_register:bytes):
        modbus_data = self.servo_port+self.read_addr+addr_register+num_register
        modbus_data += libscrc.modbus(modbus_data).to_bytes(2,'little')
        return modbus_data

    def get_write_modbus(self,addr_register:bytes,num_register:bytes,num_bytes:bytes,byte_data:bytes):
        modbus_data = self.servo_port+self.write_addr+addr_register+num_register+num_bytes+byte_data
        modbus_data += libscrc.modbus(modbus_data).to_bytes(2,'little')
        return modbus_data

    def read_servo_status(self):
        modbus_data = self.get_read_modbus(self.status_fdb_addr,b'\x00\x01')
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.01)
        rec_data = self.ser.readline()
        return rec_data[3:5]

    def read_servo_pos(self):
        modbus_data = self.get_read_modbus(self.pos_fdb_addr,b'\x00\x02')
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.01)
        rec_data = self.ser.readline()
        pos_high = rec_data[5:7]
        pos_low = rec_data[3:5]
        return pos_high+pos_low

    def read_servo_vel(self):
        modbus_data = self.get_read_modbus(self.vel_fdb_addr,b'\x00\x02')
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.01)
        rec_data = self.ser.readline()
        return rec_data[5:7]+rec_data[3:5]
    
    def read_servo_err(self):
        modbus_data = self.get_read_modbus(self.err_fdb_addr,b'\x00\x02')
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.01)
        rec_data = self.ser.readline()
        return rec_data[5:7]+rec_data[3:5]

    def read_servo_flag(self):
        modbus_data = self.get_read_modbus(b'\x46\x5d',b'\x00\x02')
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.1)
        rec_data = self.ser.readline()
        return rec_data[5:7]+rec_data[3:5]

    def write_servo_status(self,cmd:bytes):
        modbus_data = self.get_write_modbus(self.status_cmd_addr,b'\x00\x01',b'\x02',cmd)
        self.ser.write(modbus_data)
        while not self.ser.in_waiting:
            time.sleep(0.1)
        rec_data = self.ser.readline()
        if libscrc.modbus(rec_data[0:6]).to_bytes(2,'little')==rec_data[6:8]:
            return True
        else:
            return False

    def write_servo_pos(self,cmd):
        modbus_data = self.get_write_modbus(self.pos_cmd_addr,b'\x00\x02',b'\x04',cmd)
        self.ser.write(modbus_data)
        while self.ser.in_waiting!=8:
            time.sleep(0.1)
        rec_data = self.ser.readline()
        if libscrc.modbus(rec_data[0:6]).to_bytes(2,'little')==rec_data[6:8]:
            return True
        else:
            return False

    def write_servo_vel(self,cmd):
        modbus_data = self.get_write_modbus(self.vel_cmd_addr,b'\x00\x02',b'\x04',cmd)
        self.ser.write(modbus_data)
        while self.ser.in_waiting!=8:
            time.sleep(0.1)
        rec_data = self.ser.readline()
        if libscrc.modbus(rec_data[0:6]).to_bytes(2,'little')==rec_data[6:8]:
            return True
        else:
            return False

    def start_move(self):
        addr_register = b'\x43\xbf'
        cmd = b'\x00\x02'
        modbus_data = self.get_write_modbus(addr_register,b'\x00\x01',b'\x02',cmd)
        # print(modbus_data.hex())
        self.ser.write(modbus_data)
        while self.ser.in_waiting!=8:
            time.sleep(0.1)
        rec_data = self.ser.readline()
        # print(rec_data.hex())
        # print(libscrc.modbus(rec_data[0:6]).to_bytes(2,'little').hex())
        # if libscrc.modbus(rec_data[0:6]).to_bytes(2,'little')==rec_data[6:8]:
        if b'\x24\x78'==rec_data[6:8]:
            return True
        else:
            return True

    def write_servo_err(self):
        pass

    def write_servo_mode(self,cmd):
        pass

    def convert_vel(self,cmd:int)->bytes:
        byte_data = cmd.to_bytes(4,'big')
        return byte_data[2:4]+byte_data[0:2]

    def convert_pos(self,cmd:int)->bytes:
        byte_data = cmd.to_bytes(2,'big',signed=True)
        return b'\x00\x00'+byte_data

    def run(self):
        servo_status = self.read_servo_status()
        # print(servo_status.hex())
        if not int.from_bytes(servo_status,'big'):
            if self.write_servo_status(b'\x00\x01'):
                print('enabling servo ...')
        else:
            print('servo is enabled')

        pos_dir = 1
        enable_fail_count = 0
        while True:
            servo_status = self.read_servo_status()
            # print(servo_status.hex())
            # print(int.from_bytes(servo_status,'big'))
            if int.from_bytes(servo_status,'big'):
                enable_fail_count = 0
                servo_flag = self.read_servo_flag()
                if int.from_bytes(servo_flag,'big') & 1:
                    print('complete servo motion')
                    time.sleep(2)
                    vel_cmd = 1000
                    pos_cmd = 1*pos_dir
                    if self.write_servo_vel(self.convert_vel(vel_cmd)):
                        print('complete servo feedforward velocity setting')
                    if self.write_servo_pos(self.convert_pos(pos_cmd)):
                        print('complete servo reference position setting')
                    if self.start_move():
                        print('start to move ...')
                        print(pos_dir)
                        pos_dir *= -1
                else:
                    time.sleep(0.1)
            else:
                enable_fail_count += 1
                time.sleep(0.5)

            if enable_fail_count==10:
                print('servo is disabled!')
                self.ser.close()
                break



if __name__ == '__main__':
    modbus = SerialModbus(port='COM4',bps=9600,timex=0.1)
    modbus.run()
