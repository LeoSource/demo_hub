import pyads
from collections import OrderedDict
import ctypes
import time


struct_def = (('ControlWord',pyads.PLCTYPE_UINT,1),
                ('ControlMode',pyads.PLCTYPE_INT,1),
                ('MotorVel_cmd',pyads.PLCTYPE_DINT,1),
                ('MotorCurrent_cmd',pyads.PLCTYPE_DINT,1),
                ('MotorPos_cmd',pyads.PLCTYPE_DINT,1),
                ('MotorVel_ffd',pyads.PLCTYPE_DINT,1))

plc = pyads.Connection('192.168.60.1.1.1',pyads.PORT_TC3PLC1)



def connect_with_name(plc):
    print('read and write variables by name:')
    # 1. read and write by name with basic datatypes
    # print(plc.read_by_name('MAIN.initial'))
    # print(plc.read_by_name('MAIN.servo_rx_data[0].MotorPos_fdb',pyads.PLCTYPE_DINT))
    # plc.write_by_name('MAIN.servo_tx_data[0].ControlWord',7)

    # 2. read and write by name with arrays
    print(plc.read_by_name('MAIN.sample_array',pyads.PLCTYPE_REAL*4))
    plc.write_by_name('MAIN.sample_array',[-1,2,3,4],pyads.PLCTYPE_REAL*4)
    sample_array = plc.read_by_name('MAIN.sample_array',pyads.PLCTYPE_REAL*4)
    print(sample_array)
    print(sample_array[2])

    # 3. read and write by name with structures
    # servo_tx_data = OrderedDict([('ControlWord',6),('ControlMode',3),('MotorVel_cmd',0),
    #     ('MotorCurrent_cmd',0),('MotorPos_cmd',0),('MotorVel_ffd',0)])
    # plc.write_structure_by_name('MAIN.servo_tx_data[2]',servo_tx_data,struct_def)
    # td = plc.read_structure_by_name('MAIN.servo_tx_data[2]',struct_def)
    # print(td)
    # print(td["ControlWord"])

    # 4. read and write multiple variables with one command
    print('read and write multiple variables with one command:')
    var_list = ['MAIN.servo_tx_data[0].ControlWord','MAIN.servo_tx_data[1].ControlWord',
        'MAIN.servo_tx_data[2].ControlWord','MAIN.servo_tx_data[3].ControlWord']
    print(plc.read_list_by_name(var_list))
    write_dict = {'MAIN.servo_tx_data[0].ControlWord': 7, 'MAIN.servo_tx_data[1].ControlWord': 7,
        'MAIN.servo_tx_data[2].ControlWord': 7, 'MAIN.servo_tx_data[3].ControlWord': 7}
    plc.write_list_by_name(write_dict)

# read and write by handle
def connect_with_handle(plc):
    print('read and write variables by handle:')

    arr_handle = plc.get_handle('MAIN.sample_array')
    plc.write_by_name('',[5,6,7,8],pyads.PLCTYPE_REAL*4,handle=arr_handle)
    sample_array = plc.read_by_name('',pyads.PLCTYPE_REAL*4,handle=arr_handle)
    plc.release_handle(arr_handle)
    print(sample_array)

# read and write basic types by device notification
@plc.notification(pyads.PLCTYPE_REAL)
def call_back_noti_basic(handle,name,timestamp,value):
    print('{1}: received new notification for variable "{0}", value: {2}'
        .format(name,timestamp,value))

def connect_with_notification_basic(plc):
    attr = pyads.NotificationAttrib(ctypes.sizeof(pyads.PLCTYPE_REAL),
        trans_mode=pyads.ADSTRANS_SERVERCYCLE,cycle_time=1000)
    plc.add_device_notification('MAIN.sample_array[2]',attr,call_back_noti_basic)
    while True:
        time.sleep(0.01)

# read and write structure by device notification
@plc.notification(ctypes.c_ubyte*pyads.size_of_structure(struct_def))
def call_back_noti_struc(handle,name,timestamp,value):
    v = pyads.dict_from_bytes(value,struct_def)
    print('{1}:received new notification for variable "{0}",value:{2}'.
        format(name,timestamp,v['ControlWord']))

def connect_with_notification_struc(plc):
    attr = pyads.NotificationAttrib(pyads.size_of_structure(struct_def),
        trans_mode=pyads.ADSTRANS_SERVERCYCLE,cycle_time=1000)
    plc.add_device_notification('MAIN.servo_tx_data[1]',attr,call_back_noti_struc)
    while True:
        time.sleep(0.1)


def connnect_with_symbol(plc):
    # symbol = plc.get_symbol('MAIN.sample_array[0]')
    symbol = plc.get_symbol('MAIN.servo_tx_data',structure_def=struct_def,array_size=4)
    print(symbol.read())



if __name__=='__main__':
    plc.open()

    # connect_with_name(plc)
    # connect_with_handle(plc)
    # connect_with_notification_basic(plc)
    # connect_with_notification_struc(plc)
    connnect_with_symbol(plc)

    plc.close()