from PyCRC.CRC16 import CRC16
from PyCRC.CRC16DNP import CRC16DNP
from PyCRC.CRC16Kermit import CRC16Kermit
from PyCRC.CRC16SICK import CRC16SICK
from PyCRC.CRC32 import CRC32
from PyCRC.CRCCCITT import CRCCCITT
import crcmod
import crc
import libscrc
import binascii

val = '0x000343c60002'
# print(CRC16().calculate(val))
# print(CRC16DNP().calculate(val))
# print(CRC16Kermit().calculate(val))
# print(CRC16SICK().calculate(val))
# print(CRCCCITT().calculate(val))
# print(CRC32().calculate(val))

print((-10).to_bytes(2,'big',signed=True).hex(' '))
data = b'\x00\x03\x46\x5d\x00\x02'
# print(type(data))
print(data.hex(' '))
config = crc.Configuration(width=16,polynomial=0x8005,
        init_value=0xffff,final_xor_value=0x0000,
        reverse_input=True,reverse_output=True)
crc_cal = crc.Calculator(configuration=config)
# print(hex(crc_cal.checksum(data)))

crc16 = crcmod.mkCrcFun(0x18005,rev=True,initCrc=0xffff,xorOut=0x0000)
# print(hex(crc16(data)))

crc_val = libscrc.modbus(data)
print(hex(crc_val))
cc = crc_val.to_bytes(2,'little')
print(cc.hex(' '))
msg = data+cc

print(msg.hex(' '))
print(data[0:5])
