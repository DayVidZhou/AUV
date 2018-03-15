import smbus
import struct
import time
from mpu6050 import mpu6050
import queue
imu6050 = mpu6050(0x68)

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

ARDUINO_ADDR = 0x04

# read only registers
REG_R_ALL = 0x10 # reads everything at once
ARDUINO_PACKET_SIZE = 8

# write only registers
REG_REF_SURGE = 0x03
REG_REF_DEPTH = 0x04
REG_REF_HEADING = 0x05 # Desired heading from LOS path following controller
REG_GX = 0x06
REG_GY = 0x07
REG_GZ = 0x08
REG_AX = 0x09
REG_AY = 0x0A
REG_AZ = 0x0B
REG_X = 0x0C
REG_Y = 0x0D
REG_Z = 0x0E
REG_IMU_ALL = 0x0F

def get_imu_data():
    data = imu6050.get_all_data()
    return {'ax' : data[0]['x'], 'ay':data[0]['y'], 'az':data[0]['z'], 'gx':data[1]['x'], 'gy':data[1]['y'], 'gz':data[1]['z']}
    
def main():
    imu_fifo = queue.Queue() # {'ax','ay','az','gx','gy','gz'}
    depth_buf = queue.Queue()

    imu_fifo.put(get_imu_data())
    motion = imu_fifo.get()
    float2bytes = struct.pack('=6f', motion['ax'], motion['ay'], motion['az'], motion['gx'], motion['gy'], motion['gz'])
    bus.write_block_data(ARDUINO_ADDR, REG_IMU_ALL, list(float2bytes))

    arduino_packet = bus.read_i2c_block_data(ARDUINO_ADDR, REG_R_ALL, ARDUINO_PACKET_SIZE)
    arduino_packet_unpacked = struct.unpack('=hhf',bytes(arduino_packet))
    print(arduino_packet_unpacked)

main()
