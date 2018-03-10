import smbus
import time
from mpu6050 import mpu6050
import Queue
imu6050 = mpu6050(0x68)

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

ARDUINO_ADDR = 0x04

# read only registers
REG_ALL = 0x01 # reads everything at once

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

def get_imu_data():
    data = imu6050.get_all_data()
    return {'ax' : data[0]['x'], 'ay':data[0]['y'], 'az':data[0]['z']:, 'gx':data[1]['x'], 'gy':data[1]['y'], 'gz':data[1]['z']}
    


def main():
    imu_buf = Queue.Queue() # {'ax','ay','az','gx','gy','gz'}
    depth_buf = Queue.Queue()
    arduino_packet = []

    imu_fifo.put(get_imu_data())
    motion_data = imu_buf.get()
    bus.write_word_data(ARDUINO_ADDR, REG_GZ, motion_data['gz'])
    bus.write_word_data(ARDUINO_ADDR, REG_GZ, motion_data['ax'])
    bus.write_word_data(ARDUINO_ADDR, REG_GZ, motion_data['ay'])

    arduino_packet = read_block_data(ARDUINO_ADDR, REG_ALL)


