#from smbus2 import SMBus
import smbus
import _thread
import subprocess
#import _thread
#from threading import Thread
import struct
import time
#from mpu6050 import mpu6050
import queue # change this
import curses
import sys

sys.path.append('/home/pi/MTE380/MPU6050_I2C_Python_Class')
from MPU6050 import MPU6050
from Quaternion import XYZVector as Vec

"""

1. IMU sample rate is 200Hz so with 1024 FIFO data must be read from IMU
    at least once every 366 ms
2. Recalibrate the IMU before live tests and reinitalize respective offsets

"""

ARDUINO_ADDR = 0x04

# read only registers
REG_R_ALL = 0x03 # reads everything at once
ARDUINO_PACKET_SIZE = 12  # Read size

# write only registers
REG_IMU_ALL = 0x00 # Send all IMU data
REG_REF_TRAG = 0x01 # Desired tragectory
REG_USER_CMD = 0x02 # Send manual user command
REG_CHANGE_MODE = 0x04

# User Commands
IDLE = 0x00
FWD_SURGE = 0x01
RWD_SURGE = 0x02
UP_HEAVE = 0x03
DWN_HEAVE = 0x04
YAW_LEFT = 0x05
YAW_RIGHT = 0x06
CHANGE_YAW_PID = 0x07
CHANGE_HEAVE_PID = 0x08
SHUT_DWN = 0xff
SCREEN_X = 70
SCREEN_Y = 20

i2c_bus = 1
device_address = 0x68
x_accel_offset = -2490 # -5489
y_accel_offset = 3628 #-1441
z_accel_offset = -534 #1305
x_gyro_offset = 51 #-2
y_gyro_offset =203 #-72
z_gyro_offset = 68 #-5
enable_debug_output = False

user_mode = True
#imu6050 = mpu6050(0x68)
bus = smbus.SMBus(1) # for RPI version 1, use "bus = smbus.SMBus(0)"
user_cmd_fifo = queue.Queue()
imu_fifo = queue.Queue() # {'ax','ay','az','gz'}
pid_tune_fifo = queue.Queue()
user_print_fifo = queue.Queue()

def get_imu_data():
    data = imu6050.get_all_data()
    return {'ax' : data[0]['x'], 'ay':data[0]['y'], 'az':data[0]['z'], 'gx':data[1]['x'], 'gy':data[1]['y'], 'gz':data[1]['z']}
    
def io_thread():
    depth_buf = queue.Queue()
    cmd_dir = IDLE
    cmd_pwr = 100
    count = 0
    motion = None
    param_change = False
    f = open('tuning.txt', 'w')
    if (user_mode == True):
        try:
            bus.write_word_data(ARDUINO_ADDR, REG_USER_CMD, (cmd_dir<<8)|cmd_pwr)
        except IOError as ioe:
            subprocess.call(['i2cdetect', '-y', '1'])
            print( ioe)

    while True:
        if (user_mode == True):
            try:
                cmd_dir = user_cmd_fifo.get(timeout=0.1)
            except queue.Empty as e:
                print( e)
                cmd_dir = IDLE
            try:
                chksum = -6 # cmd + size + data
                short2bytes = struct.pack('=hh', cmd_dir,cmd_pwr)
                bus.write_block_data(ARDUINO_ADDR, REG_USER_CMD, list(short2bytes))
                time.sleep(0.1)

            except IOError as e:
                print (e)
                time.sleep(1)
                subprocess.call(['i2cdetect', '-y', '1'])
        try:    
            arduino_packet = bus.read_i2c_block_data(ARDUINO_ADDR, REG_R_ALL, ARDUINO_PACKET_SIZE)
            arduino_data = struct.unpack('=fff',bytes(arduino_packet))
            f.write('%s %s %s\n' % (str(arduino_data[0]),str(arduino_data[1]),str(arduino_data[2])))
            #print('depth ' + str(arduino_data[2]))
            #print('yaw_pid_out ' + str(arduino_data[0]))
            #print('heave_pid_out ' + str(arduino_data[1]))
        except IOError as e:
            print(e)
            time.sleep(1)
            subprocess.call(['i2cdetect', '-y', '1'])
        time.sleep(0.1)
        
        
        if (param_change == True):
            try:
                if (user_mode == False):
                    Kp = int(input('Yaw Kp: '))
                    Kd = int(input('Yaw Kd: '))
                    Ki = int(input('Yaw Ki: '))
                    st = int(input('Yaw st: '))
                else:
                    Kp = 0
                    Kd = 0
                    Ki = 0
                    st = 0
                float2bytes = struct.pack('=4f', Kp, Kd, Ki, st)
                bus.write_block_data(ARDUINO_ADDR, CHANGE_YAW_PID, list(float2bytes))

            except IOError as e:
                time.sleep(1)
                subprocess.call(['i2cdetect', '-y', '1'])
            time.sleep(0.1)

            try:
                if (user_mode == False):
                    Kp = int(input('Heave Kp: '))
                    Kd = int(input('Heave Kd: '))
                    Ki = int(input('Heave Ki: '))
                    st = int(input('Heave st: '))
                else:
                    st = 0
                    Kp = 0
                    Kd = 0
                    Ki = 0
                float2bytes = struct.pack('=4f', Kp, Kd, Ki, st)
                bus.write_block_data(ARDUINO_ADDR, CHANGE_HEAVE_PID, list(float2bytes))

            except IOError as e:
                time.sleep(1)
                subprocess.call(['i2cdetect', '-y', '1'])
            time.sleep(0.1)
        
        try:
            if (user_mode == False):
                xd = int(input('xd: '))
                yd = int(input('yd: '))
                zd = int(input('zd: '))
            else:
                xd = 1.0
                yd = 0.2
                zd = 1.0
            float2bytes = struct.pack('=3f', xd, yd, zd)
            bus.write_block_data(ARDUINO_ADDR, REG_REF_TRAG , list(float2bytes))
            time.sleep(0.1)
        except IOError as e:
            time.sleep(1)
            subprocess.call(['i2cdetect', '-y', '1'])
        
        try:
            motion = imu_fifo.get(timeout=0.5)
        except queue.Empty as e:
            continue
        try:
            float2bytes = struct.pack('=3f', motion['ax'], motion['az'], motion['yaw'])
            bus.write_block_data(ARDUINO_ADDR, REG_IMU_ALL, list(float2bytes))
            time.sleep(0.1)
        except IOError as e:
            time.sleep(1)
            subprocess.call(['i2cdetect', '-y', '1'])

def input_handler(stdscr):
    cmd = IDLE
    last_cmd = -1
    stdscr.clear()
    stdscr.refresh()
    stdscr.addstr(SCREEN_Y,SCREEN_X,"AUTONOMOUS MODE...", curses.A_BLINK)
    first_input = stdscr.getch()
    #curses.halfdelay(5)
    while True:
        k = stdscr.getch()
        if (k == curses.KEY_UP):
            cmd = FWD_SURGE
        #    stdscr.clear()
         #   stdscr.addstr(20,70,"SURGE FWD")
        elif (k == curses.KEY_DOWN):
            cmd = RWD_SURGE
          #  stdscr.clear()
          #  stdscr.addstr(20,70,"SURGE RWD")
        elif (k == curses.KEY_LEFT):
            cmd = YAW_LEFT
          #  stdscr.clear()
          #  stdscr.addstr(20,70,"YAW LEFT")
        elif (k == curses.KEY_RIGHT):
            cmd = YAW_RIGHT
          #  stdscr.clear()
          #  stdscr.addstr(20,70,"YAW RIGHT")
        elif (k == ord('w')):
            cmd = UP_HEAVE
          #  stdscr.clear()
          #  stdscr.addstr(20,70,"HEAVE UP")
        elif (k == ord('d')):
            cmd = DWN_HEAVE
           # stdscr.clear()
           # stdscr.addstr(20,70,"HEAVE DWN")
        elif (k == curses.ERR):
            cmd = IDLE
           # stdscr.clear()
            stdscr.addstr(20,70,"IDLE")
        elif(k == ord('q')):
            cmd = SHUT_DWN
            break;
        user_cmd_fifo.put(cmd)
        
def imu_thread():
    DEBUG_MODE = False
    mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))

    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)
    
    while True:
       
        FIFO_count = mpu.get_FIFO_count()
        mpu_int_status = mpu.get_int_status()
        if (FIFO_count == 1024) or (mpu_int_status & 0x10): # Check if overflowed
            mpu.reset_FIFO()

        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < packet_size:
                FIFO_count = mpu.get_FIFO_count()
            FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
            quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
            grav = mpu.DMP_get_gravity(quat)
            rpy = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
            a_raw = mpu.get_acceleration()
            imu_fifo.put({'ax':9.80665*(a_raw[0]/16384.0), 'ay' : 9.80665*(a_raw[1]/16384.0), 'az' : 9.80665*(a_raw[2]/16384.0), 'yaw': rpy.z })
            
            if DEBUG_MODE == True:
                print('ax: ' + str( 9.80665*(a_raw[0]/16384.0)))
                print('az: ' + str( 9.80665*(a_raw[2]/16384.0)))
                print('yaw: ' + str( rpy.z) + '\n\n')
        
  
if __name__ == '__main__':
    _thread.start_new_thread(imu_thread,())
    if (user_mode == True):
        _thread.start_new_thread(io_thread, ())
        curses.wrapper(input_handler)
    else:
        io_thread()
