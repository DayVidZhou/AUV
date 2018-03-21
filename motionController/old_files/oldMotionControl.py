import smbus
import _thread
import struct
import time
from mpu6050 import mpu6050
import queue
import curses


ARDUINO_ADDR = 0x04
# read only registers
REG_R_ALL = 0x03 # reads everything at once
ARDUINO_PACKET_SIZE = 8 # Read size
# write only registers
REG_IMU_ALL = 0x00 # Send all IMU data
REG_REF_TRAG = 0x01 # Desired tragectory
REG_USER_CMD = 0x02 # Send manual user command
REG_CHANGE_MODE = 0x04

IDLE = 0x00
FWD_SURGE = 0x01
RWD_SURGE = 0x02
UP_HEAVE = 0x03
DWN_HEAVE = 0x04
YAW_LEFT = 0x05
YAW_RIGHT = 0x06
SHUT_DWN = 0xff
SCREEN_X = 70
SCREEN_Y = 20
cmd = IDLE
imu6050 = mpu6050(0x68)
bus = smbus.SMBus(1) # for RPI version 1, use "bus = smbus.SMBus(0)"
user_cmd_fifo = queue.Queue()

def get_imu_data():
    data = imu6050.get_all_data()
    return {'ax' : data[0]['x'], 'ay':data[0]['y'], 'az':data[0]['z'], 'gx':data[1]['x'], 'gy':data[1]['y'], 'gz':data[1]['z']}
    
#class IO(threading.Thread):
def run():
    imu_fifo = queue.Queue() # {'ax','ay','az','gx','gy','gz'}
    depth_buf = queue.Queue()
    cmd_dir = IDLE
    cmd_pwr = 100
    last_cmd = IDLE
    while True:
        #cmd_arr[0] = cmd
        cmd_dir = user_cmd_fifo.get(timeout=1)
        bus.write_word_data(ARDUINO_ADDR, REG_USER_CMD, (cmd_dir<<8)|cmd_pwr)
        #imu_fifo.put(get_imu_data())
        #motion = imu_fifo.get()
        #print (cmd_dir)
        #float2bytes = struct.pack('=6f', motion['ax'], motion['ay'], motion['az'], motion['gx'], motion['gy'], motion['gz'])
        #bus.write_block_data(ARDUINO_ADDR, REG_IMU_ALL, list(float2bytes))
        #short2bytes = struct.pack('=2h', cmd_arr[0],cmd_arr[1])
        #print(short2bytes)
        #arduino_packet = bus.read_i2c_block_data(ARDUINO_ADDR, REG_R_ALL, ARDUINO_PACKET_SIZE)
        #arduino_packet_unpacked = struct.unpack('=hhf',bytes(arduino_packet))
        #print(arduino_packet_unpacked)
        #time.sleep(0.5)


def input_handler(stdscr):
    stdscr.clear()
    stdscr.refresh()
    stdscr.addstr(SCREEN_Y,SCREEN_X,"AUTONOMOUS MODE...", curses.A_BLINK)
    first_input = stdscr.getch()
    curses.halfdelay(10)
    while True:
        k = stdscr.getch()
        if (k == curses.KEY_UP):
            cmd = FWD_SURGE
            stdscr.clear()
            stdscr.addstr(20,70,"SURGE FWD")
        elif (k == curses.KEY_DOWN):
            cmd = RWD_SURGE
            stdscr.clear()
            stdscr.addstr(20,70,"SURGE RWD")
        elif (k == curses.KEY_LEFT):
            cmd = YAW_LEFT
            stdscr.clear()
            stdscr.addstr(20,70,"YAW LEFT")
        elif (k == curses.KEY_RIGHT):
            cmd = YAW_RIGHT
            stdscr.clear()
            stdscr.addstr(20,70,"YAW RIGHT")
        elif (k == ord('w')):
            cmd = UP_HEAVE
            stdscr.clear()
            stdscr.addstr(20,70,"HEAVE UP")
        elif (k == ord('d')):
            cmd = DWN_HEAVE
            stdscr.clear()
            stdscr.addstr(20,70,"HEAVE DWN")
        elif (k == curses.ERR):
            cmd = IDLE
            stdscr.clear()
            #stdscr.addstr(20,70,"IDLE")
        elif(k == ord('q')):
            cmd = SHUT_DWN
            break;
        user_cmd_fifo.put(cmd)
        

if __name__ == '__main__':
    _thread.start_new_thread(run, ())
    curses.wrapper(input_handler)

