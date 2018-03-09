import smbus
import time
from mpu6050 import mpu6050
imu6050 = mpu6050(0x68)

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

Arduino = 0x04
testType = 0

def writeNumber(address, value):
    bus.write_byte(address, int(value))
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

def setTest():
    T = 0
    amplitude = 0
    pulse_delay = 0
    while True:
        if bus.read_byte(Arduino) == 0:
            test = input("Enter test type 1 - 3: ")
            testType = int(test)
            if not test:
                continue
            writeNumber(Arduino, test)
        if bus.read_byte(Arduino) == 1:
            amp = input("Enter amplitude 1 - 5: ")
            amplitude = int(amp)
            if not amp:
                continue
            writeNumber(Arduino, amp)
        if bus.read_byte(Arduino) == 2:
            period = input("Enter period/2 x100: ")
            T = int(period)*100
            if not period:
                continue
            writeNumber(Arduino, period)
        if bus.read_byte(Arduino) == 3:
            delay = input("Enter delay x1000: ")
            pulse_delay = int(delay)*1000
            if not delay:
                continue
            writeNumber(Arduino, delay)
        if bus.read_byte(Arduino) == 4:
            writeNumber(Arduino, 1)
            if testType == 1:
                collectdata(T, amplitude, pulse_delay, "testyaw.txt")
            elif testType == 2:
                collectdata(T, amplitude, pulse_delay, "testheave.txt")
            elif testType == 3:
                collectdata(T, amplitude, pulse_delay, "testsurge.txt")
            else:
                print ("No good error type")
            return 0

def collectdata(period, amplitude, pulse_delay, filename):
    data = []
    curtime = time.time()
    while (time.time() - curtime)*1000 < ((period + 200 + (pulse_delay*2))*2):
        accel_data = imu6050.get_accel_data()
        gyro_data = imu6050.get_gyro_data()
        imu = [accel_data,gyro_data]
        data.append(imu)
    
    thefile = open(filename, 'w')
    thefile.write("ax,ay,az,gx,gy,gz\n")
    
    for item in data:
        thefile.write("%s,%s,%s,%s,%s,%s\n" % (str(item[0]['x']), str(item[0]['y']), str(item[0]['z']), str(item[1]['x']), str(item[1]['y']), str(item[1]['z']) ))

    print("The ARDUINO STATUS IS ")
    print(bus.read_byte(Arduino))

setTest()

    # var = input("Enter 1 - 9: ")
    # if not var:
    #     continue

    # writeNumber(var)
    # print "RPI: Hi Arduino, I sent you ", var
    # # sleep one second
    # time.sleep(1)

    # number = readNumber()
    # print "Arduino: Hey RPI, I received a digit ", number
    # print

