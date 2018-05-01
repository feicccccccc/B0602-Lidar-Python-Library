#import pyserial library

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# Store 1 frame of data

class B0602Lidar:
    angle = 0
    signalStrength = 0
    dsit = 0

def init(port):
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = port
    try:
        ser.open()
        if ser.is_open:
            print("B0602 Lidar Connected")
    except serial.serialutil.SerialException as err:
        print(err)
        exit(0)
    return ser

def bytes_to_int (input_bytes) : #change byte to int
    isinstance(input_bytes, bytes) or exit (99)
    if (len(input_bytes) == 0) : return 0
    #(input_bytes[0] < 0x80) or exit (98)

    shift = i1 = 0
    for p in range(1, len(input_bytes)+1) :
        i1 += (input_bytes[-p] << shift)
        shift += 8

    return i1

def plt_dynamic(x, y, ax, colors=['b']):
    for color in colors:
        ax.plot(x, y, color)
    fig.canvas.draw()


if __name__ == "__main__":

    xdata = []
    ydata = []

    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    c = ax.scatter(xdata, ydata)
    ax.set_ylim(0, 6000)
    line, = ax.plot(xdata, ydata, 'ro')

    flag = []
    data = []
    length = 0
    data_length = 0
    cmd = ''

    zero_offest = 0
    rpm = 0

    print("Initializing the B0602 Lidar")
    ser = init('/dev/cu.SLAB_USBtoUART')

    print("Setting up matplot")


    while 1:
        c = ser.read()
        if c == b'\xAA': #flag
            cur = time.time()
            print("Current time tick:",cur)
            print("Package start")
            temp = []
            temp = ser.read(2)
            print("Length bit:",format(temp[0], '02x'),format(temp[1], '02x'))
            length = bytes_to_int(temp)
            print("Length of packet:",length)
            if ser.read() == b'\x00':
                print("addr = 0x00")
            else:
                print("Addr not match")
                break
            if ser.read() == b'\x61':
                print("type = 0x61")
            else:
                print("type not match")
                break
            cmd = ser.read()
            if cmd == b'\xAD':
                print("Command = 0xAD")
            else:
                if cmd == b'\xAE':
                    print("Command = \xAE")
                    print("Lidar Sensor Error")
                    exit(1)
            data_length_byte = ser.read(2)
            data_length = bytes_to_int(data_length_byte)
            print("Expected data length =",data_length)
            if length - data_length == 8:
                print("bit length confirmed")
            else:
                print("bit length not match")
                break
            rpm = ser.read()[0] * 0.05
            print("rpm(r/s) =",rpm)
            zero_offset_bit = ser.read(2)
            print("Zero offset bit:",format(zero_offset_bit[0], '02x'),format(zero_offset_bit[1], '02x'))
            zero_offest = bytes_to_int(zero_offset_bit)/100
            if zero_offest >= 2**15:
                zero_offest -= 2**16
            print("Zero offset(degree) =",zero_offest)
            for i in range(0,20):
                print("Loop count:",i)
                angle_bit = ser.read(2)
                print("Angle bit:",format(angle_bit[0], '02x'),format(angle_bit[1], '02x'))
                angle = bytes_to_int(angle_bit)/100 + zero_offest
                print("Angle(degree)",angle)
                signalStrength_bit = ser.read()
                print("signal Strength bit:", format(signalStrength_bit[0], '02x'))
                signalStrength = ord(signalStrength_bit)
                print("signal Strength:", format(signalStrength))
                dist_bit = ser.read(2)
                print("Dist bit:",format(dist_bit[0], '02x'),format(dist_bit[1], '02x'))
                dist = bytes_to_int(dist_bit) * 0.25 #in mm
                print("Distance (mm):",dist)
                data.append([angle,signalStrength,dist])
            end = time.time()
            print("Time take for 1 packet", (end - cur)*1000)
            print("Data package:",data)