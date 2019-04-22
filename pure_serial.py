import serial

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

import numpy as np

def crc16(data: bytes):
    '''
    CRC-16-CCITT Algorithm
    '''
    data = bytearray(data)
    poly = 0x8005
    crc = 0xFFFF
    for b in data:
        cur_byte = 0xFF & b
        for _ in range(0, 8):
            if (crc & 0x0001) ^ (cur_byte & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1

            cur_byte >>= 1

    crc = (~crc & 0xFFFF)
    crc = (crc << 8) | ((crc >> 8) & 0xFF)

    return np.uint16(crc)

if __name__ == "__main__":
    ser = init('/dev/cu.SLAB_USBtoUART')
    while 1:
        c = ser.read()
        print(format(ord(c), '02x'))
    print(format(crc16(B''),'02X'))