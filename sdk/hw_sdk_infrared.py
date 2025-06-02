#!/usr/bin/python3
# coding=utf8
import time
import smbus2


class Infrared:

    def __init__(self, address=0x78, register=0x01, bus=1):
        self.address = address
        self.register = register
        self.busno = bus

    def readData(self) :
        try:
            with smbus2.SMBus(self.busno) as bus:
                #bus.pec = 1  # Enable PEC
                value = bus.read_byte_data(self.address, self.register)
                print(value)
        except BaseException as e:
            print(e)
        return [True if value & v > 0 else False for v in [0x01, 0x02, 0x04, 0x08]]

if __name__ == "__main__":
    line = Infrared()
    while True:
        data = line.readData()
        print("Sensor1:", data[0], " Sensor2:", data[1], " Sensor3:", data[2], " Sensor4:", data[3])
        time.sleep(0.5)


