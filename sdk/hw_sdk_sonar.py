import sys
import time
from smbus2 import SMBus, i2c_msg


class Sonar:
    
    def __init__(self, address=0x77, register=0x01, bus=1):
        self.address = address
        self.busno   = bus
        self.Pixels  = [0,0]
        self.RGBMode = 2

    def setRGBMode(self, mode):
        try:
            with SMBus(self.busno) as bus:
                bus.write_byte_data(self.address, self.RGBMode, mode)
        except BaseException as e:
            print(e)

    def setPixelColor(self, index, rgb):
        color = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
        try:
            if index != 0 and index != 1:
                return 
            start_reg = 3 if index == 0 else 6
            with SMBus(self.busno) as bus:
                bus.write_byte_data(self.address, start_reg, 0xFF & (color >> 16))
                bus.write_byte_data(self.address, start_reg+1, 0xFF & (color >> 8))
                bus.write_byte_data(self.address, start_reg+2, 0xFF & color)
                self.Pixels[index] = color
        except BaseException as e:
            print(e)

    def getPixelColor(self, index):
        if index != 0 and index != 1:
            raise ValueError("Invalid pixel index", index)
        return ((self.Pixels[index] >> 16) & 0xFF,
                (self.Pixels[index] >> 8) & 0xFF,
                self.Pixels[index] & 0xFF)

    def getDistance(self):
        dist = 99999
        try:
            with SMBus(self.busno) as bus:
                write = i2c_msg.write(self.address, [0,])
                bus.i2c_rdwr(write)
                read  = i2c_msg.read(self.address, 2)
                bus.i2c_rdwr(read)
                dist = int.from_bytes(bytes(list(read)), byteorder='little', signed=False)
                if dist > 5000:
                    dist = 5000
        except BaseException as e:
            print(e)
        return dist

if __name__ == '__main__':
    s = Sonar()
    s.setRGBMode(0)
    s.setPixelColor(0, (0, 0, 0))
    s.setPixelColor(1, (0, 0, 0))
    time.sleep(0.1)
    s.setPixelColor(0, (255, 0, 0))
    s.setPixelColor(1, (255, 0, 0))
    time.sleep(1)
    s.setPixelColor(0, (0, 255, 0))
    s.setPixelColor(1, (0, 255, 0))
    time.sleep(1)
    s.setPixelColor(0, (0, 0, 255))
    s.setPixelColor(1, (0, 0, 255))
    time.sleep(1)
    while True:
        time.sleep(0.1)
        print(s.getDistance())

