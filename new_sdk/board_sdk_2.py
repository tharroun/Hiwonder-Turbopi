import logging
import struct
import time
import serial
import threading
from typing import List, Tuple
import collections

# Fast SDK Utilities
from check_sum import checksum_crc8

from enum import Enum

"""
These function codes correspond to the board's functions, and are used in
communication data packets.
"""
class Functions(Enum):
    FUNC_SYS:       int = 0    # SYS is the battery voltage.
    FUNC_LED:       int = 1
    FUNC_BUZZER:    int = 2
    FUNC_MOTOR:     int = 3
    FUNC_PWM_SERVO: int = 4
    FUNC_RGB:       int = 11
    FUNC_NONE:      int = 12

"""
Finite state machine codes refer to the expected bytes
in communication data packets
"""
class FSM(Enum):
    START_BYTE_1:  int = 0
    START_BYTE_2:  int = 1
    FUNCTION_BYTE: int = 2
    LENGTH_BYTE:   int = 3
    ID_BYTE:       int = 4
    DATA_BYTES:    int = 5
    CHECKSUM_BYTE: int = 6

class BoardSDK:
    """
    Represents a controller board with functionality to set RGB LEDs, buzzer, battery, and motors.
    """

    MAGIC_HEADER_1 = 0xAA
    MAGIC_HEADER_2 = 0x55
    MAGIC_BATTERY  = 0x04

    def __init__(
        self, device: str = "/dev/ttyAMA0", baudrate: int = 1000000, timeout: int = 5, enable_recv: bool = True
    ):
        """
        Initialize the board with a serial connection.

        :param device: Serial port device name.
        :param baudrate: Baud rate for serial communication.
        :param timeout: Timeout in seconds for serial communication.
        """
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(filename='BordSDK.log', filemode='w', level=logging.INFO)

        try:
            self.port = serial.Serial(None, baudrate=baudrate, timeout=timeout)
            self.port.rts = False
            self.port.dtr = False
            self.port.setPort(device)
            self.port.open()
            time.sleep(0.1)
            self.logger.info("Opened serial port.")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to initialize serial port: {e}")
        
        """
        Added thread to listen on the serial port
        Added queue for the data (battery) 
        """
        self.enable_recv = enable_recv
        if enable_recv:
            self.parsers = {
                Functions.FUNC_SYS:       self._packet_sys,        # SYS is the battery voltage.
                Functions.FUNC_PWM_SERVO: self._packet_pwm_servo  # PWM_SERVO is the servo function.
            }
            
            self.stop_listening = False
            self.listening = threading.Thread(target=self._listen_thread, daemon=False)
            self.frame          = []
            self.bytes_received = 0
            self.state          = FSM.START_BYTE_1
            self.queue_sys      = collections.deque(maxlen=1)
            self.queue_sys.append(0)
            self.listening.start()
            self.logger.info("Started listening to serial port.")
            self.servo_read_lock = threading.Lock()
            self.queue_servo     = collections.deque(maxlen=1)
            self.queue_servo.append(0)
        


        # ------------------    
        return
    
    def _packet_pwm_servo(self, data):
        """
        Handle a FUNC_PWM_SERVO data packet 
        """
        self.queue_servo.append(data)
        return

    def _packet_sys(self, data):
        """
        Handle a FUNC_SYS data packet 
        """
        self.queue_sys.append(data)
        return

    def _listen_thread(self):
        """
        Finite state machine for communication from the board.
        """
        while not self.stop_listening:
            packet = self.port.read()
            # ------------------------------------------------------------------
            # Finite state machine to unpack the data packet
            if packet:
                for byte in packet:
                    if self.state == FSM.START_BYTE_1:
                        if byte == self.MAGIC_HEADER_1: self.state = FSM.START_BYTE_2
                        continue
                    elif self.state == FSM.START_BYTE_2:
                        if byte == self.MAGIC_HEADER_2: self.state = FSM.FUNCTION_BYTE
                        else :                     self.state = FSM.START_BYTE_1 
                        continue
                    elif self.state == FSM.FUNCTION_BYTE:
                        if byte < Functions.FUNC_NONE.value:
                            self.frame = [byte, 0]
                            self.state = FSM.LENGTH_BYTE
                        else :                     self.state = FSM.START_BYTE_1
                        continue
                    elif self.state == FSM.LENGTH_BYTE:
                        self.frame[1] = byte
                        self.bytes_received = 0
                        if byte == 0:              self.state = FSM.CHECKSUM_BYTE
                        else:                      self.state = FSM.DATA_BYTES  
                        continue
                    elif self.state == FSM.DATA_BYTES:
                        self.frame.append(byte)
                        self.bytes_received += 1
                        if self.bytes_received >= self.frame[1]: self.state = FSM.CHECKSUM_BYTE
                        continue
                    elif self.state == FSM.CHECKSUM_BYTE:
                        if checksum_crc8(bytes(self.frame)) == byte:
                            _function = Functions(self.frame[0])
                            _data     = bytes(self.frame[2:])
                            if _function in self.parsers: self.parsers[_function](_data)
                        else:
                            self.logger.warning(f"Packet checksum failed : {packet}")
                        self.state = FSM.START_BYTE_1
                        continue
                 # END LOOP OVER BYTES IN THE PACKET   
            # ------------------------------------------------------------------
            else:
                time.sleep(0.01)
        return


    def stop_BoardSDK(self):
        """
        Stop the listening thread and close the serial port.
        To conintue with the main program, a new BoardSDK must be created.
        First, break the loop in listen_thread, which will close the serial port.
        Second, wait for the thread to complete 
        """
        if self.enable_recv:
            self.stop_listening = True
            self.enable_recv    = False
            self.listening.join()
            self.logger.info("Stopped listening thread.")
        else: 
            self.logger.warning(f"Reception was not enabled to stop the listening thread.")
        self.port.close()
        self.logger.info("Closed serial port.")
        return

    def get_servo(self, servo_id: int):
        if servo_id > 2 or servo_id < 0:
            self.logger.warning(f"Servo id must be 1 or 2")
            return None
        with self.servo_read_lock:
            self._send_data_to_port(Functions.FUNC_PWM_SERVO,[0x05,servo_id])
            data = self.queue_servo[0]
            servo_id, cmd, info = struct.unpack("<BBH", data)
            return info
        return None

    def get_battery(self):
        """
        Reads the current battery voltage in the sys queue

        :returns: The battery voltage.
        :returns: None if the packet was not correct, or if receivng is turned off.
        :rtype: unsigned short
        """
        if self.enable_recv:
            d = self.queue_sys[0]
            if len(d) == 0 :
                self.logger.warning(f"Battery data not there")
                return None
            if d[0] == self.MAGIC_BATTERY:
                v = struct.unpack('<H', d[1:])[0] # unsigned short
                return v/1000.0
            else:
                self.logger.warning(f"Battery data corrupted")
                return None
        else:
            self.logger.warning(f"Reception not enabled to read the battery voltage")
            return None
         
    
    def set_rgb(self, pixels: List[Tuple[int, int, int, int]]) -> None:
        """
        Set the RGB values for the board's LEDs.

        :param pixels: List of tuples where each tuple contains (index, R, G, B).
        :raises ValueError: If any of the RGB values are out of range.
        """
        data = [0x01, len(pixels)]

        for pixel in pixels:
            if len(pixel) != 4:
                raise ValueError("Each pixel must be a tuple of (index, R, G, B).")
            index, r, g, b = pixel
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError(
                    f"RGB values must be in the range 0-255. Received: {r}, {g}, {b}"
                )
            data.extend(struct.pack("<BBBB", int(index - 1), int(r), int(g), int(b)))

        self._send_data_to_port(Functions.FUNC_RGB.value, data)

    def set_led(
        self, on_time: float, off_time: float, repeat: int = 1, led_id: int = 1
    ) -> None:
        """
        Control the LED with specified on/off timings and repetitions.

        :param on_time: Time in seconds the LED stays on (must be > 0).
        :param off_time: Time in seconds the LED stays off (must be >= 0).
        :param repeat: Number of times the LED blinks (must be >= 1).
        :param led_id: LED ID to control (must be in range 1-255).
        :raises ValueError: If any parameter is out of the expected range.
        """
        if on_time <= 0:
            raise ValueError(f"on_time must be greater than 0. Received: {on_time}")
        if off_time < 0:
            raise ValueError(
                f"off_time must be greater than or equal to 0. Received: {off_time}"
            )
        if repeat < 1:
            raise ValueError(f"repeat must be at least 1. Received: {repeat}")
        if not (1 <= led_id <= 255):
            raise ValueError(f"led_id must be in the range 1-255. Received: {led_id}")

        on_time_ms = int(on_time * 1000)
        off_time_ms = int(off_time * 1000)
        data = struct.pack("<BHHH", led_id, on_time_ms, off_time_ms, repeat)

        self._send_data_to_port(Functions.FUNC_LED.value, data)

    def set_buzzer(
        self, freq: int, on_time: float, off_time: float, repeat: int = 1
    ) -> None:
        """
        Control the buzzer with specified frequency, on/off timings, and repetitions.

        :param freq: Frequency of the buzzer in Hz (must be > 0).
        :param on_time: Time in seconds the buzzer stays on (must be > 0).
        :param off_time: Time in seconds the buzzer stays off (must be >= 0).
        :param repeat: Number of times the buzzer activates (must be >= 1).
        :raises ValueError: If any parameter is out of the expected range.
        """
        if freq <= 0:
            raise ValueError(f"freq must be greater than 0. Received: {freq}")
        if on_time <= 0:
            raise ValueError(f"on_time must be greater than 0. Received: {on_time}")
        if off_time < 0:
            raise ValueError(
                f"off_time must be greater than or equal to 0. Received: {off_time}"
            )
        if repeat < 1:
            raise ValueError(f"repeat must be at least 1. Received: {repeat}")

        on_time_ms = int(on_time * 1000)
        off_time_ms = int(off_time * 1000)
        data = struct.pack("<HHHH", freq, on_time_ms, off_time_ms, repeat)

        self._send_data_to_port(Functions.FUNC_BUZZER.value, data)

    def set_motor_speed(self, speeds: List[Tuple[int, float]]) -> None:
        """
        Set the speed of motors.

        :param speeds: List of tuples where each tuple contains (index, speed).
        """
        data = [0x01, len(speeds)]
        for index, speed in speeds:
            data.extend(struct.pack("<Bf", int(index - 1), float(speed)))

        self._send_data_to_port(Functions.FUNC_MOTOR.value, data)

    def set_motor_duty(self, duty: List[Tuple[int, float]]) -> None:
        """
        Set motor duty cycles

        :param duty: List of tuples, where each tuple contains (motor_id, duty_cycle).
                      motor_id: 1-based index of the motor.
                      duty_cycle: Duty cycle value as a float.
        """
        if len(duty) > 4:
            raise ValueError("Too many motors specified. Maximum is 4.")

        # Preallocate the exact required size for the data list
        data = bytearray(2 + len(duty) * 5)  # 2 for header, 5 bytes per motor (1 byte ID + 4 bytes float)
        data[0] = 0x05  # Command ID
        data[1] = len(duty)  # Number of motors

        offset = 2
        for motor_id, duty_cycle in duty:
            motor_id = motor_id - 1  # Convert to 0-based index
            struct.pack_into("<Bf", data, offset, motor_id, duty_cycle)
            offset += 5

        self._send_data_to_port(Functions.FUNC_MOTOR.value, list(data))

    def _send_data_to_port(self, func: int, data: List[int]) -> None:
        """
        Write a buffer to the serial port.

        :param func: Function code for the operation.
        :param data: Data to send as a list of integers.
        """
        buf = [self.MAGIC_HEADER_1, self.MAGIC_HEADER_2, func, len(data)]
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        try:
            self.port.write(bytes(buf))
        except Exception as e:
            self.logger.error("Failed to send data to port: %s", e, exc_info=True)
            raise

if __name__ == "__main__":
    board = BoardSDK(enable_recv=True)
    
    board.set_rgb([(1,0,10,0)])
    time.sleep(0.5)

    #board.set_led(1.0,1.0,20,1)
    #time.sleep(1.5)
    
    board.set_buzzer(2400, 0.1, 0.9, 1)
    time.sleep(0.5)
    
    v = board.get_battery()
    if v != None : print(f"Battery : {v:4.3f} V")
    time.sleep(0.1)
    
    p1 = board.get_servo(1)
    p2 = board.get_servo(2)
    if p1 != None : print(f"Servo 1: {p1}")
    if p2 != None : print(f"Servo 1: {p2}")

    #board.set_motor_duty([(1,17.0),(2,17.0),(3,17.0),(4,17.0)])
    #time.sleep(1.0)
    #board.set_motor_duty([(1,0.0),(2,0.0),(3,0.0),(4,0.0)])

    board.stop_BoardSDK()
