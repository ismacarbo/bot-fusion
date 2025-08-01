import serial
import struct
import time

class LidarError(Exception):
    pass


class Lidar:
    SYNC_A = 0xA5
    SYNC_B = 0x5A

    CMD_GET_INFO   = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_START      = 0x20
    CMD_STOP       = 0x25

    DESCRIPTOR_LEN = 7

    SET_PWM_BYTE = 0xF0
    DEFAULT_MOTOR_PWM = 660
    MAX_MOTOR_PWM = 1023

    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.port=port
        self.baudrate=baudrate
        self.timeout=timeout
        self._serial=None
        self._motor_speed = self.DEFAULT_MOTOR_PWM
        self.motor_running = False

    def start_motor(self):
        #starts the motor
        self._serial.setDTR(False)
        payload = struct.pack("<H", self._motor_speed)
    
        packet = bytes([self.SYNC_A, self.SET_PWM_BYTE, 2]) + payload
        checksum = 0
        for b in packet:
            checksum ^= b
        packet += bytes([checksum])
        self._serial.write(packet)

        self.motor_running = True
        time.sleep(0.1)  


    def stop_motor(self):
        #stops the motor
        payload = struct.pack("<H", 0)
        packet = bytes([self.SYNC_A, self.SET_PWM_BYTE, 2]) + payload
        checksum = 0
        for b in packet:
            checksum ^= b
        packet += bytes([checksum])
        self._serial.write(packet)
        self._serial.setDTR(True)

        self.motor_running = False
        time.sleep(0.1)
    
    def connect(self):
        #opens the serial and initialize the device
        try:
            self._serial=serial.Serial(self.port,self.baudrate,timeout=self.timeout)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as e:
            raise LidarError(f"Impossible to open {self.port}:{e}")
        
    def disconnect(self):
        #closes the serial
        if self._serial and self._serial.is_open:
            self._serial.close()

    def sendCmd(self,cmd):
        #sends a command without payload
        packet=bytes([self.SYNC_A,cmd])
        self._serial.write(packet)
    
    def readDescriptor(self):
        #reads and checks the descriptor of response
        sync = bytearray()
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            b = self._serial.read(1)
            if not b:
                break
            sync += b
            if len(sync) >= 2 and sync[-2] == self.SYNC_A and sync[-1] == self.SYNC_B:
                rest = self._serial.read(self.DESCRIPTOR_LEN - 2)
                if len(rest) != self.DESCRIPTOR_LEN - 2:
                    raise LidarError("Received Descriptor too short")
                desc = bytes([self.SYNC_A, self.SYNC_B]) + rest
                return desc[2]
        raise LidarError("Timeout waiting for valid descriptor")
    
    def readResponse(self,size):
        #reads the size byte of the payload
        data=self._serial.read(size)
        if(len(data)!=size):
            raise LidarError("Received response too short")
        return data
    
    def getInfo(self):
        #returns a dict of with principal datas
        self._serial.reset_input_buffer()
        self.sendCmd(self.CMD_GET_INFO)
        size=self.readDescriptor()
        raw=self.readResponse(size)
        return {
            "model": raw[0],
            "firmware": (raw[2],raw[1]),
            "hardware": raw[3],
            "serial": raw[3:].hex().upper()
        }

    def getHealth(self):
        #returns a tuple of status,error_code
        self.sendCmd(self.CMD_GET_HEALTH)
        size=self.readDescriptor()
        raw=self.readResponse(size)
        statusMap={0:"Good",1:"Warning",2:"Error"}
        status=statusMap.get(raw[0],"Unknown")
        err=(raw[1]<<8) | raw[2]
        return status,err
    
    def startScan(self):
        #start scan in normal mode
        if not self.motor_running:
            self.start_motor()
        self.sendCmd(self.CMD_START)
        time.sleep(0.1)
        _=self.readDescriptor()
        self._serial.reset_input_buffer()

    def stopScan(self):
        #stops the scan
        self.sendCmd(self.CMD_STOP)
        time.sleep(0.1)
        if self.motor_running:
            self.stop_motor()

    def iterScan(self):
        buffer = []
        while True:
            
            b0 = self._serial.read(1)
            if not b0: 
                continue
            b0 = b0[0]
            
            if (b0 & 0x1) == 0:
                continue

            rest = self._serial.read(4)
            if len(rest) < 4:
                continue
            packet = bytes([b0]) + rest

            try:
                new_scan, quality, angle, dist = self.procesScan(packet)
            except LidarError:
                
                continue
            if new_scan and buffer:
                yield buffer
                buffer = []
            if dist > 0:
                buffer.append((quality, angle, dist))


    @staticmethod
    def procesScan(raw):
        #data parsing
        b0, b1, b2, b3, b4 = raw
        new = bool(b0 & 0x1)
        if new != bool((b0 >> 1) & 0x1) ^ True:
            raise LidarError("Flag scan mismatch")
        quality = b0 >> 2
        angle = (((b1 >> 1) | (b2 << 7)) / 64.0)  
        dist  = ((b3 | (b4 << 8)) / 4.0)        
        return new, quality, angle, dist
