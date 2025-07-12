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

    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.port=port
        self.baudrate=baudrate
        self.timeout=timeout
        self._serial=None
    
    def connect(self):
        #opens the serial and initialize the device
        try:
            self-serial=serial.Serial(self.port,self.baudrate,timeout=self.timeout)
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
        desc=self._serial.read(self.DESCRIPTOR_LEN)
        if len(desc) != self.DESCRIPTOR_LEN:
            raise LidarError("Received Descriptor too short")
        
        if not (desc[0]==self.SINC_A and desc[1]==self.SYNC_B):
            raise LidarError("Received Descriptor malformed")
        size=desc[2]
        return size
    
    def readResponse(self,size):
        #reads the size byte of the payload
        data=self._serial.read(size)
        if(len(data)!=size):
            raise LidarError("Received response too short")
        return data
    
    def getInfo(self):
        #returns a dict of with principal datas
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
        self.sendCmd(self.CMD_START)
        _=self.readDescriptor()


