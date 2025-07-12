import serial
import struct
import time


class Lidar:
    SYNC_A = 0xA5
    SYNC_B = 0x5A

    CMD_GET_INFO   = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_START      = 0x20
    CMD_STOP       = 0x25

    DESCRIPTOR_LEN = 7