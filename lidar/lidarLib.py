from dataclasses import dataclass


@dataclass
class ScanPoint: 
    angle: float
    distance: float
    quality: int
    start: bool

def decodeScanPacket(data: bytearray)->ScanPoint:
    
    if (len(data)!=5):
        raise ValueError("Invalid scan packet length")
    
    b0, b1, b2, b3, b4 = data
    newScan=(b0&0x01)!=0
    quality=b0>>2
    
    angleRaw=((b1>>1)| (b2<<7))
    angle=angleRaw*64.0
    
    distanceRaw=b3|(b4<<8)
    distance=distanceRaw/4.0
    
    return ScanPoint(angle, distance, quality, newScan)