import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Any, Dict, List, Optional, Tuple, Union

SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
HEADER_LEN = 6
FRAME_OVERHEAD = 2 + HEADER_LEN + 2
DEFAULT_MAX_PAYLOAD = 128


class RSPError(Exception):
    pass


class DecodeError(RSPError):
    pass


class MsgType(IntEnum):
    PING = 0x01
    ACK = 0x02
    ERROR = 0x03

    MOTOR_CMD = 0x10
    STOP_CMD = 0x11
    MODE_CMD = 0x12
    GYRO_ZERO_CMD = 0x13
    CONFIG_SET = 0x14
    HEARTBEAT_CMD = 0x15

    IMU_TELEMETRY = 0x20
    SAFETY_TELEMETRY = 0x21
    ENCODER_TELEMETRY = 0x22
    MOTOR_STATE = 0x23
    HEARTBEAT_STATE = 0x24


class FrameFlag(IntFlag):
    ACK_REQ = 0x01
    ACK_FRAME = 0x02
    ERR_FRAME = 0x04


class AckStatus(IntEnum):
    ACCEPTED_COMPLETED = 0x00
    ACCEPTED_PENDING = 0x01
    ACCEPTED_ALREADY = 0x02
    REJECTED = 0x03


class ErrorCode(IntEnum):
    UNKNOWN_MSG_TYPE = 0x01
    INVALID_LENGTH = 0x02
    CRC_MISMATCH = 0x03
    INVALID_VALUE = 0x04
    IMU_NOT_READY = 0x05
    CALIBRATION_BUSY = 0x06
    MOTORS_DISABLED = 0x07
    ENCODERS_UNAVAILABLE = 0x08
    SENSOR_TIMEOUT = 0x09
    UNSUPPORTED_MODE = 0x0A
    INTERNAL_FAULT = 0x0B
    BUSY = 0x0C
    NOT_IMPLEMENTED = 0x0D


class MotorControlMode(IntEnum):
    DIRECT_PWM = 0x00
    SAFE_DIRECT_PWM = 0x01
    VELOCITY_RESERVED = 0x02
    RESERVED = 0x03


class StopReason(IntEnum):
    USER_REQUEST = 0x00
    HOST_TIMEOUT = 0x01
    OBSTACLE_DETECTED = 0x02
    SAFETY_OVERRIDE = 0x03
    FAULT_RECOVERY = 0x04
    SHUTDOWN = 0x05


class ControllerMode(IntEnum):
    IDLE = 0x00
    MANUAL = 0x01
    AUTONOMOUS = 0x02
    CALIBRATION = 0x03
    EMERGENCY_STOP_LATCHED = 0x04


class ConfigValueType(IntEnum):
    UINT8 = 0x01
    INT16 = 0x02
    UINT16 = 0x03
    INT32 = 0x04
    UINT32 = 0x05


ValueArg = Union[int, bytes, bytearray]


_CONFIG_TYPE_TO_STRUCT = {
    ConfigValueType.UINT8: "<B",
    ConfigValueType.INT16: "<h",
    ConfigValueType.UINT16: "<H",
    ConfigValueType.INT32: "<i",
    ConfigValueType.UINT32: "<I",
}


@dataclass(frozen=True)
class Frame:
    version: int
    msg_type: int
    flags: int
    seq: int
    payload: bytes
    crc16: int

    @property
    def ack_requested(self) -> bool:
        return bool(self.flags & int(FrameFlag.ACK_REQ))

    @property
    def is_ack(self) -> bool:
        return bool(self.flags & int(FrameFlag.ACK_FRAME)) or self.msg_type == int(MsgType.ACK)

    @property
    def is_error(self) -> bool:
        return bool(self.flags & int(FrameFlag.ERR_FRAME)) or self.msg_type == int(MsgType.ERROR)


@dataclass(frozen=True)
class DecodedMessage:
    frame: Frame
    name: str
    data: Dict[str, Any]


PAYLOAD_LENGTHS: Dict[int, Optional[int]] = {
    int(MsgType.PING): 0,
    int(MsgType.ACK): 4,
    int(MsgType.ERROR): 4,
    int(MsgType.MOTOR_CMD): 6,
    int(MsgType.STOP_CMD): 1,
    int(MsgType.MODE_CMD): 2,
    int(MsgType.GYRO_ZERO_CMD): 0,
    int(MsgType.CONFIG_SET): None,
    int(MsgType.HEARTBEAT_CMD): 6,
    int(MsgType.IMU_TELEMETRY): 20,
    int(MsgType.SAFETY_TELEMETRY): 12,
    int(MsgType.ENCODER_TELEMETRY): 16,
    int(MsgType.MOTOR_STATE): 14,
    int(MsgType.HEARTBEAT_STATE): 12,
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def message_name(msg_type: int) -> str:
    try:
        return MsgType(msg_type).name
    except ValueError:
        return f"UNKNOWN_0x{int(msg_type) & 0xFF:02X}"


def expected_payload_length(msg_type: int) -> Optional[int]:
    return PAYLOAD_LENGTHS.get(int(msg_type))


def validate_payload_length(msg_type: int, payload: bytes) -> bool:
    expected = expected_payload_length(msg_type)
    return expected is None or len(payload) == expected


def build_frame(msg_type: int, payload: bytes = b"", seq: int = 0, flags: int = 0) -> bytes:
    payload = bytes(payload)
    header = struct.pack("<BBBBH", VERSION, int(msg_type) & 0xFF, int(flags) & 0xFF, int(seq) & 0xFF, len(payload))
    crc = crc16_ccitt_false(header + payload)
    return bytes((SOF1, SOF2)) + header + payload + struct.pack("<H", crc)


class StreamDecoder:
    def __init__(self, max_payload: int = DEFAULT_MAX_PAYLOAD):
        self.max_payload = int(max_payload)
        self.buffer = bytearray()
        self.frames_ok = 0
        self.frames_dropped = 0
        self.crc_errors = 0

    def feed(self, data: bytes) -> List[Frame]:
        if data:
            self.buffer.extend(data)
        frames: List[Frame] = []

        while True:
            sof_idx = self.buffer.find(bytes((SOF1, SOF2)))
            if sof_idx < 0:
                if self.buffer and self.buffer[-1] == SOF1:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break

            if sof_idx > 0:
                del self.buffer[:sof_idx]

            if len(self.buffer) < FRAME_OVERHEAD:
                break

            version, msg_type, flags, seq, length = struct.unpack_from("<BBBBH", self.buffer, 2)
            if version != VERSION or length > self.max_payload:
                self.frames_dropped += 1
                del self.buffer[0]
                continue

            frame_len = 2 + HEADER_LEN + length + 2
            if len(self.buffer) < frame_len:
                break

            body = bytes(self.buffer[2:2 + HEADER_LEN + length])
            crc_rx = struct.unpack_from("<H", self.buffer, 2 + HEADER_LEN + length)[0]
            crc_calc = crc16_ccitt_false(body)
            if crc_rx != crc_calc:
                self.frames_dropped += 1
                self.crc_errors += 1
                del self.buffer[0]
                continue

            payload = bytes(self.buffer[2 + HEADER_LEN:2 + HEADER_LEN + length])
            frames.append(Frame(version, msg_type, flags, seq, payload, crc_rx))
            self.frames_ok += 1
            del self.buffer[:frame_len]

        return frames


def encode_ack(acked_seq: int, acked_type: int, status: int = AckStatus.ACCEPTED_COMPLETED, detail: int = 0) -> bytes:
    return struct.pack("<BBBB", int(acked_seq) & 0xFF, int(acked_type) & 0xFF, int(status) & 0xFF, int(detail) & 0xFF)


def decode_ack(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.ACK, payload)
    acked_seq, acked_type, status, detail = struct.unpack("<BBBB", payload)
    return {
        "acked_seq": acked_seq,
        "acked_type": acked_type,
        "status": status,
        "detail": detail,
    }


def encode_error(error_code: int, related_type: int, related_seq: int, detail: int = 0) -> bytes:
    return struct.pack("<BBBB", int(error_code) & 0xFF, int(related_type) & 0xFF, int(related_seq) & 0xFF, int(detail) & 0xFF)


def decode_error(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.ERROR, payload)
    error_code, related_type, related_seq, detail = struct.unpack("<BBBB", payload)
    return {
        "error_code": error_code,
        "related_type": related_type,
        "related_seq": related_seq,
        "detail": detail,
    }


def encode_motor_cmd(
    pwm_left: int,
    pwm_right: int,
    control_mode: int = MotorControlMode.SAFE_DIRECT_PWM,
    reserved: int = 0,
) -> bytes:
    return struct.pack("<hhBB", int(pwm_left), int(pwm_right), int(control_mode) & 0xFF, int(reserved) & 0xFF)


def decode_motor_cmd(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.MOTOR_CMD, payload)
    pwm_left, pwm_right, control_mode, reserved = struct.unpack("<hhBB", payload)
    return {
        "pwm_left": pwm_left,
        "pwm_right": pwm_right,
        "control_mode": control_mode,
        "reserved": reserved,
    }


def encode_stop_cmd(reason: int) -> bytes:
    return struct.pack("<B", int(reason) & 0xFF)


def decode_stop_cmd(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.STOP_CMD, payload)
    (reason,) = struct.unpack("<B", payload)
    return {"reason": reason}


def encode_mode_cmd(mode: int, reserved: int = 0) -> bytes:
    return struct.pack("<BB", int(mode) & 0xFF, int(reserved) & 0xFF)


def decode_mode_cmd(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.MODE_CMD, payload)
    mode, reserved = struct.unpack("<BB", payload)
    return {"mode": mode, "reserved": reserved}


def encode_gyro_zero_cmd() -> bytes:
    return b""


def decode_gyro_zero_cmd(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.GYRO_ZERO_CMD, payload)
    return {}


def pack_config_value(value_type: int, value: ValueArg) -> bytes:
    vt = ConfigValueType(int(value_type))
    if vt == ConfigValueType.UINT8:
        return struct.pack("<B", int(value) & 0xFF)
    if vt == ConfigValueType.INT16:
        return struct.pack("<h", int(value))
    if vt == ConfigValueType.UINT16:
        return struct.pack("<H", int(value) & 0xFFFF)
    if vt == ConfigValueType.INT32:
        return struct.pack("<i", int(value))
    if vt == ConfigValueType.UINT32:
        return struct.pack("<I", int(value) & 0xFFFFFFFF)
    raise DecodeError(f"Unsupported config value type: {value_type}")


def unpack_config_value(value_type: int, raw_value: bytes) -> Union[int, bytes]:
    vt = ConfigValueType(int(value_type))
    fmt = _CONFIG_TYPE_TO_STRUCT.get(vt)
    if fmt is None:
        return raw_value
    size = struct.calcsize(fmt)
    if len(raw_value) != size:
        raise DecodeError(
            f"CONFIG_SET value length mismatch for {vt.name}: expected {size}, got {len(raw_value)}"
        )
    return struct.unpack(fmt, raw_value)[0]


def encode_config_set(param_id: int, value_type: int, value: ValueArg) -> bytes:
    return struct.pack("<BB", int(param_id) & 0xFF, int(value_type) & 0xFF) + pack_config_value(value_type, value)


def decode_config_set(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 2:
        raise DecodeError("CONFIG_SET payload too short")
    param_id, value_type = struct.unpack("<BB", payload[:2])
    raw_value = payload[2:]
    return {
        "param_id": param_id,
        "value_type": value_type,
        "value": unpack_config_value(value_type, raw_value),
        "raw_value": raw_value,
    }


def encode_heartbeat_cmd(host_time_ms: int, host_status: int = 0) -> bytes:
    return struct.pack("<IH", int(host_time_ms) & 0xFFFFFFFF, int(host_status) & 0xFFFF)


def decode_heartbeat_cmd(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.HEARTBEAT_CMD, payload)
    host_time_ms, host_status = struct.unpack("<IH", payload)
    return {"host_time_ms": host_time_ms, "host_status": host_status}


def encode_imu_telemetry(
    mcu_time_ms: int,
    yaw_mrad: int,
    yaw_rate_mrad_s: int,
    acc_x_raw: int,
    acc_y_raw: int,
    acc_z_raw: int,
    gyro_z_raw: int,
) -> bytes:
    return struct.pack(
        "<Iiihhhh",
        int(mcu_time_ms) & 0xFFFFFFFF,
        int(yaw_mrad),
        int(yaw_rate_mrad_s),
        int(acc_x_raw),
        int(acc_y_raw),
        int(acc_z_raw),
        int(gyro_z_raw),
    )


def decode_imu_telemetry(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.IMU_TELEMETRY, payload)
    mcu_time_ms, yaw_mrad, yaw_rate_mrad_s, acc_x_raw, acc_y_raw, acc_z_raw, gyro_z_raw = struct.unpack(
        "<Iiihhhh", payload
    )
    return {
        "mcu_time_ms": mcu_time_ms,
        "yaw_mrad": yaw_mrad,
        "yaw_rate_mrad_s": yaw_rate_mrad_s,
        "acc_x_raw": acc_x_raw,
        "acc_y_raw": acc_y_raw,
        "acc_z_raw": acc_z_raw,
        "gyro_z_raw": gyro_z_raw,
    }


def encode_safety_telemetry(
    mcu_time_ms: int,
    ultra_cm: int,
    ir_left_raw: int,
    ir_right_raw: int,
    safety_flags: int,
) -> bytes:
    return struct.pack(
        "<IHHHH",
        int(mcu_time_ms) & 0xFFFFFFFF,
        int(ultra_cm) & 0xFFFF,
        int(ir_left_raw) & 0xFFFF,
        int(ir_right_raw) & 0xFFFF,
        int(safety_flags) & 0xFFFF,
    )


def decode_safety_telemetry(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.SAFETY_TELEMETRY, payload)
    mcu_time_ms, ultra_cm, ir_left_raw, ir_right_raw, safety_flags = struct.unpack("<IHHHH", payload)
    return {
        "mcu_time_ms": mcu_time_ms,
        "ultra_cm": ultra_cm,
        "ir_left_raw": ir_left_raw,
        "ir_right_raw": ir_right_raw,
        "safety_flags": safety_flags,
    }


def encode_encoder_telemetry(
    mcu_time_ms: int,
    ticks_left: int,
    ticks_right: int,
    dt_ms: int,
    enc_flags: int,
) -> bytes:
    return struct.pack(
        "<IiiHH",
        int(mcu_time_ms) & 0xFFFFFFFF,
        int(ticks_left),
        int(ticks_right),
        int(dt_ms) & 0xFFFF,
        int(enc_flags) & 0xFFFF,
    )


def decode_encoder_telemetry(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.ENCODER_TELEMETRY, payload)
    mcu_time_ms, ticks_left, ticks_right, dt_ms, enc_flags = struct.unpack("<IiiHH", payload)
    return {
        "mcu_time_ms": mcu_time_ms,
        "ticks_left": ticks_left,
        "ticks_right": ticks_right,
        "dt_ms": dt_ms,
        "enc_flags": enc_flags,
    }


def encode_motor_state(
    mcu_time_ms: int,
    target_pwm_left: int,
    target_pwm_right: int,
    current_pwm_left: int,
    current_pwm_right: int,
    motor_flags: int,
) -> bytes:
    return struct.pack(
        "<IhhhhH",
        int(mcu_time_ms) & 0xFFFFFFFF,
        int(target_pwm_left),
        int(target_pwm_right),
        int(current_pwm_left),
        int(current_pwm_right),
        int(motor_flags) & 0xFFFF,
    )


def decode_motor_state(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.MOTOR_STATE, payload)
    mcu_time_ms, target_pwm_left, target_pwm_right, current_pwm_left, current_pwm_right, motor_flags = struct.unpack(
        "<IhhhhH", payload
    )
    return {
        "mcu_time_ms": mcu_time_ms,
        "target_pwm_left": target_pwm_left,
        "target_pwm_right": target_pwm_right,
        "current_pwm_left": current_pwm_left,
        "current_pwm_right": current_pwm_right,
        "motor_flags": motor_flags,
    }


def encode_heartbeat_state(
    mcu_time_ms: int,
    uptime_s: int,
    status_flags: int,
    fw_major: int,
    fw_minor: int,
    error_code: int,
) -> bytes:
    return struct.pack(
        "<IHHBBH",
        int(mcu_time_ms) & 0xFFFFFFFF,
        int(uptime_s) & 0xFFFF,
        int(status_flags) & 0xFFFF,
        int(fw_major) & 0xFF,
        int(fw_minor) & 0xFF,
        int(error_code) & 0xFFFF,
    )


def decode_heartbeat_state(payload: bytes) -> Dict[str, int]:
    _require_payload_len(MsgType.HEARTBEAT_STATE, payload)
    mcu_time_ms, uptime_s, status_flags, fw_major, fw_minor, error_code = struct.unpack("<IHHBBH", payload)
    return {
        "mcu_time_ms": mcu_time_ms,
        "uptime_s": uptime_s,
        "status_flags": status_flags,
        "fw_major": fw_major,
        "fw_minor": fw_minor,
        "error_code": error_code,
    }


def _require_payload_len(msg_type: MsgType, payload: bytes) -> None:
    expected = expected_payload_length(int(msg_type))
    if expected is not None and len(payload) != expected:
        raise DecodeError(
            f"{message_name(int(msg_type))} payload length mismatch: expected {expected}, got {len(payload)}"
        )


def decode_payload(msg_type: int, payload: bytes) -> Dict[str, Any]:
    mt = int(msg_type)
    if mt == int(MsgType.PING):
        return {}
    if mt == int(MsgType.ACK):
        return decode_ack(payload)
    if mt == int(MsgType.ERROR):
        return decode_error(payload)
    if mt == int(MsgType.MOTOR_CMD):
        return decode_motor_cmd(payload)
    if mt == int(MsgType.STOP_CMD):
        return decode_stop_cmd(payload)
    if mt == int(MsgType.MODE_CMD):
        return decode_mode_cmd(payload)
    if mt == int(MsgType.GYRO_ZERO_CMD):
        return decode_gyro_zero_cmd(payload)
    if mt == int(MsgType.CONFIG_SET):
        return decode_config_set(payload)
    if mt == int(MsgType.HEARTBEAT_CMD):
        return decode_heartbeat_cmd(payload)
    if mt == int(MsgType.IMU_TELEMETRY):
        return decode_imu_telemetry(payload)
    if mt == int(MsgType.SAFETY_TELEMETRY):
        return decode_safety_telemetry(payload)
    if mt == int(MsgType.ENCODER_TELEMETRY):
        return decode_encoder_telemetry(payload)
    if mt == int(MsgType.MOTOR_STATE):
        return decode_motor_state(payload)
    if mt == int(MsgType.HEARTBEAT_STATE):
        return decode_heartbeat_state(payload)
    raise DecodeError(f"Unknown message type: 0x{mt:02X}")


def decode_frame(frame: Frame) -> DecodedMessage:
    return DecodedMessage(frame=frame, name=message_name(frame.msg_type), data=decode_payload(frame.msg_type, frame.payload))
