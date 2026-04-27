from __future__ import annotations

FRAME_START = 0xAA
FRAME_MAX_PAYLOAD = 64

CMD_EXT = 0x0
CMD_GET_STATUS = 0x1
CMD_SET_POSITION = 0x2
CMD_SET_DUTY = 0x3
CMD_CAL_START = 0x4
CMD_CAL_STATUS = 0x5
CMD_CAL_READ_POINT = 0x6
CMD_CAL_WRITE_BLOB = 0x7
CMD_CAL_COMMIT = 0x8
CMD_CAL_CLEAR = 0x9
CMD_CAL_INFO = 0xA
CMD_ENTER_BOOT = 0xB
CMD_SET_ADVANCE = 0xC
CMD_SET_QUIET = 0xD
CMD_STEP_SET = 0xE
CMD_STEP_TRANSITION = 0xF

EXT_CAPTURE_START = 0x1
EXT_CAPTURE_STATUS = 0x2
EXT_CAPTURE_READ = 0x3
EXT_SET_OBSERVER = 0x4

OBSERVER_RAW = 0
OBSERVER_LP2 = 1
OBSERVER_LP4 = 2
OBSERVER_LP8 = 3
OBSERVER_AB1 = 4
OBSERVER_AB2 = 5
OBSERVER_AB3 = 6
OBSERVER_KCV1 = 7
OBSERVER_KCV2 = 8
OBSERVER_KCV3 = 9
OBSERVER_KCA1 = 10
OBSERVER_KCA2 = 11
OBSERVER_KCA3 = 12

OBSERVER_NAMES = {
    OBSERVER_RAW: "raw",
    OBSERVER_LP2: "lp2",
    OBSERVER_LP4: "lp4",
    OBSERVER_LP8: "lp8",
    OBSERVER_AB1: "ab1",
    OBSERVER_AB2: "ab2",
    OBSERVER_AB3: "ab3",
    OBSERVER_KCV1: "kcv1",
    OBSERVER_KCV2: "kcv2",
    OBSERVER_KCV3: "kcv3",
    OBSERVER_KCA1: "kca1",
    OBSERVER_KCA2: "kca2",
    OBSERVER_KCA3: "kca3",
}

BOOT_MAGIC = 0x424F4F54

RESULT_OK = 0
RESULT_BAD_STATE = 1
RESULT_BAD_ARG = 2
RESULT_NOT_CALIBRATED = 3
RESULT_BUSY = 4
RESULT_RANGE = 5
RESULT_FLASH = 6
RESULT_CRC = 7

MODE_IDLE = 0
MODE_RUN = 1
MODE_CAL = 2

FLAG_CAL_VALID = 1 << 0
FLAG_BUSY = 1 << 1
FLAG_POSITION_REACHED = 1 << 2
FLAG_FAULT = 1 << 3
FLAG_SENSOR_OK = 1 << 4

FAULT_SENSOR = 1 << 0
FAULT_UNCALIBRATED = 1 << 1
FAULT_FLASH = 1 << 2


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def encode_frame(address: int, cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= address <= 0xF:
        raise ValueError("address must fit in 4 bits")
    if len(payload) > FRAME_MAX_PAYLOAD:
        raise ValueError("payload too large")
    frame = bytes((FRAME_START, ((address & 0xF) << 4) | (cmd & 0xF), len(payload))) + payload
    return frame + bytes((crc8(frame),))


def decode_frame(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 4:
        raise ValueError("frame too short")
    if frame[0] != FRAME_START:
        raise ValueError("bad start byte")
    if frame[2] > FRAME_MAX_PAYLOAD:
        raise ValueError("payload too large")
    if len(frame) != frame[2] + 4:
        raise ValueError("bad frame length")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("bad frame crc")
    return frame[1] >> 4, frame[1] & 0xF, frame[3:-1]
