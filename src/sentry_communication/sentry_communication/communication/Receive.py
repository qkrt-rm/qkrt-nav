import struct
from .CRC import calculateCRC8, calculateCRC16

HEAD_FMT = "<BHBBH"
HEAD_SIZE = struct.calcsize(HEAD_FMT)

def parse_frame(frame: bytes):
    if len(frame) < HEAD_SIZE + 2:
        return None

    head, payload_len, seq, crc8, msg_type = struct.unpack_from(HEAD_FMT, frame, 0)

    if head != 0xA5:
        return None

    expected_len = HEAD_SIZE + payload_len + 2
    if len(frame) != expected_len:
        return None

    body = frame[HEAD_SIZE:HEAD_SIZE + payload_len]
    recv_crc16 = struct.unpack_from("<H", frame, HEAD_SIZE + payload_len)[0]

    if calculateCRC8(frame[0:4]) != crc8:
        return None

    if calculateCRC16(frame[0:HEAD_SIZE + payload_len]) != recv_crc16:
        return None

    return {
        "payload_len": payload_len,
        "seq": seq,
        "msg_type": msg_type,
        "body": body,
        "crc8": crc8,
        "crc16": recv_crc16,
    }