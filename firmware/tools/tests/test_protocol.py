import io
import sys
import struct
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pennycal import (  # noqa: E402
    CAL_BLOB_SIZE,
    BrushedControl,
    CalibrationError,
    CMD_GET_STATUS,
    Stm32Client,
    build_blob,
    decode_frame,
    encode_frame,
    iter_blob_chunks,
    validate_blob,
)


class ProtocolTests(unittest.TestCase):
    def test_brushed_control_supports_high_gain(self) -> None:
        payload = BrushedControl(kp=300.0, kd=12.5, clip=500).payload()
        self.assertEqual(len(payload), 14)
        self.assertEqual(struct.unpack("<iihhh", payload), (76800, 3200, 0, 0, 500))

    def test_client_keeps_late_reply_for_next_command(self) -> None:
        class FakePort:
            def __init__(self) -> None:
                self.pending: list[tuple[float, bytes]] = []
                self.reset_calls = 0
                self.tx = bytearray()

            def write(self, data: bytes) -> int:
                self.tx.extend(data)
                if len(self.tx) >= 3:
                    expected_len = self.tx[2] + 4
                    if len(self.tx) == expected_len:
                        address, cmd, _payload = decode_frame(bytes(self.tx))
                        self.tx.clear()
                        frame = encode_frame(address, cmd, b"\x01")
                        delay = 0.03 if len(self.pending) == 0 else 0.0
                        self.pending.append((time.monotonic() + delay, frame))
                return len(data)

            def flush(self) -> None:
                return None

            def reset_input_buffer(self) -> None:
                self.reset_calls += 1
                self.pending.clear()

            def read(self, n: int = 1) -> bytes:
                if not self.pending:
                    time.sleep(0.001)
                    return b""
                ready_at, frame = self.pending[0]
                if time.monotonic() < ready_at:
                    time.sleep(0.001)
                    return b""
                chunk = frame[:n]
                rest = frame[n:]
                if rest:
                    self.pending[0] = (ready_at, rest)
                else:
                    self.pending.pop(0)
                return chunk

        port = FakePort()
        client = Stm32Client(port, address=2)
        with self.assertRaises(TimeoutError):
            client.exchange(CMD_GET_STATUS, timeout=0.01)
        self.assertEqual(client.exchange(CMD_GET_STATUS, timeout=0.08), b"\x01")
        self.assertEqual(port.reset_calls, 0)

    def test_stream_parser(self) -> None:
        from pnyproto import read_frame
        for address in range(16):
            for cmd in range(16):
                frame = encode_frame(address, cmd, b"\xaa\x00")
                self.assertEqual(read_frame(io.BytesIO(frame), address, cmd, 0.1), b"\xaa\x00")
        bad = encode_frame(1, 1, b"bad")[:-1] + b"\x00"
        noise = b"\x00\x7f\xaa\x11\xff" + bad + encode_frame(2, 1, b"wrong address")
        self.assertEqual(read_frame(io.BytesIO(noise + encode_frame(1, 1, b"ok")), 1, 1, 0.1), b"ok")
        with self.assertRaises(TimeoutError):
            read_frame(io.BytesIO(bad), 1, 1, 0.001)

    def test_frame_roundtrip(self) -> None:
        payload = bytes(range(10))
        frame = encode_frame(3, CMD_GET_STATUS, payload)
        address, cmd, decoded = decode_frame(frame)
        self.assertEqual(address, 3)
        self.assertEqual(cmd, CMD_GET_STATUS)
        self.assertEqual(decoded, payload)

    def test_blob_pack_and_validate(self) -> None:
        affine_q20 = (100, 0, 0, 0, 100, 0)
        angle_lut = tuple(range(256))
        blob = build_blob(affine_q20, angle_lut, 1.25, 0.75, 12345, -1)
        self.assertEqual(len(blob), CAL_BLOB_SIZE)
        valid, crc32 = validate_blob(blob)
        self.assertTrue(valid)
        self.assertNotEqual(crc32, 0)
        self.assertEqual(int.from_bytes(blob[552:554], "little"), 12345)
        self.assertEqual(int.from_bytes(blob[554:555], "little", signed=True), -1)

    def test_blob_chunks_reassemble(self) -> None:
        data = bytes(range(256)) * 2 + bytes(range(128))
        chunks = list(iter_blob_chunks(data, chunk_size=48))
        rebuilt = bytearray()
        offset = 0
        for chunk_offset, chunk in chunks:
            self.assertEqual(chunk_offset, offset)
            rebuilt.extend(chunk)
            offset += len(chunk)
        self.assertEqual(bytes(rebuilt), data)

    def test_calibration_read_and_crc(self) -> None:
        blob = build_blob((100, 0, 0, 0, 100, 0), tuple(range(256)), 1.25, 0.75, 12345, 1)
        client = Stm32Client(None, 1)
        requests = []

        def exchange(command, payload):
            subcmd, offset, count = struct.unpack("<BHB", payload)
            self.assertEqual((command, subcmd), (4, 8))
            self.assertLessEqual(count, 63)
            requests.append(offset)
            return b"\0" + blob[offset:offset + count]

        client.exchange = exchange
        self.assertEqual(client.cal_read_blob(), blob)
        self.assertEqual(requests, list(range(0, 640, 63)))
        blob = blob[:-1] + bytes([blob[-1] ^ 1])
        with self.assertRaises(CalibrationError):
            client.cal_read_blob()
        client.exchange = lambda command, payload: b"\0"
        with self.assertRaises(CalibrationError):
            client.cal_read_blob()


if __name__ == "__main__":
    unittest.main()
