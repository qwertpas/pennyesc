import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pennycal import (  # noqa: E402
    CAL_BLOB_SIZE,
    CMD_GET_STATUS,
    build_blob,
    decode_frame,
    encode_frame,
    iter_blob_chunks,
    validate_blob,
)


class ProtocolTests(unittest.TestCase):
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
        blob = build_blob(affine_q20, angle_lut, 1.25, 0.75)
        self.assertEqual(len(blob), CAL_BLOB_SIZE)
        valid, crc32 = validate_blob(blob)
        self.assertTrue(valid)
        self.assertNotEqual(crc32, 0)

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


if __name__ == "__main__":
    unittest.main()
