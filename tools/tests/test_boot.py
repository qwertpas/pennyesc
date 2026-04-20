import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pnyboot import (  # noqa: E402
    FLASH_APP_BYTES,
    FLASH_PAGE_BYTES,
    BootError,
    load_image,
    read_boot_ack,
    xor_bytes,
)
from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, decode_frame, encode_frame  # noqa: E402


class BootToolTests(unittest.TestCase):
    def test_xor_bytes(self) -> None:
        self.assertEqual(xor_bytes(bytes((0x01, 0x02, 0x03))), 0x00)

    def test_boot_frame_roundtrip(self) -> None:
        frame = encode_frame(0, CMD_ENTER_BOOT, BOOT_MAGIC.to_bytes(4, "little"))
        address, cmd, payload = decode_frame(frame)
        self.assertEqual(address, 0)
        self.assertEqual(cmd, CMD_ENTER_BOOT)
        self.assertEqual(int.from_bytes(payload, "little"), BOOT_MAGIC)

    def test_load_image_pads_to_page(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "image.bin"
            path.write_bytes(bytes(range(200)))
            image, padded, pages = load_image(path)
        self.assertEqual(len(image), 200)
        self.assertEqual(len(padded), 2 * FLASH_PAGE_BYTES)
        self.assertEqual(pages, [0, 1])
        self.assertTrue(padded.startswith(image))
        self.assertEqual(set(padded[len(image) :]), {0xFF})

    def test_load_image_rejects_oversize(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "image.bin"
            path.write_bytes(b"\x00" * (FLASH_APP_BYTES + 1))
            with self.assertRaises(RuntimeError):
                load_image(path)

    def test_read_boot_ack_ok(self) -> None:
        class FakePort:
            def __init__(self, frame: bytes) -> None:
                self.buf = bytearray(frame)

            def read(self, n: int = 1) -> bytes:
                if not self.buf:
                    return b""
                chunk = bytes(self.buf[:n])
                del self.buf[:n]
                return chunk

        frame = encode_frame(0, CMD_ENTER_BOOT, b"\x00")
        self.assertEqual(read_boot_ack(FakePort(frame), 0), 0)

    def test_read_boot_ack_rejects_bad_payload(self) -> None:
        class FakePort:
            def __init__(self, frame: bytes) -> None:
                self.buf = bytearray(frame)

            def read(self, n: int = 1) -> bytes:
                if not self.buf:
                    return b""
                chunk = bytes(self.buf[:n])
                del self.buf[:n]
                return chunk

        frame = encode_frame(0, CMD_ENTER_BOOT, b"")
        with self.assertRaises(BootError):
            read_boot_ack(FakePort(frame), 0)


if __name__ == "__main__":
    unittest.main()
