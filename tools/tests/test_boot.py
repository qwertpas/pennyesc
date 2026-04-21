import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pnyboot import (  # noqa: E402
    FLASH_APP_BYTES,
    FLASH_BASE,
    FLASH_PAGE_BYTES,
    BootError,
    Stm32Bootloader,
    load_image,
    read_boot_ack,
    send_enter_boot_frames,
    xor_bytes,
)
from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, RESULT_OK, decode_frame, encode_frame  # noqa: E402


class FakeSerial:
    def __init__(self, reads: bytes = b"") -> None:
        self._reads = bytearray(reads)
        self.writes = bytearray()

    def read(self, count: int = 1) -> bytes:
        if not self._reads:
            return b""
        chunk = self._reads[:count]
        del self._reads[:count]
        return bytes(chunk)

    def write(self, data: bytes) -> int:
        self.writes.extend(data)
        return len(data)

    def flush(self) -> None:
        pass


class BootToolTests(unittest.TestCase):
    def test_xor_bytes(self) -> None:
        self.assertEqual(xor_bytes(bytes((0x01, 0x02, 0x03))), 0x00)

    def test_boot_frame_roundtrip(self) -> None:
        frame = encode_frame(0, CMD_ENTER_BOOT, BOOT_MAGIC.to_bytes(4, "little"))
        address, cmd, payload = decode_frame(frame)
        self.assertEqual(address, 0)
        self.assertEqual(cmd, CMD_ENTER_BOOT)
        self.assertEqual(int.from_bytes(payload, "little"), BOOT_MAGIC)

    def test_send_enter_boot_frames(self) -> None:
        fake = FakeSerial()
        send_enter_boot_frames(fake, 3, count=2)
        frame = encode_frame(3, CMD_ENTER_BOOT, BOOT_MAGIC.to_bytes(4, "little"))
        self.assertEqual(fake.writes, frame + frame)

    def test_read_boot_ack(self) -> None:
        frame = encode_frame(2, CMD_ENTER_BOOT, bytes((RESULT_OK,)))
        fake = FakeSerial(frame)
        self.assertTrue(read_boot_ack(fake, 2, timeout=0.01))

    def test_go_tolerates_truncated_final_ack(self) -> None:
        fake = FakeSerial(bytes((0x79, 0xF9)))
        boot = Stm32Bootloader(fake)
        boot.go(FLASH_BASE)
        self.assertEqual(
            fake.writes,
            bytes((0x21, 0xDE, 0x08, 0x00, 0x05, 0x80, 0x8D)),
        )

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


if __name__ == "__main__":
    unittest.main()
