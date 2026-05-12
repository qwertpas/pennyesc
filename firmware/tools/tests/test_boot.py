import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pnyboot import (  # noqa: E402
    FLASH_APP_BYTES,
    FLASH_BASE,
    FLASH_PAGE_BYTES,
    BootError,
    check_connection,
    recover_image,
    Stm32Bootloader,
    load_image,
    read_boot_ack,
    send_enter_boot_frames,
    upload_image,
    wait_for_app_ready,
    xor_bytes,
)
from pnyproto import BOOT_MAGIC, CMD_ENTER_BOOT, RESULT_OK, decode_frame, encode_frame  # noqa: E402


class FakeSerial:
    def __init__(self, reads: bytes = b"") -> None:
        self._reads = bytearray(reads)
        self.writes = bytearray()
        self.is_open = True

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

    def readline(self) -> bytes:
        if not self._reads:
            return b""
        end = self._reads.find(b"\n")
        if end < 0:
            return self.read(len(self._reads))
        return self.read(end + 1)

    def reset_input_buffer(self) -> None:
        pass

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


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

    def test_check_connection_missing_port_fails_without_retry_sleep(self) -> None:
        with mock.patch("pnyboot.serial.Serial", side_effect=serial.SerialException("missing")):
            with mock.patch("pnyboot.time.sleep") as sleep:
                with self.assertRaises(serial.SerialException):
                    check_connection("/dev/missing", 3)
        sleep.assert_not_called()

    def test_check_connection_reports_bridge_shell_timeout(self) -> None:
        bridge = mock.Mock()
        bridge.enter_bridge.side_effect = BootError("bridge shell unavailable")
        context = mock.MagicMock()
        context.__enter__.return_value = bridge
        context.__exit__.return_value = None

        with mock.patch("pnyboot.BootBridge", return_value=context):
            with self.assertRaisesRegex(BootError, "bridge shell unavailable"):
                check_connection("/dev/test", 3)

    def test_check_connection_reports_missing_target(self) -> None:
        bridge = mock.Mock()
        context = mock.MagicMock()
        context.__enter__.return_value = bridge
        context.__exit__.return_value = None

        with mock.patch("pnyboot.BootBridge", return_value=context):
            with mock.patch("pnyboot.request_app_status", side_effect=BootError("target address 7 not found")):
                with self.assertRaisesRegex(BootError, "target address 7 not found"):
                    check_connection("/dev/test", 7)

    def test_upload_uses_app_path_only(self) -> None:
        boot = mock.Mock()
        bridge = mock.Mock()
        context = mock.MagicMock()
        context.__enter__.return_value = bridge
        context.__exit__.return_value = None

        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "fw.bin"
            image.write_bytes(b"\x01")
            with mock.patch("pnyboot.BootBridge", return_value=context):
                with mock.patch("pnyboot.open_bootloader_from_app", return_value=boot) as open_app:
                    with mock.patch("pnyboot.program_image") as program:
                        with mock.patch("pnyboot.wait_for_app_ready", return_value=b"\x00") as wait_ready:
                            with mock.patch("pnyboot.time.sleep") as sleep:
                                upload_image("/dev/test", image, 3)
        open_app.assert_called_once_with(bridge, 3)
        program.assert_called_once()
        boot.go.assert_called_once_with(FLASH_BASE)
        wait_ready.assert_called_once()
        sleep.assert_not_called()

    def test_recover_uses_reset_path(self) -> None:
        boot = mock.Mock()
        bridge = mock.Mock()
        context = mock.MagicMock()
        context.__enter__.return_value = bridge
        context.__exit__.return_value = None

        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "fw.bin"
            image.write_bytes(b"\x01")
            with mock.patch("pnyboot.BootBridge", return_value=context):
                with mock.patch("pnyboot.open_bootloader_from_reset", return_value=boot) as open_reset:
                    with mock.patch("pnyboot.open_bootloader_from_app") as open_app:
                        with mock.patch("pnyboot.program_image"):
                            with mock.patch("pnyboot.wait_for_app_ready", return_value=b"\x00"):
                                recover_image("/dev/test", image, 3)
        open_reset.assert_called_once_with(bridge)
        open_app.assert_not_called()
        boot.go.assert_called_once_with(FLASH_BASE)

    def test_wait_for_app_ready_has_no_initial_delay(self) -> None:
        bridge = mock.Mock()
        context = mock.MagicMock()
        context.__enter__.return_value = bridge
        context.__exit__.return_value = None

        with mock.patch("pnyboot.BootBridge", return_value=context):
            with mock.patch("pnyboot.request_app_status", return_value=b"\xAA") as request_status:
                with mock.patch("pnyboot.time.sleep") as sleep:
                    payload = wait_for_app_ready(
                        "/dev/test",
                        3,
                        timeout_s=2.0,
                        retry_delay=0.1,
                        shell_timeout=1.0,
                        allow_reset=False,
                        status_timeout=0.25,
                    )
        self.assertEqual(payload, b"\xAA")
        request_status.assert_called_once_with(bridge, 3, 0.25)
        sleep.assert_not_called()


if __name__ == "__main__":
    unittest.main()
