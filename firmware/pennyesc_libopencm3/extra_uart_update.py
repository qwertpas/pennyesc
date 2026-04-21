Import("env")

import subprocess
from pathlib import Path
import os

from SCons.Script import ARGUMENTS


APP_OFFSET = 0x580


def project_option(name, default):
    try:
        return env.GetProjectOption(name)
    except Exception:
        return default


def env_or_project(env_name, project_name, default):
    value = os.environ.get(env_name)
    if value:
        return value
    return project_option(project_name, default)


def run_checked(cmd, cwd, extra_env=None):
    child_env = os.environ.copy()
    child_env["PYTHONUNBUFFERED"] = "1"
    if extra_env:
        child_env.update(extra_env)
    result = subprocess.run(cmd, cwd=cwd, env=child_env, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"command failed: {' '.join(str(part) for part in cmd)}")


def build_env(env_name, extra_env=None, targets=None):
    cmd = list(pio)
    cmd.extend(["-e", env_name])
    if targets:
        for target in targets:
            cmd.extend(["-t", target])
    run_checked(cmd, project_dir, extra_env=extra_env)


def build_merged_seed(seed_path, boot_path, app_path):
    seed_path.parent.mkdir(parents=True, exist_ok=True)
    boot = boot_path.read_bytes()
    app = app_path.read_bytes()

    if len(boot) > APP_OFFSET:
        raise RuntimeError(f"boot image is too large: {len(boot)} > {APP_OFFSET}")

    merged = bytearray(b"\xFF" * APP_OFFSET)
    merged[: len(boot)] = boot
    merged.extend(app)
    seed_path.write_bytes(merged)


def ensure_binary(image_path, elf_path):
    if image_path.exists():
        return

    platform = env.PioPlatform()
    toolchain_dir = Path(platform.get_package_dir("toolchain-gccarmnoneeabi"))
    objcopy = toolchain_dir / "bin" / "arm-none-eabi-objcopy"
    run_checked([str(objcopy), "-O", "binary", str(elf_path), str(image_path)], project_dir)


def openocd_seed_args(openocd_bin, scripts_dir, image):
    return [
        [
            str(openocd_bin),
            "-s",
            str(scripts_dir),
            "-f",
            "interface/stlink.cfg",
            "-c",
            "transport select hla_swd",
            "-f",
            "target/stm32l0.cfg",
            "-c",
            f"program {{{image}}} 0x08000000 verify reset; shutdown;",
        ],
        [
            str(openocd_bin),
            "-s",
            str(scripts_dir),
            "-f",
            "interface/stlink.cfg",
            "-c",
            "transport select hla_swd",
            "-f",
            "target/stm32l0.cfg",
            "-c",
            "adapter speed 50",
            "-c",
            f"program {{{image}}} 0x08000000 verify reset; shutdown;",
        ],
        [
            str(openocd_bin),
            "-s",
            str(scripts_dir),
            "-f",
            "interface/stlink.cfg",
            "-c",
            "transport select hla_swd",
            "-f",
            "target/stm32l0.cfg",
            "-c",
            "reset_config srst_nogate connect_assert_srst",
            "-c",
            "adapter speed 50",
            "-c",
            f"program {{{image}}} 0x08000000 verify reset; shutdown;",
        ],
    ]


def flash_seed_image(image):
    platform = env.PioPlatform()
    openocd_dir = Path(platform.get_package_dir("tool-openocd"))
    openocd_bin = openocd_dir / "bin" / "openocd"
    scripts_dir = openocd_dir / "openocd" / "scripts"
    last_error = None

    for cmd in openocd_seed_args(openocd_bin, scripts_dir, image):
        try:
            run_checked(cmd, project_dir)
            return
        except RuntimeError as exc:
            last_error = exc

    raise RuntimeError(f"seed upload failed after retries: {last_error}")


def seed_upload_action(source=None, target=None, env=None, **_kwargs):
    del source
    del target
    del env

    build_env(boot_env, targets=["buildprog"])
    ensure_binary(boot_image, boot_elf)
    boot_cache_image.parent.mkdir(parents=True, exist_ok=True)
    boot_cache_image.write_bytes(boot_image.read_bytes())

    build_env(app_env, extra_env={"PLATFORMIO_BUILD_FLAGS": f"-DPNY_ESC_ADDRESS={build_address}"})
    ensure_binary(app_image, app_elf)
    build_merged_seed(seed_image, boot_cache_image, app_image)
    flash_seed_image(seed_image)

    if ARGUMENTS.get("SEED_SKIP_PROBE", "0") == "1" or int(probe_rounds) <= 0:
        return

    print("seed_upload: probing UART boot path", flush=True)
    run_checked(
        [python, "-u", str(tool), "probe", "--port", bridge_port, "--address", build_address, "--rounds", probe_rounds],
        project_dir,
    )


def uart_upload_action(source=None, target=None, env=None, **_kwargs):
    del source
    del target
    del env

    build_env(app_env, extra_env={"PLATFORMIO_BUILD_FLAGS": f"-DPNY_ESC_ADDRESS={build_address}"})
    ensure_binary(app_image, app_elf)
    run_checked(
        [python, str(tool), "upload", "--port", bridge_port, "--address", target_address, "--image", app_image],
        project_dir,
    )


def uart_readdress_action(source=None, target=None, env=None, **_kwargs):
    del source
    del target
    del env

    if target_address == build_address:
        raise RuntimeError("readdress requires CURRENT_ESC_ADDRESS and NEW_ESC_ADDRESS to differ")

    uart_upload_action()


project_dir = Path(env["PROJECT_DIR"])
project_env = env["PIOENV"]
python = env.subst("$PYTHONEXE")
tool = project_dir.parent.parent / "tools" / "pnyboot.py"
app_env = env_or_project("APP_ENV", "custom_app_env", project_env)
boot_env = env_or_project("BOOT_ENV", "custom_boot_env", project_env)
bridge_port = ARGUMENTS.get("BRIDGE_PORT", env_or_project("BRIDGE_PORT", "custom_bridge_port", "/dev/cu.usbmodem101"))
address = ARGUMENTS.get("ESC_ADDRESS", env_or_project("ESC_ADDRESS", "custom_esc_address", "1"))
build_address = ARGUMENTS.get(
    "NEW_ESC_ADDRESS",
    env_or_project("NEW_ESC_ADDRESS", "custom_new_esc_address", address),
)
target_address = ARGUMENTS.get(
    "CURRENT_ESC_ADDRESS",
    env_or_project("CURRENT_ESC_ADDRESS", "custom_current_esc_address", build_address),
)
probe_rounds = str(
    ARGUMENTS.get(
        "SEED_PROBE_ROUNDS",
        env_or_project("SEED_PROBE_ROUNDS", "custom_seed_probe_rounds", "0"),
    )
)
pio = [python, "-m", "platformio", "run", "-d", str(project_dir)]
boot_image = project_dir / ".pio" / "build" / boot_env / "firmware.bin"
boot_elf = project_dir / ".pio" / "build" / boot_env / "firmware.elf"
app_image = project_dir / ".pio" / "build" / app_env / "firmware.bin"
app_elf = project_dir / ".pio" / "build" / app_env / "firmware.elf"
boot_cache_image = project_dir / ".pio" / "seed-boot.bin"
seed_image = project_dir / ".pio" / "seed-combined.bin"


env.AddCustomTarget(
    name="uart_upload",
    dependencies=[],
    actions=[uart_upload_action],
    title="UART Upload",
    description="Upload firmware through the ESP32 bridge",
)

env.AddCustomTarget(
    name="uart_readdress",
    dependencies=[],
    actions=[uart_readdress_action],
    title="UART Readdress",
    description="Upload firmware built for a new address to a device at its current address",
)

env.AddCustomTarget(
    name="seed_upload",
    dependencies=[],
    actions=[seed_upload_action],
    title="Seed UART Bootloader",
    description="Flash one combined STM32 image over SWD",
)
