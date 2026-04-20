Import("env")

from pathlib import Path
from SCons.Script import ARGUMENTS


def project_option(name, default):
    try:
        return env.GetProjectOption(name)
    except Exception:
        return default


project_dir = Path(env["PROJECT_DIR"])
python = env.subst("$PYTHONEXE")
image = env.subst("$BUILD_DIR/${PROGNAME}.bin")
tool = project_dir.parent.parent / "tools" / "pnyboot.py"
bridge_port = ARGUMENTS.get("BRIDGE_PORT", project_option("custom_bridge_port", "/dev/cu.usbmodem101"))
address = ARGUMENTS.get("ESC_ADDRESS", project_option("custom_esc_address", "0"))


env.AddCustomTarget(
    name="uart_upload",
    dependencies=[image],
    actions=[[python, str(tool), "upload", "--port", bridge_port, "--address", address, "--image", image]],
    title="UART Upload",
    description="Upload firmware through the ESP32 bridge",
)

env.AddCustomTarget(
    name="uart_recover",
    dependencies=[image],
    actions=[[python, str(tool), "recover", "--port", bridge_port, "--address", address, "--image", image]],
    title="UART Recover",
    description="Recover firmware through the ESP32 bridge after reset",
)
