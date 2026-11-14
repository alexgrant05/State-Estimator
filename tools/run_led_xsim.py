#!/usr/bin/env python3
"""Run LED reset and reset-free startup tests in Vivado's four-state simulator."""

from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    executables = {}
    for name in ("xvlog", "xelab", "xsim"):
        executable = shutil.which(name)
        if executable is None:
            raise RuntimeError(f"{name} not found; add Vivado bin to PATH")
        executables[name] = executable

    for top in ("tb_led_blinker", "tb_led_blinker_startup"):
        directory = ROOT / ".rtl_build" / "led_xsim" / top
        directory.mkdir(parents=True, exist_ok=True)
        commands = (
            [executables["xvlog"], "--sv",
             str(ROOT / "src/rtl/bringup/led_blinker.sv"),
             str(ROOT / "src/tb/bringup" / f"{top}.sv")],
            [executables["xelab"], top, "--snapshot", top],
            [executables["xsim"], top, "--runall"],
        )
        for command in commands:
            result = subprocess.run(
                command, cwd=directory, capture_output=True, text=True,
                errors="replace", check=False,
            )
            output = result.stdout + result.stderr
            if result.returncode or "FATAL" in output.upper() or "ERROR:" in output.upper():
                raise RuntimeError(f"{Path(command[0]).name} failed:\n{output}")
        # XSim can finish after a testbench failure without a nonzero exit code.
        if f"PASS {top}" not in output:
            raise RuntimeError(f"Missing test completion marker:\n{output}")
        print(f"PASS {top} (four-state XSim)", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
