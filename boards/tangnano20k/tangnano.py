#!/usr/bin/env python3
"""Test, build, load, and monitor the Tang Nano 20K bring-up design.

Subcommands:
  test     Verilator lint + self-checking testbenches for the board RTL
  build    Yosys (slang frontend) -> nextpnr-himbaechel -> gowin_pack
  load     write the bitstream to FPGA SRAM (lost on power cycle)
  flash    write the bitstream to on-board flash (survives power cycle)
  monitor  print what the board sends over the USB-serial UART

Requires the oss-cad-suite tools on PATH:  source ~/oss-cad-suite/environment
"""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


BOARD_ROOT = Path(__file__).resolve().parent
REPO_ROOT = BOARD_ROOT.parents[1]
BOARD_RTL = BOARD_ROOT / "rtl"
BOARD_TB = BOARD_ROOT / "tb"
SHARED_RTL = REPO_ROOT / "src" / "rtl"
CONSTRAINTS = BOARD_ROOT / "tangnano20k.cst"
BUILD_DIR = BOARD_ROOT / "build"
BITSTREAM = BUILD_DIR / "tangnano20k_top.fs"

TOP = "tangnano20k_top"
DEVICE = "GW2AR-LV18QN88C8/I7"
FAMILY = "GW2A-18C"
CLOCK_MHZ = "27"
BAUD = 115_200

# Verilator's --binary make step breaks on paths containing spaces (macOS
# gotcha in CONTEXT.md), so its build products default to the temp dir.
SIM_BUILD_ROOT = Path(
    os.environ.get(
        "RTL_BUILD_ROOT", str(Path(tempfile.gettempdir()) / "cornell_tangnano20k_rtl")
    )
)


def require(tool: str) -> str:
    path = shutil.which(tool)
    if not path:
        raise RuntimeError(
            f"'{tool}' was not found. Run 'source ~/oss-cad-suite/environment' first."
        )
    return path


def run_command(command: list[str], cwd: Path = REPO_ROOT, log: Path | None = None) -> None:
    print("+", subprocess.list2cmdline(command), flush=True)
    if log is None:
        subprocess.run(command, cwd=cwd, check=True)
        return
    with log.open("w") as handle:
        result = subprocess.run(command, cwd=cwd, stdout=handle, stderr=subprocess.STDOUT)
    if result.returncode != 0:
        print(log.read_text()[-4000:], file=sys.stderr)
        raise subprocess.CalledProcessError(result.returncode, command)


def rtl_sources() -> list[Path]:
    # Shared repo RTL is included so later steps can instantiate gnss_scc etc.
    return sorted(BOARD_RTL.glob("*.sv")) + sorted(SHARED_RTL.rglob("*.sv"))


def relative(path: Path) -> str:
    # Yosys's read_slang does not accept quoted paths, and the repo path may
    # contain spaces, so pass paths relative to the repo root instead.
    return path.relative_to(REPO_ROOT).as_posix()


def test(pattern: str | None) -> None:
    verilator = require("verilator")
    options = ["--language", "1800-2017", "--assert", "--Wall", "--Wno-MULTITOP"]
    sources = [p.resolve().as_posix() for p in rtl_sources()]

    run_command([verilator, *options, "--lint-only", *sources])

    benches = sorted(BOARD_TB.glob("tb_*.sv"))
    if pattern:
        benches = [b for b in benches if pattern.lower() in b.stem.lower()]
    if not benches:
        raise RuntimeError("no matching testbenches were found")
    for bench in benches:
        top = bench.stem
        build_directory = SIM_BUILD_ROOT / top
        build_directory.mkdir(parents=True, exist_ok=True)
        run_command(
            [
                verilator, *options, "--binary", "--timing", "-j", "0",
                "--top-module", top, "--Mdir", build_directory.as_posix(), "-o", top,
                *sources, bench.resolve().as_posix(),
            ]
        )
        run_command([str(build_directory / top)], cwd=build_directory)
    print(f"PASS {len(benches)} Tang Nano testbenches")


def build() -> None:
    yosys = require("yosys")
    nextpnr = require("nextpnr-himbaechel")
    gowin_pack = require("gowin_pack")
    BUILD_DIR.mkdir(exist_ok=True)
    synth_json = BUILD_DIR / "synth.json"
    pnr_json = BUILD_DIR / "pnr.json"

    files = " ".join(relative(p) for p in rtl_sources())
    script = (
        f"read_slang --top {TOP} {files}; "
        f"synth_gowin -top {TOP} -family gw2a -json {relative(synth_json)}"
    )
    run_command([yosys, "-m", "slang", "-p", script], log=BUILD_DIR / "yosys.log")
    run_command(
        [
            nextpnr, "--json", str(synth_json), "--write", str(pnr_json),
            "--device", DEVICE, "--vopt", f"family={FAMILY}",
            "--vopt", f"cst={CONSTRAINTS}", "--freq", CLOCK_MHZ,
        ],
        log=BUILD_DIR / "nextpnr.log",
    )
    # gowin_pack warns that optional speed-up modules (numpy etc.) are missing;
    # the output is identical without them, so hide that noise.
    os.environ["PYTHONWARNINGS"] = "ignore::UserWarning"
    run_command([gowin_pack, "-d", FAMILY, "-o", str(BITSTREAM), str(pnr_json)])

    # Summarize the two numbers that matter: does it fit, and does it meet timing.
    report = (BUILD_DIR / "nextpnr.log").read_text()
    utilisation = report.split("Info: Device utilisation:")[-1].split("\n\n")[0]
    print("Device utilisation:")
    for line in utilisation.splitlines():
        if re.search(r"(LUT4|DFF|BSRAM|IOB|ALU)\w*:\s+\d+/", line):
            print("  " + line.replace("Info:", "").strip())
    # nextpnr reports timing after placement and again after routing; the
    # post-route figure (the last one) is the real result.
    fmax_lines = [line for line in report.splitlines() if "Max frequency for clock" in line]
    if fmax_lines:
        print("  " + fmax_lines[-1].replace("Info:", "").strip())
    print(f"Bitstream: {BITSTREAM}")


def program(to_flash: bool) -> None:
    loader = require("openFPGALoader")
    if not BITSTREAM.exists():
        raise RuntimeError(f"{BITSTREAM} does not exist; run 'build' first")
    command = [loader, "-b", "tangnano20k"]
    if to_flash:
        command.append("-f")
    run_command(command + [str(BITSTREAM)])


def monitor(port: str | None) -> None:
    import termios  # POSIX only; avoids depending on pyserial

    if port is None:
        ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/ttyUSB*"))
        if not ports:
            raise RuntimeError("no USB serial port found; is the board plugged in?")
        # The on-board bridge exposes two ports (JTAG, then UART); take the last.
        port = ports[-1]
    print(f"monitoring {port} at {BAUD} baud (Ctrl-C to stop)", flush=True)

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
    try:
        attributes = termios.tcgetattr(fd)
        attributes[0] = 0                                        # iflag: raw
        attributes[1] = 0                                        # oflag: raw
        attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1
        attributes[3] = 0                                        # lflag: raw
        attributes[4] = attributes[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        while True:
            data = os.read(fd, 256)
            sys.stdout.write(data.decode("ascii", errors="replace"))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subcommands = parser.add_subparsers(dest="command", required=True)
    test_parser = subcommands.add_parser("test", help="lint and run board testbenches")
    test_parser.add_argument("--test", help="run testbenches whose name contains this text")
    subcommands.add_parser("build", help="synthesize, place and route, and pack")
    subcommands.add_parser("load", help="load the bitstream into SRAM (volatile)")
    subcommands.add_parser("flash", help="write the bitstream to flash (persistent)")
    monitor_parser = subcommands.add_parser("monitor", help="print UART output")
    monitor_parser.add_argument("--port", help="serial device (default: auto-detect)")
    arguments = parser.parse_args()

    if arguments.command == "test":
        test(arguments.test)
    elif arguments.command == "build":
        build()
    elif arguments.command in ("load", "flash"):
        program(arguments.command == "flash")
    else:
        monitor(arguments.port)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
