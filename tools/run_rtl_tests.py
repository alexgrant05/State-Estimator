#!/usr/bin/env python3
"""Compile and run each SystemVerilog testbench with Verilator."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
RTL_ROOT = ROOT / "src" / "rtl"
TB_ROOT = ROOT / "src" / "tb"
BUILD_ROOT = Path(
    os.environ.get(
        "RTL_BUILD_ROOT",
        str(Path(tempfile.gettempdir()) / "cornell_state_estimation_rtl")
        if os.name == "nt"
        else str(ROOT / ".rtl_build"),
    )
)


def verilator_command() -> tuple[list[str], dict[str, str]]:
    environment = os.environ.copy()
    configured = environment.get("VERILATOR")
    if configured:
        return [configured], environment
    executable = shutil.which("verilator")
    if executable:
        return [executable], environment
    if os.name == "nt":
        msys_root = Path(environment.get("MSYS2_ROOT", r"C:\msys64"))
        binary = msys_root / "ucrt64" / "bin" / "verilator_bin.exe"
        if binary.exists():
            environment["PATH"] = os.pathsep.join(
                [
                    str(msys_root / "ucrt64" / "bin"),
                    str(msys_root / "usr" / "bin"),
                    environment.get("PATH", ""),
                ]
            )
            environment["VERILATOR_ROOT"] = str(
                msys_root / "ucrt64" / "share" / "verilator"
            )
            return [str(binary)], environment
    raise RuntimeError(
        "Verilator was not found. Run 'bash tools/install_rtl_tools.sh', "
        "install Verilator with your package manager, or set VERILATOR."
    )


def discover(pattern: str | None) -> list[Path]:
    benches = sorted(TB_ROOT.rglob("tb_*.sv"))
    if pattern:
        benches = [path for path in benches if pattern.lower() in path.stem.lower()]
    if not benches:
        raise RuntimeError("no matching SystemVerilog testbenches were found")
    return benches


def run_command(
    command: list[str], environment: dict[str, str], cwd: Path = ROOT
) -> None:
    print("+", subprocess.list2cmdline(command), flush=True)
    subprocess.run(command, cwd=cwd, env=environment, check=True)


def tool_path(path: Path) -> str:
    """Use forward slashes so generated C++ locations are escaped correctly."""

    return path.resolve().as_posix()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", help="run testbenches whose name contains this text")
    parser.add_argument("--list", action="store_true", help="list testbenches and exit")
    parser.add_argument(
        "--lint-only",
        action="store_true",
        help="lint synthesizable RTL without running testbenches",
    )
    parser.add_argument("--trace", action="store_true", help="enable VCD tracing")
    arguments = parser.parse_args()

    rtl_sources = sorted(RTL_ROOT.rglob("*.sv")) + sorted(RTL_ROOT.rglob("*.v"))
    if not rtl_sources:
        raise RuntimeError("no RTL sources were found")
    benches = discover(arguments.test) if not arguments.lint_only else []
    if arguments.list:
        for bench in benches:
            print(bench.relative_to(ROOT))
        return 0

    verilator, environment = verilator_command()
    common_options = [
        "--language",
        "1800-2017",
        "--assert",
        "--Wall",
        "--Wno-MULTITOP",
    ]
    if arguments.lint_only:
        run_command(
            verilator
            + common_options
            + ["--lint-only", *map(tool_path, rtl_sources)],
            environment,
        )
        print(f"PASS linted {len(rtl_sources)} RTL source files")
        return 0

    passed = 0
    for bench in benches:
        top = bench.stem
        build_directory = BUILD_ROOT / top
        build_directory.mkdir(parents=True, exist_ok=True)
        output_name = f"{top}.exe" if os.name == "nt" else top
        command = verilator + common_options + [
            "--binary",
            "--timing",
            "-j",
            "0",
            "--top-module",
            top,
            "--Mdir",
            tool_path(build_directory),
            "-o",
            output_name,
        ]
        if arguments.trace:
            command.extend(["--trace", "-DTRACE"])
        command.extend(map(tool_path, rtl_sources))
        command.append(tool_path(bench))
        run_command(command, environment)
        run_command([str(build_directory / output_name)], environment, build_directory)
        passed += 1

    print(f"PASS {passed} Verilator module tests")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
