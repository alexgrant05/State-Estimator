#!/usr/bin/env bash
set -euo pipefail

run_as_root() {
    if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
        "$@"
    elif command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        echo "ERROR: $1 requires root privileges and sudo is unavailable." >&2
        exit 1
    fi
}

if [[ "${MSYSTEM:-}" == "UCRT64" ]]; then
    pacman -S --noconfirm --needed \
        mingw-w64-ucrt-x86_64-verilator \
        mingw-w64-ucrt-x86_64-gcc \
        mingw-w64-ucrt-x86_64-python \
        make
elif command -v apt-get >/dev/null 2>&1; then
    run_as_root apt-get update
    run_as_root apt-get install -y verilator g++ make python3
elif command -v dnf >/dev/null 2>&1; then
    run_as_root dnf install -y verilator gcc-c++ make python3
elif command -v pacman >/dev/null 2>&1; then
    run_as_root pacman -S --noconfirm --needed verilator gcc make python
elif command -v brew >/dev/null 2>&1; then
    brew install verilator python
else
    echo "ERROR: unsupported package manager. Install Verilator, a C++ compiler, GNU Make, and Python 3." >&2
    exit 1
fi

python_command=""
if command -v python3 >/dev/null 2>&1; then
    python_command="python3"
elif command -v python >/dev/null 2>&1; then
    python_command="python"
else
    echo "ERROR: Python was installed but is not available on PATH." >&2
    exit 1
fi

verilator --version
"$python_command" --version
echo "RTL tools are installed. Run: $python_command tools/run_rtl_tests.py"
