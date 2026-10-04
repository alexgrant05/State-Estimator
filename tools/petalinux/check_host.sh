#!/usr/bin/env bash
set -euo pipefail

failures=0

pass() {
  printf 'PASS: %s\n' "$1"
}

fail() {
  printf 'FAIL: %s\n' "$1" >&2
  failures=$((failures + 1))
}

if grep -qi microsoft /proc/sys/kernel/osrelease; then
  pass "running under WSL"
else
  fail "this setup is intended for WSL2"
fi

if [[ "$(readlink -f /bin/sh)" == "/usr/bin/bash" ]]; then
  pass "/bin/sh resolves to Bash"
else
  fail "/bin/sh must resolve to Bash"
fi

cores=$(nproc)
if (( cores >= 8 )); then
  pass "$cores CPU cores available"
else
  fail "AMD requires at least 8 CPU cores; found $cores"
fi

memory_kib=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
if (( memory_kib >= 8 * 1024 * 1024 )); then
  pass "$((memory_kib / 1024 / 1024)) GiB RAM available"
else
  fail "AMD requires at least 8 GiB RAM"
fi

home_fs=$(findmnt -no FSTYPE /)
if [[ "$home_fs" == "ext4" ]]; then
  pass "Linux home is on ext4"
else
  fail "PetaLinux tools and projects must use the WSL Linux filesystem, found $home_fs"
fi

free_gib=$(df -Pk "$HOME" | awk 'NR == 2 {print int($4 / 1024 / 1024)}')
if (( free_gib >= 100 )); then
  pass "$free_gib GiB free in the Linux filesystem"
else
  fail "AMD requires at least 100 GiB free; found $free_gib GiB"
fi

required_commands=(
  awk autoconf automake bash bison chrpath cpio diffstat file flex g++ gawk
  gcc git make patch python3 rsync socat tar texi2any unzip wget xz zstd
)
for command_name in "${required_commands[@]}"; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    fail "missing prerequisite command: $command_name"
  fi
done

if (( failures != 0 )); then
  printf '\nHost check failed with %d issue(s).\n' "$failures" >&2
  exit 1
fi

printf '\nHost is ready for the PetaLinux 2026.1 installer.\n'
