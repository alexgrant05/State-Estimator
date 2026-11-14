#!/usr/bin/env bash
set -euo pipefail

readonly version="2026.1"
readonly expected_md5="084e9f6b58ca45ffdd965e407a98931c"
readonly install_dir="${PETALINUX_INSTALL_DIR:-$HOME/tools/petalinux/$version}"
readonly cache_dir="$HOME/.cache/petalinux/installers"

if (( EUID == 0 )); then
  printf 'Do not run the PetaLinux installer as root.\n' >&2
  exit 1
fi

if (( $# != 1 )); then
  printf 'Usage: %s /path/to/petalinux-v2026.1-final-installer.run\n' "$0" >&2
  exit 2
fi

installer=$(readlink -f "$1")
if [[ ! -f "$installer" ]]; then
  printf 'Installer not found: %s\n' "$1" >&2
  exit 1
fi

case "$install_dir" in
  /mnt/*)
    printf 'Install into the WSL Linux filesystem, not /mnt: %s\n' "$install_dir" >&2
    exit 1
    ;;
esac

mkdir -p "$cache_dir" "$install_dir"
if [[ -n "$(find "$install_dir" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  printf 'Install directory is not empty: %s\n' "$install_dir" >&2
  exit 1
fi

staged_installer="$cache_dir/$(basename "$installer")"
if [[ "$installer" != "$staged_installer" ]]; then
  printf 'Copying installer into the WSL Linux filesystem...\n'
  cp "$installer" "$staged_installer"
fi

actual_md5=$(md5sum "$staged_installer" | awk '{print $1}')
if [[ "$actual_md5" != "$expected_md5" ]]; then
  printf 'Installer checksum mismatch.\nExpected: %s\nActual:   %s\n' \
    "$expected_md5" "$actual_md5" >&2
  exit 1
fi

chmod u+x "$staged_installer"
printf 'Checksum verified. Accept the AMD licenses when prompted.\n'
# Launch from an ext4 path without spaces. The AMD installer derives its
# default log path from the working directory and does not quote that path.
cd "$cache_dir"
"$staged_installer" --dir "$install_dir" --platform aarch64 \
  --log "$cache_dir/install-$version.log"

test -f "$install_dir/settings.sh"
test -x "$install_dir/scripts/petalinux-create"
printf '\nPetaLinux %s installed at %s\n' "$version" "$install_dir"
