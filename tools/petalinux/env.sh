#!/usr/bin/env bash

# Source this file. Do not execute it.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'Use: source tools/petalinux/env.sh\n' >&2
  exit 2
fi

export PETALINUX_INSTALL_DIR="${PETALINUX_INSTALL_DIR:-$HOME/tools/petalinux/2026.1}"
export PETALINUX_PROJECT_ROOT="${PETALINUX_PROJECT_ROOT:-$HOME/work/petalinux}"

if [[ ! -f "$PETALINUX_INSTALL_DIR/settings.sh" ]]; then
  printf 'PetaLinux is not installed at %s\n' "$PETALINUX_INSTALL_DIR" >&2
  return 1
fi

# shellcheck disable=SC1090
source "$PETALINUX_INSTALL_DIR/settings.sh"
mkdir -p "$PETALINUX_PROJECT_ROOT"

printf 'PetaLinux: %s\nProjects:  %s\n' "$PETALINUX" "$PETALINUX_PROJECT_ROOT"
