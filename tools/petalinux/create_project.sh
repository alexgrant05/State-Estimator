#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)

# shellcheck disable=SC1091
source "$script_dir/env.sh"

xsa_file="${1:-$repo_dir/vivado/export/state_estimator_kr260.xsa}"
project_name="${2:-state-estimator-linux}"
project_dir="$PETALINUX_PROJECT_ROOT/$project_name"
hardware_dir="$PETALINUX_PROJECT_ROOT/hardware"

if [[ ! -f "$xsa_file" ]]; then
  printf 'XSA not found: %s\nRun the Vivado build first.\n' "$xsa_file" >&2
  exit 1
fi

case "$project_dir" in
  /mnt/*)
    printf 'Project must be created in the WSL Linux filesystem.\n' >&2
    exit 1
    ;;
esac

if [[ -e "$project_dir" ]]; then
  printf 'Refusing to overwrite existing project: %s\n' "$project_dir" >&2
  exit 1
fi

mkdir -p "$hardware_dir"
install -m 0644 "$xsa_file" "$hardware_dir/state_estimator_kr260.xsa"

cd "$PETALINUX_PROJECT_ROOT"
petalinux-create project --template zynqMP --name "$project_name"
cd "$project_dir"
petalinux-config --get-hw-description "$hardware_dir" --silentconfig

printf '\nProject created and configured at %s\n' "$project_dir"
printf 'Build it with: cd %q && petalinux-build\n' "$project_dir"
