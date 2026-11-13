#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)

# shellcheck disable=SC1091
source "$script_dir/env.sh"

project_dir="${1:-$PETALINUX_PROJECT_ROOT/state-estimator-linux}"
output_dir="${2:-$repo_dir/outputs/kr260-sd-boot}"
image_dir="$project_dir/images/linux"

for input_file in image.ub boot.scr system.bit system.dtb; do
  if [[ ! -f "$image_dir/$input_file" ]]; then
    printf 'Missing PetaLinux output: %s\n' "$image_dir/$input_file" >&2
    exit 1
  fi
done

install -d "$output_dir"
install -m 0644 "$image_dir/image.ub" "$output_dir/image.ub"
install -m 0644 "$image_dir/boot.scr" "$output_dir/boot.scr"
install -m 0644 "$image_dir/system.bit" "$output_dir/system.bit"
install -m 0644 "$image_dir/system.dtb" "$output_dir/system.dtb"
install -m 0644 "$repo_dir/linux/petalinux/uEnv.txt" "$output_dir/uEnv.txt"

(
  cd "$output_dir"
  sha256sum image.ub boot.scr system.bit system.dtb uEnv.txt > SHA256SUMS
)

printf 'Staged KR260 SD boot files at %s\n' "$output_dir"
