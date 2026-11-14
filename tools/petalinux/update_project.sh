#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)

# shellcheck disable=SC1091
source "$script_dir/env.sh"

project_dir="${1:-$PETALINUX_PROJECT_ROOT/state-estimator-linux}"
hardware_dir="$PETALINUX_PROJECT_ROOT/hardware"
xsa_file="$repo_dir/vivado/export/state_estimator_kr260.xsa"

if [[ ! -f "$xsa_file" ]]; then
  printf 'Missing Vivado hardware export: %s\n' "$xsa_file" >&2
  exit 1
fi
if [[ ! -d "$project_dir/project-spec" ]]; then
  printf 'Missing PetaLinux project: %s\n' "$project_dir" >&2
  exit 1
fi

install -d "$hardware_dir"
install -m 0644 "$xsa_file" "$hardware_dir/state_estimator_kr260.xsa"

cd "$project_dir"
petalinux-config --get-hw-description "$hardware_dir" --silentconfig
install -m 0644 \
  "$repo_dir/linux/petalinux/system-user.dtsi" \
  project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi

firmware_recipe=project-spec/meta-user/recipes-firmware/state-estimator-pl
autoload_recipe=project-spec/meta-user/recipes-apps/state-estimator-pl-autoload
install -d "$firmware_recipe/files" "$autoload_recipe/files"
install -m 0644 \
  "$repo_dir/linux/petalinux/meta-user/recipes-firmware/state-estimator-pl/state-estimator-pl.bb" \
  "$firmware_recipe/state-estimator-pl.bb"
install -m 0644 "$xsa_file" \
  "$firmware_recipe/files/state_estimator_kr260.xsa"
install -m 0644 \
  "$repo_dir/linux/petalinux/meta-user/recipes-apps/state-estimator-pl-autoload/state-estimator-pl-autoload.bb" \
  "$autoload_recipe/state-estimator-pl-autoload.bb"
install -m 0644 \
  "$repo_dir/linux/petalinux/meta-user/recipes-apps/state-estimator-pl-autoload/files/state-estimator-pl-load.service" \
  "$autoload_recipe/files/state-estimator-pl-load.service"
install -m 0755 \
  "$repo_dir/linux/petalinux/meta-user/recipes-apps/state-estimator-pl-autoload/files/state-estimator-pl-load.sh" \
  "$autoload_recipe/files/state-estimator-pl-load.sh"

if ! grep -q '^# State-estimator PL firmware' \
  project-spec/meta-user/conf/petalinuxbsp.conf; then
  cat >> project-spec/meta-user/conf/petalinuxbsp.conf <<'EOF'

# State-estimator PL firmware and verified boot-time loader.
MACHINE_FEATURES:append = " fpga-overlay"
IMAGE_INSTALL:append = " state-estimator-pl state-estimator-pl-autoload fpga-manager-script"
EOF
fi

printf 'Updated PetaLinux project from %s\n' "$xsa_file"
