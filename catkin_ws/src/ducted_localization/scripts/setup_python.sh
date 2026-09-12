#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
environment_dir="$workspace_dir/.venv-localization"
mkdir -p "$workspace_dir/logs/localization_bootstrap"
if ! "$environment_dir/bin/python3" -m pip --version >/dev/null 2>&1; then
  if ! python3 -m venv --system-site-packages "$environment_dir"; then
    python3 -m pip install --target "$workspace_dir/logs/localization_bootstrap" virtualenv
    PYTHONPATH="$workspace_dir/logs/localization_bootstrap" python3 -m virtualenv --system-site-packages "$environment_dir"
  fi
fi
"$environment_dir/bin/python3" -m pip install -r "$package_dir/requirements.txt"
"$environment_dir/bin/python3" -c 'import open3d; print("Open3D", open3d.__version__)'
