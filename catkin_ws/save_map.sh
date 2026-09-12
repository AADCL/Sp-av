#!/usr/bin/env bash
# One-command entry point for the existing detached map-save client.
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 1 || ${1:-} == --help || ${1:-} == -h ]]; then
  echo "用法：./save_map.sh [地图名] | --status | --wait"
  echo "省略地图名时按时间命名；地图保存到本工作空间 maps/ 下。"
  echo "--status 查询上次任务，--wait 重新显示上次任务进度。"
  [[ $# -le 1 ]]
  exit
fi
if [[ ! -f "$workspace_dir/devel/setup.bash" ]]; then
  echo "找不到 devel/setup.bash，请先在本工作空间执行 catkin_make。" >&2
  exit 2
fi
source "$workspace_dir/devel/setup.bash"
client="$workspace_dir/src/ducted_bringup/scripts/save_map.py"
job_root="$workspace_dir/logs/map-save-jobs"
case "${1:-}" in
  --status) exec python3 "$client" status --log-root "$job_root" ;;
  --wait) exec python3 "$client" wait --log-root "$job_root" ;;
esac
map_name="${1:-map_$(date +%Y%m%d_%H%M%S_%N)}"
case "$map_name" in
  .|..|-*|*/*|*\\*) echo "地图名必须是一个目录名，不能包含路径或选项。" >&2; exit 2 ;;
esac
echo "保存目录：$workspace_dir/maps/$map_name"
echo "请保持基础层和建图运行、机体静止。Ctrl+C 只退出进度显示。"
exec python3 "$client" start --destination "$workspace_dir/maps/$map_name" \
  --log-root "$job_root" --wait
