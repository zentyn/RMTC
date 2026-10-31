#!/usr/bin/env bash
# 按通信协议示例启动选手程序
# 用法: ./run_contestant.sh A "我的队伍" [AI网关地址]
# 可选环境变量: RMTC_TEAM_TOKEN / ROS_DOMAIN_ID / RMTC_AI_URL / RMTC_MODEL / RMTC_MAX_TOKENS
set -e

SEAT="${1:-A}"
TEAM_NAME="${2:-我的队伍}"
GATEWAY="${3:-${RMTC_AI_URL:-http://127.0.0.1:8080}}"
MODEL="${RMTC_MODEL:-deepseek-flash}"
MAX_TOKENS="${RMTC_MAX_TOKENS:-512}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 还没有编译，请先执行: cd $WS_DIR && colcon build --symlink-install"
  exit 1
fi
source "$WS_DIR/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMTC_TEAM_TOKEN="${RMTC_TEAM_TOKEN:-practice-$SEAT}"

echo "[启动] 席位=$SEAT 队名=$TEAM_NAME 网关=$GATEWAY 令牌=${RMTC_TEAM_TOKEN:0:8}... 域=$ROS_DOMAIN_ID 模型=$MODEL"

EXTRA_ARGS=()
if [ -n "$MODEL" ]; then
  EXTRA_ARGS+=(--model "$MODEL")
fi

exec ros2 run rmtc_bot rmtc_bot_node \
  --team "$SEAT" \
  --name "$TEAM_NAME" \
  --gateway "$GATEWAY" \
  --token "$RMTC_TEAM_TOKEN" \
  --max-tokens "$MAX_TOKENS" \
  --decision-config "$WS_DIR/install/rmtc_bot/share/rmtc_bot/config/decision_config.json" \
  "${EXTRA_ARGS[@]}"
