#!/usr/bin/env bash
# 在容器內執行：bash run_mtc_tests.sh
# 前提：另一個終端機已啟動 `ros2 launch moveit_task_constructor_demo demo.launch.py`
# 每個案例結束後，手臂會回到 ready 姿態，下一個案例會重設物體位置。
# 若某個案例在執行途中失敗（exec=fail），手臂或物體可能停在中途，請重啟 demo.launch.py 再繼續。
set -u

# setup.bash 內部會用到未定義的變數，source 時暫時關掉 set -u
set +u
source /root/ros2_ws/install/setup.bash
set -u
LOG_DIR="mtc_test_logs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/summary.md"
TIMEOUT_SEC=180

# 格式：名稱|launch 參數|預期（這些預期是推測，以實測結果為準）
CASES=(
  "m2_default||預期成功（預設參數，已手動驗證過）"
  "m3a_object_near|object_x:=0.4 object_y:=-0.2|不確定，觀察"
  "m3b_object_far|object_x:=1.0 object_y:=-0.25|預期規劃失敗（物體超出手臂可達範圍）"
  "m4a_place_y_0.3|place_y:=0.3|不確定，觀察"
  "m4b_place_y_1.0|place_y:=1.0|預期規劃失敗（放置點超出可達範圍）"
)

{
  echo "| 案例 | 參數 | 預期 | 規劃 | 規劃耗時（秒） | 執行 |"
  echo "| --- | --- | --- | --- | --- | --- |"
} > "$SUMMARY"

for c in "${CASES[@]}"; do
  IFS='|' read -r name args expect <<< "$c"
  log="$LOG_DIR/$name.log"
  echo "==> $name  (${args:-預設參數})"

  # shellcheck disable=SC2086
  timeout "$TIMEOUT_SEC" ros2 launch mtc_tutorial pick_place_demo.launch.py \
    exit_after_task:=true $args 2>&1 | tee "$log"

  # 去掉終端機顏色碼後，取最後一行 RESULT
  line=$(sed 's/\x1b\[[0-9;]*m//g' "$log" | grep -o "RESULT plan=[a-z]* plan_time=[0-9.]* exec=[a-z]*" | tail -1)
  if [ -n "$line" ]; then
    plan=$(echo "$line" | sed -E 's/.*plan=([a-z]+) .*/\1/')
    ptime=$(echo "$line" | sed -E 's/.*plan_time=([0-9.]+) .*/\1/')
    exec_r=$(echo "$line" | sed -E 's/.*exec=([a-z]+).*/\1/')
  else
    plan="無結果"; ptime="-"; exec_r="無結果（逾時或程式中斷）"
  fi

  echo "| $name | \`${args:-預設}\` | $expect | $plan | $ptime | $exec_r |" >> "$SUMMARY"
  sleep 3
done

echo
echo "完成，摘要（也存在 $SUMMARY）："
cat "$SUMMARY"