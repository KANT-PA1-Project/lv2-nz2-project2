#!/usr/bin/env bash
# NZ2 실험 대기열 실행기 (개발 PC, 터미널 하나에 띄워 두기)
#   Claude(클라우드 세션)가 agent/queue/*.job 파일을 써 넣으면 순서대로 expert-check 를 돌리고
#   로그·결과 json 을 agent/results/ 에 남긴다. Claude 는 연결된 폴더로 결과를 읽고 코드를 고친 뒤 다음 작업을 넣는다.
#
# 실행:  bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_agent_runner.sh
# 중지:  Ctrl+C  (또는 agent/STOP 파일 생성)
#
# 안전장치 (임의 명령 실행 방지)
#   - 작업 파일은 'KEY=VALUE' 줄만 허용. KEY 는 NZ2_[A-Z0-9_]+ 또는 MODE / STAGE / NOISE 만
#   - VALUE 는 [A-Za-z0-9._,;:+-] 문자만 (공백·따옴표·$·`·| 등 금지), 셸로 평가하지 않음 (env 배열로 전달)
#   - MODE 는 expert-check / percep-dump(영상 저장 → agent/percep/<작업이름>/) 만 허용, STAGE 0~3, NOISE 0/1
#   - 작업당 시간 제한 TIMEOUT_S (기본 900 s), 한 번에 하나씩
set -uo pipefail
ROOT=~/manipulator-isaac61-lyrical-practice
R=$ROOT/scripts/nz2_vispick_run.sh
A=$ROOT/agent
LOGDIR=~/pa-course/IsaacLab-6.1/logs/nz2_vispick
TIMEOUT_S="${TIMEOUT_S:-900}"
RE='^([A-Z][A-Z0-9_]*)=([A-Za-z0-9._,;:+-]*)$'
mkdir -p "$A/queue" "$A/results" "$A/done" "$A/bad"
rm -f "$A/STOP"
echo "[runner] 대기열 감시 시작: $A/queue (중지: Ctrl+C 또는 touch $A/STOP)"
echo "[runner] $(date '+%F %T') 시작" >> "$A/runner.log"

run_job() {
  local job="$1" name; name=$(basename "$job" .job)
  local -a envs=(); local mode="" stage="3" noise="0" ok=1
  while IFS= read -r line || [ -n "$line" ]; do
    line="${line%%#*}"; line="${line//[$'\r\t ']/}"
    [ -z "$line" ] && continue
    if [[ ! "$line" =~ $RE ]]; then echo "[runner] 거부(형식): $line"; ok=0; break; fi
    local k="${BASH_REMATCH[1]}" v="${BASH_REMATCH[2]}"
    case "$k" in
      MODE) mode="$v" ;;
      STAGE) [[ "$v" =~ ^[0-3]$ ]] || { echo "[runner] 거부(STAGE): $v"; ok=0; break; }; stage="$v" ;;
      NOISE) [[ "$v" =~ ^[01]$ ]] || { echo "[runner] 거부(NOISE): $v"; ok=0; break; }; noise="$v" ;;
      NZ2_*) envs+=("$k=$v") ;;
      *) echo "[runner] 거부(키): $k"; ok=0; break ;;
    esac
  done < "$job"
  [ "$mode" = "expert-check" ] || [ "$mode" = "percep-dump" ] || { [ $ok = 1 ] && echo "[runner] 거부(MODE): '$mode'"; ok=0; }
  if [ $ok = 0 ]; then mv "$job" "$A/bad/"; echo "[runner] $(date '+%T') $name 거부" >> "$A/runner.log"; return; fi
  local log="$A/results/$name.log" t0; t0=$(date +%s)
  echo "[runner] $(date '+%T') 실행 $name: ${envs[*]} expert-check $stage $noise"
  echo "[runner] $(date '+%T') 실행 $name: ${envs[*]} expert-check $stage $noise" >> "$A/runner.log"
  local before; before=$(ls -t "$LOGDIR"/expert_check_*.json 2>/dev/null | head -1)
  if [ "$mode" = "percep-dump" ]; then
    timeout "$TIMEOUT_S" env "${envs[@]}" NZ2_PERCEP_DUMP="$name" bash "$ROOT/scripts/nz2_percep_run.sh" "$stage" > "$log" 2>&1
  else
    timeout "$TIMEOUT_S" env "${envs[@]}" bash "$R" expert-check "$stage" "$noise" > "$log" 2>&1
  fi
  local rc=$?
  local after; after=$(ls -t "$LOGDIR"/expert_check_*.json 2>/dev/null | head -1)
  if [ -n "$after" ] && [ "$after" != "$before" ]; then cp "$after" "$A/results/$name.json"; fi
  # 로그는 NZ2 줄 + 오류 꼬리만 따로 (읽기 쉽게)
  { grep -E '^\[NZ2' "$log"; echo "--- 꼬리 ---"; tail -n 40 "$log"; } > "$A/results/$name.summary.txt" 2>/dev/null
  mv "$job" "$A/done/"
  echo "[runner] $(date '+%T') 끝 $name rc=$rc $(( $(date +%s) - t0 )) s"
  echo "[runner] $(date '+%T') 끝 $name rc=$rc $(( $(date +%s) - t0 )) s" >> "$A/runner.log"
}

while [ ! -f "$A/STOP" ]; do
  job=$(ls "$A"/queue/*.job 2>/dev/null | sort | head -1)
  if [ -n "$job" ]; then run_job "$job"; else sleep 3; fi
  date +%s > "$A/heartbeat"
done
echo "[runner] STOP 파일 → 종료"
