#!/usr/bin/env python3
"""
monitor.py  -  NZ2 로봇팔 가동범위 측정 대시보드 (다이나믹셀 위자드의 "손으로 돌려서 범위 재기" 대체)

실행 위치: 개발 PC (OpenCR#2 를 USB 로 꽂고, OpenCR 에 usb_to_dxl 예제가 올라가 있어야 함)
사전 준비 (한 번만):
    python3 -m venv ~/dxlenv && ~/dxlenv/bin/pip install -r ../requirements.txt
실행:
    ~/dxlenv/bin/python monitor.py --port /dev/ttyACM0        # 실물
    ~/dxlenv/bin/python monitor.py --mock                     # 모터 없이 화면·CSV 시험
    ~/dxlenv/bin/python monitor.py --analyze range_logs/xxx.csv  # CSV 만 다시 분석
그다음 브라우저에서  http://127.0.0.1:8765  를 연다.

※ 다이나믹셀 위자드와 이 프로그램은 같은 포트를 못 쓴다. 위자드는 반드시 끄고 실행할 것 (연결 해제가 아니라 종료).
※ 이 프로그램이 모터에 쓰는 것은 "토크 OFF" 버튼 하나뿐이다 (손으로 돌리기 위해). 위치·EEPROM 은 쓰지 않는다.
※ 토크를 끄면 팔이 힘이 빠져 떨어진다. 한 손으로 팔을 받치고 누를 것.

각도 약속: 위치 모드(raw 0~4095)에서 raw 2048 = 팔 0°  ->  팔각도 = (raw % 4096) * 360/4096 - 180   (arm_config.h 와 같은 약속)
          모터가 Velocity/Extended 모드이면 위치가 멀티턴(예: 384.8°)으로 나오므로 % 4096 로 한 바퀴 안으로 접어서 쓴다.
          관절이 0/4095 경계를 넘나들어도 "연속 각도"(unwrap)로 이어서 계산하므로 최소/최대가 끊기지 않는다.
CSV: host/range_logs/arm_range_YYYYmmdd_HHMMSS.csv (원본 샘플 + 마커), 끝나면 *_summary.csv (모터별 최소·최대·제안 한계) 자동 생성.
"""
import argparse, csv, json, math, os, sys, threading, time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ======================= 상수 (여기만 고치면 됨) =======================
MOTOR_IDS = [11, 12, 13, 14, 16, 15]           # arm_config.h 와 같은 순서: 관절 11~14, 롤 16, 그리퍼 15
NAMES = {11: "관절1(베이스)", 12: "관절2(어깨)", 13: "관절3(팔꿈치)", 14: "관절4(손목)", 16: "손목 롤", 15: "그리퍼"}
BAUD = 1_000_000
PROTOCOL = 2.0
SAMPLE_HZ = 20
HISTORY_SEC = 12
SLOW_POLL_SEC = 1.0
HTTP_HOST, HTTP_PORT = "127.0.0.1", 8765
DEFAULT_MARGIN_DEG = 5.0                        # 관측 끝에서 안쪽으로 줄일 여유
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "range_logs")   # 실행 위치와 무관하게 host/range_logs/ (git 제외)
TICKS = 4096.0
# XM430-W350 컨트롤 테이블 (Protocol 2.0)
A_OPMODE, A_HOMING, A_TORQUE, A_HWERR = 11, 20, 64, 70
A_FAST, L_FAST = 126, 21                        # 126~146: 전류(2) 속도(4) 위치(4) ... 전압(2) 온도(1)
OPMODE_NAME = {0: "Current", 1: "Velocity", 3: "Position", 4: "ExtPosition", 5: "CurBasedPos", 16: "PWM"}
CUR_UNIT_MA, VEL_UNIT_RPM = 2.69, 0.229


def raw_to_deg(raw):                             # 한 바퀴 안으로 접은 팔 각도
    return (raw % TICKS) * 360.0 / TICKS - 180.0


def s16(v): return v - 65536 if v >= 32768 else v
def s32(v): return v - 4294967296 if v >= 2147483648 else v


# ======================= 모터 입출력 =======================
class RealBus:
    def __init__(self, port):
        from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
        self.sdk = __import__("dynamixel_sdk")
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise SystemExit(f"포트를 못 엽니다: {port}\n  - 다이나믹셀 위자드를 완전히 종료했나요? Arduino 시리얼 모니터도 닫으세요.\n  - 확인: sudo lsof {port}")
        self.port.setBaudRate(BAUD)
        self.pk = PacketHandler(PROTOCOL)
        self.gsr = GroupSyncRead(self.port, self.pk, A_FAST, L_FAST)
        for i in MOTOR_IDS: self.gsr.addParam(i)
        self.found = [i for i in MOTOR_IDS if self.pk.ping(self.port, i)[1] == self.sdk.COMM_SUCCESS]

    def fast(self):
        """6개 모터 위치·전류·속도·전압·온도를 한 번에. 반환 {id: dict or None}"""
        out = {i: None for i in MOTOR_IDS}
        if self.gsr.txRxPacket() != self.sdk.COMM_SUCCESS and not any(self.gsr.isAvailable(i, A_FAST, L_FAST) for i in MOTOR_IDS):
            return out
        for i in MOTOR_IDS:
            if not self.gsr.isAvailable(i, A_FAST, L_FAST): continue
            g = lambda a, n: self.gsr.getData(i, a, n)
            out[i] = dict(cur=s16(g(126, 2)) * CUR_UNIT_MA, vel=s32(g(128, 4)) * VEL_UNIT_RPM, raw=s32(g(132, 4)),
                          volt=g(144, 2) / 10.0, temp=g(146, 1))
        return out

    def slow(self, i):
        r = {}
        d, c, _ = self.pk.read1ByteTxRx(self.port, i, A_OPMODE); r["mode"] = d if c == self.sdk.COMM_SUCCESS else None
        d, c, _ = self.pk.read1ByteTxRx(self.port, i, A_TORQUE); r["torque"] = d if c == self.sdk.COMM_SUCCESS else None
        d, c, _ = self.pk.read1ByteTxRx(self.port, i, A_HWERR); r["hw_err"] = d if c == self.sdk.COMM_SUCCESS else None
        d, c, _ = self.pk.read4ByteTxRx(self.port, i, A_HOMING); r["homing"] = s32(d) if c == self.sdk.COMM_SUCCESS else None
        return r

    def torque_off(self, i):
        return self.pk.write1ByteTxRx(self.port, i, A_TORQUE, 0)[1] == self.sdk.COMM_SUCCESS

    def close(self): self.port.closePort()


class MockBus:
    """모터 없이 시험: ID11 은 Velocity 모드·멀티턴(384.8°), ID13 은 0/4095 경계를 넘나든다."""
    def __init__(self):
        self.t0 = time.time(); self.found = list(MOTOR_IDS); self.torque = {i: 1 for i in MOTOR_IDS}

    def fast(self):
        t = time.time() - self.t0; out = {}
        for k, i in enumerate(MOTOR_IDS):
            if i == 11: raw = 4376 + int(500 * math.sin(t * .7))
            elif i == 13: raw = int(4000 + 400 * math.sin(t * .9)) % 4096
            else: raw = int(2048 + (350 + 120 * k) * math.sin(t * (.5 + .1 * k)))
            out[i] = dict(cur=40 * abs(math.sin(t + k)), vel=0.0, raw=raw, volt=11.9, temp=31 + k)
        return out

    def slow(self, i):
        return dict(mode=1 if i == 11 else 3, torque=self.torque[i], hw_err=0, homing=0)

    def torque_off(self, i): self.torque[i] = 0; return True
    def close(self): pass


# ======================= 상태·기록 =======================
class Tracker:
    def __init__(self, bus):
        self.bus = bus; self.lock = threading.Lock(); self.stop = False
        self.m = {i: dict(ok=False, raw=None, deg=None, deg_cont=None, mn=None, mx=None, cur=0, vel=0, volt=0, temp=0,
                          mode=None, torque=None, hw_err=None, homing=None, first_raw=None, last_raw=None, unw=None)
                  for i in MOTOR_IDS}
        self.hist = {i: deque(maxlen=SAMPLE_HZ * HISTORY_SEC) for i in MOTOR_IDS}
        self.rec = None; self.rec_file = None; self.rec_rows = 0; self.rec_path = ""; self.writer = None
        self.t_start = time.time(); self.msg = ""
        self.zero = {i: None for i in MOTOR_IDS}   # 조립 기준 자세(=0°)일 때의 절대 각도[deg]

    # --- 연속 각도(unwrap): 0/4095 경계를 넘어도 끊기지 않게 ---
    def _unwrap(self, s, raw):
        if s["last_raw"] is None:
            s["first_raw"], s["unw"] = raw, float(raw)
        else:
            d = raw - s["last_raw"]
            if d > TICKS / 2: d -= TICKS * round(d / TICKS)
            elif d < -TICKS / 2: d -= TICKS * round(d / TICKS)
            s["unw"] += d
        s["last_raw"] = raw
        return raw_to_deg(s["first_raw"]) + (s["unw"] - s["first_raw"]) * 360.0 / TICKS

    def loop(self):
        last_slow = 0.0; dt = 1.0 / SAMPLE_HZ
        while not self.stop:
            t0 = time.time(); data = self.bus.fast(); now = time.time()
            with self.lock:
                if now - last_slow > SLOW_POLL_SEC:
                    last_slow = now
                    for i in MOTOR_IDS: self.m[i].update(self.bus.slow(i))
                for i in MOTOR_IDS:
                    s, d = self.m[i], data.get(i)
                    if d is None: s["ok"] = False; continue
                    s["ok"] = True; s.update(cur=d["cur"], vel=d["vel"], volt=d["volt"], temp=d["temp"], raw=d["raw"])
                    s["deg"] = raw_to_deg(d["raw"]); s["deg_cont"] = self._unwrap(s, d["raw"])
                    s["mn"] = s["deg_cont"] if s["mn"] is None else min(s["mn"], s["deg_cont"])
                    s["mx"] = s["deg_cont"] if s["mx"] is None else max(s["mx"], s["deg_cont"])
                    self.hist[i].append(round(s["deg_cont"], 2))
                    if self.rec:
                        self.writer.writerow(self._row(now, i, s, ""))
                        self.rec_rows += 1
                if self.rec_file: self.rec_file.flush()
            time.sleep(max(0, dt - (time.time() - t0)))

    def _row(self, now, i, s, note):
        z = self.zero[i]; zz = 0.0 if z is None else z
        return [f"{now - self.rec:.3f}", datetime.fromtimestamp(now).isoformat(timespec="milliseconds"), i, s["raw"], f"{s['unw']:.0f}",
                f"{s['deg_cont']:.2f}", f"{s['deg_cont'] - zz:.2f}", f"{zz:.2f}", f"{s['cur']:.0f}", f"{s['vel']:.1f}", s["temp"], f"{s['volt']:.1f}",
                s["torque"], s["hw_err"], s["mode"], s["homing"] or 0, note]

    def start_rec(self):
        with self.lock:
            if self.rec: return
            os.makedirs(LOG_DIR, exist_ok=True)
            self.rec_path = os.path.join(LOG_DIR, "arm_range_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
            self.rec_file = open(self.rec_path, "w", newline="", encoding="utf-8-sig")
            self.writer = csv.writer(self.rec_file)
            self.writer.writerow(CSV_HEAD)
            self.rec = time.time(); self.rec_rows = 0
            now = time.time()
            for i in MOTOR_IDS:                      # 기록 전에 영점을 잡았다면 CSV 첫머리에 ZERO 행으로 남긴다
                s = self.m[i]
                if self.zero[i] is not None and s["raw"] is not None:
                    self.writer.writerow(self._row(now, i, s, "ZERO")); self.rec_rows += 1
            self.msg = "기록 시작: " + self.rec_path

    def stop_rec(self):
        with self.lock:
            if not self.rec: return
            self.rec_file.close(); path = self.rec_path; self.rec = None; self.rec_file = None
        summ = analyze(path, DEFAULT_MARGIN_DEG)
        with self.lock: self.msg = f"기록 종료: {path} ({self.rec_rows}행) -> {summ}"

    def marker(self, mid, note):
        with self.lock:
            if not self.rec: self.msg = "기록 중일 때만 마커를 남길 수 있습니다"; return
            s = self.m.get(mid)
            if not s or s["raw"] is None: self.msg = "해당 모터 값이 없습니다"; return
            now = time.time()
            if note == "ZERO": note = "ZERO_"          # 예약어 충돌 방지
            self.writer.writerow(self._row(now, mid, s, note or "마커"))
            z = self.zero[mid] or 0.0
            self.rec_rows += 1; self.msg = f"마커: ID{mid} {s['deg_cont'] - z:.1f}° {note}"

    def set_zero(self, mid=None):
        """현재 자세를 0°(조립 기준 자세)로 기록. mid=None 이면 전체."""
        with self.lock:
            ids = MOTOR_IDS if mid in (None, 0) else [mid]
            n = 0; now = time.time()
            for i in ids:
                s = self.m.get(i)
                if not s or s["deg_cont"] is None: continue
                self.zero[i] = s["deg_cont"]; s["mn"] = s["mx"] = s["deg_cont"]; n += 1
                if self.rec: self.writer.writerow(self._row(now, i, s, "ZERO")); self.rec_rows += 1
            self.msg = f"영점 기록: {n}개 모터 (현재 자세 = 0°, 최소/최대 초기화)"

    def clear_zero(self):
        with self.lock:
            self.zero = {i: None for i in MOTOR_IDS}; self.msg = "영점 해제 (raw 2048 = 0° 기준으로 복귀)"

    def reset_minmax(self):
        with self.lock:
            for s in self.m.values(): s["mn"] = s["mx"] = s["deg_cont"]
            self.msg = "최소/최대를 현재 값으로 초기화"

    def torque_off(self):
        with self.lock:
            bad = [i for i in MOTOR_IDS if not self.bus.torque_off(i)]
            self.msg = "토크 OFF 전송 완료" + (f" (실패: {bad})" if bad else "")

    def snapshot(self):
        with self.lock:
            ms = []
            for i in MOTOR_IDS:
                s = self.m[i]; warn = []
                if s["mode"] is not None and s["mode"] != 3: warn.append(f"모드 {OPMODE_NAME.get(s['mode'], s['mode'])} (위치 모드 아님: % 4096 으로 환산)")
                if s["homing"]: warn.append(f"홈 오프셋 {s['homing']} (각도에 반영됨)")
                if s["hw_err"]: warn.append(f"하드웨어 오류 0x{s['hw_err']:02X}")
                if s["raw"] is not None and not (0 <= s["raw"] < 4096): warn.append("멀티턴 값(0~4095 밖)")
                z = self.zero[i]; zz = 0.0 if z is None else z
                sub = lambda v: None if v is None else v - zz
                ms.append(dict(id=i, name=NAMES[i], ok=s["ok"], raw=s["raw"], deg=sub(s["deg_cont"]), mn=sub(s["mn"]), mx=sub(s["mx"]), zero=z, cur=s["cur"], vel=s["vel"],
                               temp=s["temp"], volt=s["volt"], mode=OPMODE_NAME.get(s["mode"], s["mode"]), torque=s["torque"], hw_err=s["hw_err"],
                               warn=warn, hist=[round(h - zz, 2) for h in self.hist[i]]))
            return dict(motors=ms, rec=bool(self.rec), rec_path=self.rec_path, rec_rows=self.rec_rows,
                        rec_sec=(time.time() - self.rec) if self.rec else 0, msg=self.msg, found=self.bus.found,
                        mock=isinstance(self.bus, MockBus), hz=SAMPLE_HZ)


CSV_HEAD = ["t_sec", "time", "id", "raw", "unwrapped_raw", "deg_cont", "deg_rel", "zero_deg", "current_mA", "velocity_rpm", "temp_C", "volt_V",
            "torque", "hw_err", "mode", "homing_offset", "marker"]


# ======================= 분석 (CSV -> 요약 + 제안 한계) =======================
def analyze(path, margin):
    """영점(ZERO 행)이 있으면 '조립 기준 자세 = 0°' 기준 각도로, 없으면 raw 2048 = 0° 기준으로 요약한다."""
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    out = os.path.splitext(path)[0] + "_summary.csv"
    summ = []
    for i in MOTOR_IDS:
        mine = [x for x in rows if x["id"] == str(i)]
        r = [x for x in mine if not x["marker"]]
        mk = [x for x in mine if x["marker"] and x["marker"] != "ZERO"]
        zr = [x for x in mine if x["marker"] == "ZERO"]
        if not r: continue
        z = float(zr[-1]["zero_deg"]) if zr else 0.0                # 영점의 절대 각도(raw 2048 기준). 영점을 누른 순간의 값
        neutral_raw = int(round((z + 180.0) * TICKS / 360.0)) % int(TICKS) if zr else None   # 영점을 누른 순간의 raw
        ho_now = int(float(zr[-1].get("homing_offset") or 0)) if zr else None
        d = [float(x["deg_cont"]) - z for x in r]; mn, mx = min(d), max(d); span = mx - mn
        lo, hi = (mn + margin, mx - margin) if span > 2 * margin else (mn, mx)
        ho_new = None
        if neutral_raw is not None:                                   # 영점 raw 를 2048 로 옮기는 Homing Offset (가장 가까운 방향)
            T = int(TICKS); delta = ((T // 2 - neutral_raw + T // 2) % T) - T // 2
            ho_new = ho_now + delta
        summ.append(dict(id=i, name=NAMES[i], samples=len(r), zero_set="Y" if zr else "N", neutral_raw=neutral_raw if zr else "",
                         start_deg=round(d[0], 1), min_deg=round(mn, 1), max_deg=round(mx, 1), span_deg=round(span, 1),
                         propose_min=round(lo, 0), propose_max=round(hi, 0), margin=margin,
                         abs_min_deg=round(mn + z, 1), abs_max_deg=round(mx + z, 1),
                         homing_now="" if ho_now is None else ho_now, homing_new="" if ho_new is None else ho_new,
                         warn="" if span > 2 * margin else "움직인 범위가 여유의 2배 이하: 끝까지 돌렸는지 확인",
                         markers=" | ".join(f"{x['marker']}@{float(x['deg_cont']) - z:.1f}°" for x in mk)))
    if summ:
        with open(out, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
    nz = [s["id"] for s in summ if s["zero_set"] == "N"]
    print("\n=== 가동범위 요약 (0° = 조립 기준 자세; 영점 없는 모터는 raw 2048 기준) ===")
    if nz: print(f"※ 영점이 없는 모터: {nz} -> raw 2048 기준 각도입니다. 기준 자세에서 '영점 기록'을 누르고 다시 측정하세요.")
    print(f"{'ID':>3} {'이름':<12}{'시작':>8}{'최소':>8}{'최대':>8}{'범위':>8}   제안 한계(안쪽 {margin:g}°)")
    for s in summ:
        print(f"{s['id']:>3} {s['name']:<12}{s['start_deg']:>8}{s['min_deg']:>8}{s['max_deg']:>8}{s['span_deg']:>8}   J_MIN={s['propose_min']:.0f}  J_MAX={s['propose_max']:.0f}  {s['warn']}")
        if s["markers"]: print(f"      마커: {s['markers']}")
    if summ:
        order = {s["id"]: s for s in summ}
        mn = ", ".join(f"{order[i]['propose_min']:.0f}" if i in order else "?" for i in MOTOR_IDS)
        mx = ", ".join(f"{order[i]['propose_max']:.0f}" if i in order else "?" for i in MOTOR_IDS)
        print("\narm_config.h 에 넣을 값 (순서: 11,12,13,14,16,15) - 0° = 조립 기준 자세:")
        print(f"static const float J_MIN[N_MOTOR] = {{{mn}}};\nstatic const float J_MAX[N_MOTOR] = {{{mx}}};")
        hs = [s for s in summ if s["homing_new"] != ""]
        if hs:
            print("\n[중요] 펌웨어는 '팔 0° = 모터 raw 2048' 로 계산합니다. 기준 자세가 raw 2048 이 아니므로 아래 중 하나가 필요합니다.")
            print("  (A) 위자드에서 Homing Offset(주소 20)을 아래 값으로 바꿔 기준 자세가 raw 2048 이 되게 한다 (권장: 펌웨어·URDF 수정 없음)")
            print("  (B) 펌웨어에 모터별 ARM_OFFSET 배열을 추가한다")
            print(f"  {'ID':>3} {'기준 자세 raw':>13} {'현재 Homing':>12} {'새 Homing(A)':>13}")
            for s in hs: print(f"  {s['id']:>3} {s['neutral_raw']:>13} {s['homing_now']:>12} {s['homing_new']:>13}")
        print(f"요약 파일: {out}")
    return out


# ======================= 웹 대시보드 =======================
PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NZ2 팔 가동범위 측정</title><style>
:root{--bg:#f3f5f9;--card:#fff;--ink:#152238;--mut:#5d6b82;--line:#d5dbe5;--acc:#0b3d91;--red:#cf2e2e;--ok:#1f7a52;--warn:#c25e00}
@media (prefers-color-scheme:dark){:root{--bg:#10151f;--card:#18202e;--ink:#e6ebf4;--mut:#93a1b8;--line:#2b3649;--acc:#7aa7ff;--red:#ff6b6b;--ok:#58d6a0;--warn:#ffb054}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,"Noto Sans KR","Malgun Gothic",sans-serif}
header{padding:12px 16px;background:#17325f;color:#fff;display:flex;gap:12px;flex-wrap:wrap;align-items:center}header h1{font-size:16px;margin:0}
.pill{padding:2px 10px;border-radius:12px;background:#ffffff22;font-size:12px}.rec{background:var(--red)}
main{padding:12px 16px;display:grid;gap:12px;max-width:1200px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px}
.bar{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
button{font:inherit;border:1px solid var(--line);background:var(--card);color:var(--ink);padding:6px 12px;border-radius:6px;cursor:pointer}
button.p{background:var(--acc);color:#fff;border-color:var(--acc)}button.d{background:var(--red);color:#fff;border-color:var(--red)}
input,select{font:inherit;padding:5px 8px;border:1px solid var(--line);border-radius:6px;background:var(--card);color:var(--ink)}
table{width:100%;border-collapse:collapse}th,td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:right;white-space:nowrap}th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}
th{color:var(--mut);font-weight:600;font-size:12px}tr.sel{background:#0b3d9114}tr.bad td{color:var(--mut)}
.gauge{position:relative;height:16px;min-width:260px;background:linear-gradient(90deg,#0000,#0000),var(--line);border-radius:8px;overflow:hidden}
.gauge .rng{position:absolute;top:0;bottom:0;background:#1f7a5255}.gauge .cur{position:absolute;top:-2px;bottom:-2px;width:3px;background:var(--red)}
.gauge .z{position:absolute;top:0;bottom:0;width:1px;background:var(--mut)}
.warn{color:var(--warn);font-size:12px}canvas{width:100%;height:220px;display:block}.msg{color:var(--mut);font-size:13px;min-height:20px}
.legend span{display:inline-block;margin-right:12px;font-size:12px}.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px}
</style></head><body>
<header><h1>NZ2 팔 가동범위 측정</h1><span class="pill" id="conn">연결 중…</span><span class="pill" id="recp">기록 안 함</span><span class="pill" id="mockp" style="display:none">MOCK</span></header>
<main>
<div class="card"><div class="bar">
<button class="p" id="bRec">● 기록 시작</button>
<span style="margin-left:12px;color:var(--mut)">마커</span><select id="mId"></select><input id="mNote" placeholder="예: 전선 팽팽 / 차체에 닿음 / 한쪽 끝" size="26"><button id="bMark">마커 남기기</button>
<span style="flex:1"></span><button id="bZero">영점 기록(전체)</button><button id="bZero1">선택 ID 영점</button><button id="bZeroX">영점 해제</button><button id="bReset">최소/최대 초기화</button><button class="d" id="bTq">토크 OFF (전체)</button></div>
<div class="msg" id="msg"></div></div>
<div class="card"><table><thead><tr><th>ID</th><th>이름</th><th>모드</th><th>토크</th><th>위치 raw</th><th>영점(절대°)</th><th>각도(°)</th><th>최소(°)</th><th>최대(°)</th><th>범위(°)</th><th>전류 mA</th><th>온도 °C</th><th>전압 V</th><th>가동범위 (−180°~+180°)</th></tr></thead><tbody id="tb"></tbody></table><div id="warns"></div></div>
<div class="card"><div class="legend" id="lg"></div><canvas id="cv"></canvas></div>
</main><script>
const COL=['#0b3d91','#c25e00','#1f7a52','#7a3fb0','#cf2e2e','#2a9db5'];let sel=0,last=null;
const $=id=>document.getElementById(id);const f=(v,d=1)=>v==null?'—':Number(v).toFixed(d);
async function post(u,b){await fetch(u,{method:'POST',body:JSON.stringify(b||{})});}
$('bRec').onclick=()=>post('/api/record',{on:!(last&&last.rec)});
$('bMark').onclick=()=>post('/api/marker',{id:+$('mId').value,note:$('mNote').value});
$('bZero').onclick=()=>post('/api/zero',{});$('bZero1').onclick=()=>post('/api/zero',{id:+$('mId').value});$('bZeroX').onclick=()=>post('/api/zero_clear');
$('bReset').onclick=()=>post('/api/reset');
$('bTq').onclick=()=>{if(confirm('토크를 끄면 팔이 떨어집니다.\n한 손으로 팔을 받치고 있나요?'))post('/api/torque_off')};
function draw(ms){const c=$('cv'),r=c.getBoundingClientRect(),dp=devicePixelRatio||1;c.width=r.width*dp;c.height=r.height*dp;const g=c.getContext('2d');g.scale(dp,dp);
const W=r.width,H=r.height,sty=getComputedStyle(document.body);g.clearRect(0,0,W,H);g.font='11px system-ui';g.fillStyle=sty.getPropertyValue('--mut');g.strokeStyle=sty.getPropertyValue('--line');
let lo=-180,hi=180;ms.forEach(m=>m.hist.forEach(v=>{lo=Math.min(lo,v);hi=Math.max(hi,v)}));lo=Math.floor(lo/30)*30;hi=Math.ceil(hi/30)*30;
const y=v=>H-18-(v-lo)/(hi-lo)*(H-26);for(let v=lo;v<=hi;v+=60){g.beginPath();g.moveTo(34,y(v));g.lineTo(W,y(v));g.stroke();g.fillText(v+'°',2,y(v)+4)}
ms.forEach((m,k)=>{if(!m.hist.length)return;g.strokeStyle=COL[k];g.lineWidth=(k===sel?2.6:1.3);g.globalAlpha=(k===sel?1:.6);g.beginPath();
m.hist.forEach((v,i)=>{const x=34+i/(m.hist.length-1||1)*(W-36);i?g.lineTo(x,y(v)):g.moveTo(x,y(v))});g.stroke();g.globalAlpha=1});}
async function tick(){try{const s=await (await fetch('/api/state')).json();last=s;$('conn').textContent='연결됨 · 발견 ID '+s.found.join(', ');
$('mockp').style.display=s.mock?'':'none';const rp=$('recp');rp.textContent=s.rec?`● 기록 중 ${s.rec_sec.toFixed(0)}s · ${s.rec_rows}행`:'기록 안 함';rp.className='pill'+(s.rec?' rec':'');
$('bRec').textContent=s.rec?'■ 기록 종료 + 요약':'● 기록 시작';$('bRec').className=s.rec?'d':'p';$('msg').textContent=s.msg+(s.rec?'  → '+s.rec_path:'');
if(!$('mId').options.length)s.motors.forEach(m=>$('mId').add(new Option('ID '+m.id+' '+m.name,m.id)));
if(!$('lg').innerHTML)$('lg').innerHTML=s.motors.map((m,k)=>`<span><i style="background:${COL[k]}"></i>ID ${m.id}</span>`).join('')+'<span style="color:var(--mut)">(행을 클릭하면 굵게 표시 · 최근 12초)</span>';
$('tb').innerHTML=s.motors.map((m,k)=>{const L=v=>((v+180)/360*100).toFixed(2)+'%';const a=m.mn==null?0:Math.max(0,(m.mn+180)/360*100),b=m.mx==null?0:Math.min(100,(m.mx+180)/360*100);
return `<tr class="${k===sel?'sel':''}${m.ok?'':' bad'}" data-k="${k}"><td><b style="color:${COL[k]}">${m.id}</b></td><td>${m.name}</td><td>${m.mode??'—'}</td><td>${m.torque==null?'—':(m.torque?'ON':'OFF')}</td>
<td>${m.raw??'—'}</td><td>${m.zero==null?'—':f(m.zero)}</td><td><b>${f(m.deg)}</b></td><td>${f(m.mn)}</td><td>${f(m.mx)}</td><td>${m.mn==null?'—':f(m.mx-m.mn)}</td><td>${f(m.cur,0)}</td><td>${m.temp}</td><td>${f(m.volt)}</td>
<td><div class="gauge"><div class="z" style="left:50%"></div><div class="rng" style="left:${a}%;width:${Math.max(0,b-a)}%"></div>${m.deg==null?'':`<div class="cur" style="left:calc(${L(m.deg)} - 1px)"></div>`}</div></td></tr>`}).join('');
document.querySelectorAll('#tb tr').forEach(r=>r.onclick=()=>{sel=+r.dataset.k;$('mId').selectedIndex=sel});
$('warns').innerHTML=s.motors.filter(m=>m.warn.length).map(m=>`<div class="warn">ID ${m.id}: ${m.warn.join(' · ')}</div>`).join('');draw(s.motors);}catch(e){$('conn').textContent='서버 연결 끊김'}setTimeout(tick,100)}tick();
</script></body></html>"""


def make_handler(tr):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def _send(self, body, ctype="application/json"):
            b = body.encode() if isinstance(body, str) else body
            self.send_response(200); self.send_header("Content-Type", ctype + "; charset=utf-8"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

        def do_GET(self):
            if self.path.startswith("/api/state"): self._send(json.dumps(tr.snapshot()))
            else: self._send(PAGE, "text/html")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try: b = json.loads(self.rfile.read(n) or b"{}")
            except Exception: b = {}
            if self.path == "/api/record": tr.start_rec() if b.get("on") else tr.stop_rec()
            elif self.path == "/api/marker": tr.marker(int(b.get("id", 0)), str(b.get("note", ""))[:80])
            elif self.path == "/api/reset": tr.reset_minmax()
            elif self.path == "/api/zero": tr.set_zero(int(b["id"]) if b.get("id") else None)
            elif self.path == "/api/zero_clear": tr.clear_zero()
            elif self.path == "/api/torque_off": tr.torque_off()
            self._send("{}")
    return H


def main():
    global DEFAULT_MARGIN_DEG
    ap = argparse.ArgumentParser(description="NZ2 팔 가동범위 측정 대시보드")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="모터 없이 화면·CSV 시험")
    ap.add_argument("--analyze", metavar="CSV", help="기록된 CSV 를 다시 분석")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN_DEG, help="제안 한계에서 안쪽으로 줄일 각도(°)")
    a = ap.parse_args()
    DEFAULT_MARGIN_DEG = a.margin
    if a.analyze: analyze(a.analyze, a.margin); return
    bus = MockBus() if a.mock else RealBus(a.port)
    missing = [i for i in MOTOR_IDS if i not in bus.found]
    print(f"발견된 모터 ID: {bus.found}" + (f"   ※ 안 보이는 ID: {missing} (케이블·12 V·ID 확인)" if missing else ""))
    tr = Tracker(bus)
    threading.Thread(target=tr.loop, daemon=True).start()
    srv = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), make_handler(tr))
    print(f"대시보드: http://{HTTP_HOST}:{HTTP_PORT}   (종료: Ctrl+C)")
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        tr.stop = True; tr.stop_rec(); bus.close(); print("종료")


if __name__ == "__main__":
    main()
