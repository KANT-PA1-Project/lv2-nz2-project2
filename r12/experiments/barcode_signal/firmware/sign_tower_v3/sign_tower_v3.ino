/*
 * sign_tower_v3.ino — "주사위 기억 로봇" (발제 v5) 신호등 단독 구동 펌웨어
 * 보드: OpenCR 1.0. PC 없이 전원만 넣으면 혼자 동작한다 (시리얼은 연결했을 때만 기록·명령용).
 *
 * 전원 ON 후 동작 (AUTO_RUN = true)
 *   1) 전체 소등으로 FIRST_DELAY_MS 대기          → 로봇이 관측 자세를 잡고 꺼진 신호등을 고정하는 시간
 *   2) 무작위 12회 시퀀스 재생: 1회 = 1초 켜짐 + 1초 소등 (같은 색 연속도 소등으로 구별)
 *   3) 12번째 점멸 후 3초 연속 소등 → 종료
 *   4) REPEAT_GAP_MS 소등 대기 후 새 무작위 시퀀스로 다시 2)   (AUTO_REPEAT = false 면 1회만)
 *   버튼: SW1 = 대기 건너뛰고 바로 시작(새 무작위) / SW2 = 즉시 소등·정지 (다시 SW1 로 시작)
 *
 * 무작위 규칙 (mission.pdf 02·04)
 *   - R·Y·G 각 1~6회, 합계 12
 *   - 실제 퍽 구성으로 풀 수 있는 횟수 조합만 선택 (PUCK_* 상수). Y 는 한 모양으로 통일,
 *     R 은 빨간 퍽, G 는 초록 퍽만 → Y 에 쓰는 빨강 = 빨강총수-R, 초록 = 초록총수-G 가
 *     어느 한 모양의 개수 안에 들어가야 풀이 가능
 *   - 풀 수 있는 (R,Y,G) 조합 중 하나를 고르게 뽑고, 순서는 Fisher-Yates 로 섞음
 *   - 난수 씨앗: 아날로그 입력 잡음 여러 번 + micros() 를 섞음 (전원마다 다른 패턴)
 *
 * 하드웨어 (sign_tower_v2 와 동일)
 *   릴레이 3채널 Active-LOW → OpenCR 핀 5, 6, 7 = [가정] R, Y, G
 *   OpenCR 사용자 LED (있으면): LED1 = 대기 중 깜빡임, LED2 = 재생 중, LED4 = 설정 오류
 *
 * 시리얼 (115200, 연결했을 때만): v2 명령 + AUTO 1|0, PUCKS rc rb gc gb
 *   로그 "SEQ,<12글자>,R=,Y=,G=" 와 "EVT,<ms>,<번호>,<색>,ON|OFF" → 로봇 기록과 대조하는 정답
 *   PC 를 연결하지 않으면 정답 기록이 남지 않음 → 채점 대조가 필요하면 시리얼 연결 또는 FIXED_SEQ 사용
 */

// ===================== 설정값 (여기만 고치면 됨) =====================
const uint8_t  RELAY_PIN[3]     = {5, 6, 7};     // [가정] R, Y, G 순서
const char     LAMP_CHAR[3]     = {'R', 'Y', 'G'};
const bool     RELAY_ACTIVE_LOW = true;          // 릴레이 모듈이 LOW 에서 켜짐

const bool     AUTO_RUN         = true;          // 전원 ON 후 자동 시작
const bool     AUTO_REPEAT      = true;          // 끝나면 새 무작위로 계속 반복
const uint32_t FIRST_DELAY_MS   = 15000;         // 전원 ON 후 첫 시작까지 소등 대기 (로봇 준비 시간)
const uint32_t REPEAT_GAP_MS    = 20000;         // 반복 사이 소등 대기
const char*    FIXED_SEQ        = "";            // 비우면 무작위. "RGRRGRGGYGYG" 처럼 넣으면 항상 이 패턴

const uint32_t BLINK_ON_MS      = 1000;          // 켜짐 1초
const uint32_t BLINK_OFF_MS     = 1000;          // 점멸 사이 소등 1초
const uint32_t END_OFF_MS       = 3000;          // [가정] 12번째 소등 시작부터 연속 소등 3초 (마지막 소등 1초 포함)

// 퍽 구성 [가정: 공개 예제 빨강6·초록6, 모양 반반]. 실제 회차 등록표로 바꿀 것. 합계 12 여야 함
const uint8_t  PUCK_RED_CYL     = 3;
const uint8_t  PUCK_RED_BOX     = 3;
const uint8_t  PUCK_GREEN_CYL   = 3;
const uint8_t  PUCK_GREEN_BOX   = 3;

const uint8_t  RULE_BLINKS      = 12;
const uint8_t  RULE_MIN_COUNT   = 1;
const uint8_t  RULE_MAX_COUNT   = 6;
const uint8_t  MAX_BLINKS       = 24;
const uint32_t SERIAL_BAUD      = 115200;
const uint32_t BTN_DEBOUNCE_MS  = 50;
// =====================================================================

enum State { ST_IDLE, ST_WAIT, ST_ON, ST_OFF_GAP, ST_END_HOLD };

State    state = ST_IDLE;
char     seq[MAX_BLINKS + 1];
uint8_t  seqLen = 0;
uint8_t  curIdx = 0;
uint32_t tNext = 0;           // 다음 전환 시각 (누적 → 오차 누적 없음)
uint32_t tStart = 0;
bool     autoRepeat = AUTO_REPEAT;
bool     configError = false;
uint8_t  puck[4] = {PUCK_RED_CYL, PUCK_RED_BOX, PUCK_GREEN_CYL, PUCK_GREEN_BOX};
char     lineBuf[64];
uint8_t  lineLen = 0;

// ---------- 릴레이 ----------
void relayWrite(uint8_t ch, bool on) {
  digitalWrite(RELAY_PIN[ch], (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setLamp(int8_t lamp) {                       // 항상 최대 1개만 켬. lamp<0 → 전체 소등
  for (uint8_t i = 0; i < 3; i++) relayWrite(i, false);
  if (lamp >= 0 && lamp < 3) relayWrite(lamp, true);
}

int8_t lampIndex(char c) {
  for (uint8_t i = 0; i < 3; i++) if (LAMP_CHAR[i] == c) return i;
  return -1;
}

// ---------- 상태 LED (OpenCR 사용자 LED, 없으면 무시) ----------
void statusLed(uint8_t n, bool on) {
#ifdef BDPIN_LED_USER_1
  const uint8_t pins[4] = {BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4};
  if (n >= 1 && n <= 4) digitalWrite(pins[n - 1], on ? LOW : HIGH);   // OpenCR 사용자 LED 는 LOW 점등
#else
  (void)n; (void)on;
#endif
}

// ---------- 시퀀스 규칙 ----------
// 풀이 가능 여부: Y 에 쓸 빨강·초록 수가 한 모양 안에 들어가는가
bool feasible(uint8_t r, uint8_t y, uint8_t g) {
  uint8_t redTotal = puck[0] + puck[1], greenTotal = puck[2] + puck[3];
  if (r > redTotal || g > greenTotal) return false;
  uint8_t yRed = redTotal - r, yGreen = greenTotal - g;
  if (yRed + yGreen != y) return false;
  for (uint8_t s = 0; s < 2; s++)                 // s=0 원기둥, s=1 직육면체
    if (yRed <= puck[0 + s] && yGreen <= puck[2 + s]) return true;
  return false;
}

const char* validateSeq(const char* s, bool strict) {
  uint8_t n = strlen(s), cnt[3] = {0, 0, 0};
  if (n == 0) return "빈 시퀀스";
  if (n > MAX_BLINKS) return "길이 초과";
  for (uint8_t i = 0; i < n; i++) {
    int8_t k = lampIndex(s[i]);
    if (k < 0) return "R/Y/G 이외의 글자";
    cnt[k]++;
  }
  if (!strict) return NULL;
  if (n != RULE_BLINKS) return "점멸 횟수가 12가 아님";
  for (uint8_t k = 0; k < 3; k++)
    if (cnt[k] < RULE_MIN_COUNT || cnt[k] > RULE_MAX_COUNT) return "색별 횟수가 1~6 범위 밖";
  if (!feasible(cnt[0], cnt[1], cnt[2])) return "현재 퍽 구성으로 풀 수 없는 조합";
  return NULL;
}

void setSeq(const char* s) {
  strncpy(seq, s, MAX_BLINKS);
  seq[MAX_BLINKS] = 0;
  seqLen = strlen(seq);
}

// 풀 수 있는 (R,Y,G) 조합을 모두 세고 하나를 고르게 선택 → 순서 섞기. 조합이 없으면 false
bool randomSeq() {
  uint8_t combos[64][3], n = 0;
  for (uint8_t r = RULE_MIN_COUNT; r <= RULE_MAX_COUNT; r++)
    for (uint8_t y = RULE_MIN_COUNT; y <= RULE_MAX_COUNT; y++) {
      int8_t g = RULE_BLINKS - r - y;
      if (g < RULE_MIN_COUNT || g > RULE_MAX_COUNT) continue;
      if (!feasible(r, y, g)) continue;
      combos[n][0] = r; combos[n][1] = y; combos[n][2] = g; n++;
    }
  if (n == 0) return false;
  uint8_t* c = combos[random(n)];
  uint8_t m = 0;
  for (uint8_t k = 0; k < 3; k++) for (uint8_t j = 0; j < c[k]; j++) seq[m++] = LAMP_CHAR[k];
  for (int8_t i = m - 1; i > 0; i--) {             // Fisher-Yates
    uint8_t j = random(i + 1);
    char t = seq[i]; seq[i] = seq[j]; seq[j] = t;
  }
  seq[m] = 0; seqLen = m;
  return true;
}

void printCounts() {
  uint8_t cnt[3] = {0, 0, 0};
  for (uint8_t i = 0; i < seqLen; i++) { int8_t k = lampIndex(seq[i]); if (k >= 0) cnt[k]++; }
  Serial.print("SEQ,"); Serial.print(seq);
  Serial.print(",R="); Serial.print(cnt[0]);
  Serial.print(",Y="); Serial.print(cnt[1]);
  Serial.print(",G="); Serial.println(cnt[2]);
}

// 다음 재생할 시퀀스 준비 (고정 시퀀스가 있으면 그것, 없으면 새 무작위)
bool prepareNext() {
  if (FIXED_SEQ[0]) {
    const char* e = validateSeq(FIXED_SEQ, true);
    if (e) { Serial.print("ERR,FIXED_SEQ "); Serial.println(e); return false; }
    setSeq(FIXED_SEQ);
    return true;
  }
  if (!randomSeq()) { Serial.println("ERR,풀 수 있는 조합 없음 (PUCK_* 확인)"); return false; }
  return true;
}

// ---------- 진행 ----------
void scheduleStart(uint32_t delayMs) {            // delayMs 동안 소등 대기 후 재생
  setLamp(-1);
  if (configError || !prepareNext()) { configError = true; state = ST_IDLE; statusLed(4, true); return; }
  curIdx = 0;
  tNext = millis() + delayMs;
  state = ST_WAIT;
  Serial.print("WAIT,"); Serial.print(delayMs); Serial.print(",");
  printCounts();
}

void stopSignal(const char* why) {
  setLamp(-1);
  state = ST_IDLE;
  statusLed(1, false); statusLed(2, false);
  Serial.print("STOP,"); Serial.println(why);
}

void logEvt(uint8_t k, char c, const char* onoff) {
  Serial.print("EVT,"); Serial.print(millis() - tStart);
  Serial.print(","); Serial.print(k);
  Serial.print(","); Serial.print(c);
  Serial.print(","); Serial.println(onoff);
}

void updateStateMachine() {
  if (state == ST_IDLE) return;
  if (state == ST_WAIT) statusLed(1, (millis() / 500) % 2);       // 대기 중 LED1 깜빡임
  if ((int32_t)(millis() - tNext) < 0) return;                  // millis 오버플로 안전

  switch (state) {
    case ST_WAIT:
      statusLed(1, false); statusLed(2, true);
      tStart = millis();
      tNext = tStart;
      Serial.print("START,"); Serial.println(seq);
      // fall through: 바로 첫 점등
    case ST_OFF_GAP:
      if (state == ST_OFF_GAP) curIdx++;
      setLamp(lampIndex(seq[curIdx]));
      logEvt(curIdx + 1, seq[curIdx], "ON");
      tNext += BLINK_ON_MS;
      state = ST_ON;
      break;

    case ST_ON:
      setLamp(-1);
      logEvt(curIdx + 1, seq[curIdx], "OFF");
      if (curIdx + 1 >= seqLen) { tNext += END_OFF_MS;   state = ST_END_HOLD; }
      else                      { tNext += BLINK_OFF_MS; state = ST_OFF_GAP; }
      break;

    case ST_END_HOLD:
      statusLed(2, false);
      Serial.print("DONE,"); Serial.println(millis() - tStart);
      if (autoRepeat) scheduleStart(REPEAT_GAP_MS);
      else            state = ST_IDLE;
      break;

    default: break;
  }
}

// ---------- 시리얼 (연결했을 때만) ----------
void startCurrent() {                            // 이미 고른 seq 를 바로 재생
  setLamp(-1);
  curIdx = 0;
  tNext = millis();
  state = ST_WAIT;
}

void handleLine(char* s) {
  for (char* p = s; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  char* arg = strchr(s, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }
  bool busy = (state == ST_ON || state == ST_OFF_GAP || state == ST_END_HOLD);

  if (!strcmp(s, "STOP"))       stopSignal("명령");
  else if (!strcmp(s, "START")) {
    if (busy) Serial.println("ERR,진행 중");
    else if (state == ST_IDLE && seqLen) startCurrent();   // RAND/SEQ/Q1 로 고른 시퀀스 그대로 재생
    else scheduleStart(0);                                  // 자동 대기 중이면 바로 시작
  }
  else if (!strcmp(s, "AUTO"))  { autoRepeat = (arg && arg[0] == '1'); Serial.print("AUTO,"); Serial.println(autoRepeat); }
  else if (!strcmp(s, "STATUS")) {
    Serial.print("STATUS,state="); Serial.print((int)state); Serial.print(",auto="); Serial.print(autoRepeat);
    Serial.print(",pucks="); for (uint8_t i = 0; i < 4; i++) { Serial.print(puck[i]); Serial.print(i < 3 ? "/" : "\n"); }
    printCounts();
  }
  else if (!strcmp(s, "PUCKS")) {                 // PUCKS 빨강원기둥 빨강직육면체 초록원기둥 초록직육면체
    int v[4];
    if (!arg || sscanf(arg, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) != 4 || v[0] + v[1] + v[2] + v[3] != 12 ||
        v[0] < 0 || v[1] < 0 || v[2] < 0 || v[3] < 0) { Serial.println("ERR,PUCKS rc rb gc gb (합계 12)"); return; }
    for (uint8_t i = 0; i < 4; i++) puck[i] = v[i];
    configError = false; statusLed(4, false);
    Serial.println("OK,PUCKS (다음 시퀀스부터 적용)");
  }
  else if (!strcmp(s, "RAND") || !strcmp(s, "Q1") || !strcmp(s, "SEQ")) {
    if (busy) { Serial.println("ERR,진행 중에는 변경 불가 (STOP 후)"); return; }
    if (!strcmp(s, "RAND")) { if (!randomSeq()) { Serial.println("ERR,풀 수 있는 조합 없음"); return; } }
    else {
      const char* src = !strcmp(s, "Q1") ? "RGRRGRGGYGYG" : arg;
      const char* e = src ? validateSeq(src, true) : "인자 없음";
      if (e) { Serial.print("ERR,"); Serial.println(e); return; }
      setSeq(src);
    }
    state = ST_IDLE;                               // 지정한 시퀀스는 START 로 재생 (무작위로 덮어쓰지 않음)
    printCounts();
    Serial.println("INFO,START 로 이 시퀀스 재생 (AUTO 반복은 다음 회차부터 새 무작위)");
  }
  else if (!strcmp(s, "LAMP")) {
    if (busy || state == ST_WAIT) { Serial.println("ERR,대기·진행 중에는 불가 (STOP 후)"); return; }
    if (arg && !strcmp(arg, "OFF")) setLamp(-1);
    else if (arg && lampIndex(arg[0]) >= 0) setLamp(lampIndex(arg[0]));
    else Serial.println("ERR,LAMP R|Y|G|OFF");
  }
  else if (!strcmp(s, "HELP") || !strcmp(s, "?"))
    Serial.println("START STOP AUTO 1|0 RAND Q1 SEQ <12글자> PUCKS rc rb gc gb LAMP R|Y|G|OFF STATUS");
  else if (s[0]) Serial.println("ERR,알 수 없는 명령 (HELP)");
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    else lineLen = 0;
  }
}

// ---------- 버튼: SW1 = 바로 시작(새 무작위) / SW2 = 정지 ----------
void pollButtons() {
#ifdef BDPIN_PUSH_SW_1
  static uint32_t tLast = 0;
  static uint8_t prev = 0;
  if (millis() - tLast < BTN_DEBOUNCE_MS) return;
  tLast = millis();
  uint8_t now = getPushButton();                  // bit0 = SW1, bit1 = SW2
  uint8_t pressed = now & ~prev;
  prev = now;
  if (pressed & 0x02) stopSignal("SW2");
  else if ((pressed & 0x01) && !(state == ST_ON || state == ST_OFF_GAP || state == ST_END_HOLD)) {
    randomSeed(micros());                         // 사람이 누른 시각으로 다시 섞음
    scheduleStart(0);
  }
#endif
}

uint32_t noiseSeed() {                            // 아날로그 잡음 LSB 여러 번 + micros
  uint32_t s = micros();
  for (uint8_t i = 0; i < 64; i++) {
    s = (s << 1) ^ (s >> 31) ^ (uint32_t)analogRead(A0 + (i % 4));
    delayMicroseconds(37);
  }
  return s ^ micros();
}

void setup() {
  for (uint8_t i = 0; i < 3; i++) {               // 출력 모드 전에 '꺼짐' 레벨부터 (릴레이 오동작 방지)
    digitalWrite(RELAY_PIN[i], RELAY_ACTIVE_LOW ? HIGH : LOW);
    pinMode(RELAY_PIN[i], OUTPUT);
  }
  setLamp(-1);
#ifdef BDPIN_LED_USER_1
  const uint8_t leds[4] = {BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4};
  for (uint8_t i = 0; i < 4; i++) { pinMode(leds[i], OUTPUT); digitalWrite(leds[i], HIGH); }
#endif
  Serial.begin(SERIAL_BAUD);
  randomSeed(noiseSeed());
  if (puck[0] + puck[1] + puck[2] + puck[3] != 12) {
    configError = true; statusLed(4, true);
    Serial.println("ERR,PUCK_* 합계가 12가 아님");
  }
  Serial.println("sign_tower_v3 ready (HELP)");
  if (AUTO_RUN) scheduleStart(FIRST_DELAY_MS);
}

void loop() {
  pollSerial();
  pollButtons();
  updateStateMachine();
}
