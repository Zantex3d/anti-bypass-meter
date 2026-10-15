/*
  ============================================================================
  EXTERNAL VERIFIER  -  PROTEUS SIMULATION VERSION  (System 2, stand-in)
  Second Arduino Mega 2560 standing in for  ESP32 + ATM90E26
  ============================================================================

  Proteus has no ATM90E26 or ESP32 model, so this board plays both roles:
    * REFERENCE INSTRUMENT: its own transformer, divider and ACS712 measure the
      same load independently of the meter under test (A0 / A1).
    * VERIFIER: receives the meter's registered energy over a serial link,
      compares windows of energy, applies tolerance + persistence, and shows
      the result on LCD2 and three LEDs.
  The real hardware version is 02_Verifier_ESP32_ATM90E26.ino (Sheet 2).
  Both print the same CSV, so 03_Analysis_and_AI.py reads either one.

  PIN MAP
    A0  voltage sense (TR2 + divider)        A1  current sense (ACS2)
    LCD2 : RS=D22 E=D23 D4=D24 D5=D25 D6=D26 D7=D27 (RW = GND)
    LEDs : green D8, yellow D9, red D10  (each via 330 ohm to GND)
    RX1  : D19  <- meter TX1 (D18)  (direct wire, both 5 V, common GND)
    TX0  : D1   -> Virtual Terminal (115200), RX0 accepts commands

  COMMANDS (type in the Virtual Terminal)
    L0 / L1  experiment label (0 normal, 1 known under-registration)
    R<n>     run id, e.g. R3
    Z        zero accumulators
  ============================================================================
*/

#include <LiquidCrystal.h>

LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

const uint8_t PIN_V = A0;
const uint8_t PIN_I = A1;
const uint8_t LED_G = 8, LED_Y = 9, LED_R = 10;

// ---------------------------- MEASUREMENT CONSTANTS ------------------------
// Independent chain, same design values as the meter (TR2 20:1, R 68k/10k,
// ACS712-05B). Calibrate YOUR copy against an AC meter in simulation.
const float ADC_VREF = 5.0, ADC_STEPS = 1023.0;
const float TRAFO_RATIO = 20.0;
const float DIV_RATIO = (68.0 + 10.0) / 10.0;
const float VOLT_PER_COUNT = (ADC_VREF / ADC_STEPS) * DIV_RATIO * TRAFO_RATIO;
const float ACS_V_PER_A = 0.185;
const float AMP_PER_COUNT = (ADC_VREF / ADC_STEPS) / ACS_V_PER_A;
const float MAINS_HZ = 50.0;
const uint8_t CYCLES = 10;
const unsigned long WINDOW_US = (unsigned long)(CYCLES * 1000000.0 / MAINS_HZ);
const unsigned long MAX_SAMPLES = 3500UL;
const float I_NOISE_FLOOR = 0.03;

// ------------------------------ DETECTION CONFIG ---------------------------
const float   WINDOW_S   = 60.0;   // comparison window, seconds of meter time
const float   MIN_REF_WH = 0.5;    // skip a window below this reference energy
const float   TOL_PCT    = 3.0;    // PLACEHOLDER: derive from your normal runs
const uint8_t PERSIST_N  = 3;      // consecutive windows before ALERT

// ----------------------------------- STATE ---------------------------------
float vrms = 0, irms = 0, pAct = 0, pf = 0;      // latest reference measurement
double pSum = 0, vSum = 0, iSum = 0, pfSum = 0;  // since last frame
uint32_t nMeas = 0;

struct Frame { uint32_t seq, ms; float v, i, p, emWh, bal, acc; int relay, lab; };
char lineBuf[110];
uint8_t lineLen = 0;
bool frameReady = false;
Frame pending;

bool haveLast = false;
uint32_t lastMeterMs = 0;
double eRefWh = 0, eMeterWh = 0;
double winRef0 = 0, winMeter0 = 0, winSec = 0, totalSec = 0;
double sV = 0, sI = 0, sP = 0, sPF = 0;
uint32_t nWin = 0, lab1 = 0, winId = 0;
float prevU = 0;
uint8_t violateCount = 0, level = 0;
float lastU = 0;
int runId = 0, labelManual = -1;
String cmd;

// ------------------------------- SERIAL FRAME ------------------------------
bool parseFrame(char* s, Frame& f) {
  if (strncmp(s, "$M,", 3) != 0) return false;
  char* star = strchr(s, '*');
  if (!star) return false;
  *star = '\0';
  double vals[10];
  int n = 0;
  char* tok = strtok(s + 3, ",");
  while (tok && n < 10) { vals[n++] = atof(tok); tok = strtok(NULL, ","); }
  if (n != 10) return false;
  f.seq = (uint32_t)vals[0]; f.ms = (uint32_t)vals[1];
  f.v = vals[2]; f.i = vals[3]; f.p = vals[4]; f.emWh = vals[5]; f.bal = vals[6];
  f.relay = (int)vals[7]; f.lab = (int)vals[8]; f.acc = vals[9];
  return true;
}

// Called often (also inside the sampling loop) so the 64-byte RX buffer never overflows.
void pollLink() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        Frame f;
        if (parseFrame(lineBuf, f)) { pending = f; frameReady = true; }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}

// ------------------------------ REFERENCE MEASURE --------------------------
void measureRef() {
  long sumV = 0, sumI = 0, sumVV = 0, sumII = 0, sumVI = 0;
  unsigned long n = 0;
  unsigned long t0 = micros();
  while ((micros() - t0) < WINDOW_US && n < MAX_SAMPLES) {
    int v = analogRead(PIN_V) - 512;
    int i = analogRead(PIN_I) - 512;
    sumV += v; sumI += i;
    sumVV += (long)v * v; sumII += (long)i * i; sumVI += (long)v * i;
    n++;
    if ((n & 7) == 0) pollLink();
  }
  if (n < 10) return;
  float N = (float)n;
  float mV = sumV / N, mI = sumI / N;
  float varV = sumVV / N - mV * mV;
  float varI = sumII / N - mI * mI;
  float cov = sumVI / N - mV * mI;
  if (varV < 0) varV = 0;
  if (varI < 0) varI = 0;
  vrms = sqrt(varV) * VOLT_PER_COUNT;
  irms = sqrt(varI) * AMP_PER_COUNT;
  pAct = fabs(cov) * VOLT_PER_COUNT * AMP_PER_COUNT;
  float s = vrms * irms;
  if (irms < I_NOISE_FLOOR) { irms = 0; pAct = 0; pf = 0; }
  else { pf = (s > 0.001) ? pAct / s : 0; if (pf > 1.0) pf = 1.0; }
  pSum += pAct; vSum += vrms; iSum += irms; pfSum += pf; nMeas++;
}

// ------------------------------- DISPLAY / LEDS ----------------------------
void setLeds(uint8_t lv) {
  digitalWrite(LED_G, lv == 0); digitalWrite(LED_Y, lv == 1); digitalWrite(LED_R, lv == 2);
}
const char* statusText(uint8_t lv) {
  return lv == 0 ? "NORMAL" : (lv == 1 ? "SUSPECT" : "ALERT_POSSIBLE_UNDER_REGISTRATION");
}
void lcdLine(uint8_t row, const char* s) {
  char b[17]; snprintf(b, sizeof(b), "%-16s", s); lcd.setCursor(0, row); lcd.print(b);
}
void drawLcd() {
  char l0[24], l1[24], f[12];
  if (!haveLast) { lcdLine(0, "VERIFIER READY"); lcdLine(1, "waiting for meter"); return; }
  if (level == 0) snprintf(l0, sizeof(l0), "NORMAL");
  else if (level == 1) snprintf(l0, sizeof(l0), "SUSPECT n=%d", violateCount);
  else snprintf(l0, sizeof(l0), "ALERT UNDER-REG");
  dtostrf(lastU, 1, 1, f);
  snprintf(l1, sizeof(l1), "U:%s%% P:%dW", f, (int)(pAct + 0.5));
  lcdLine(0, l0); lcdLine(1, l1);
}

// --------------------------------- WINDOWS ---------------------------------
void resetWindow() {
  winRef0 = eRefWh; winMeter0 = eMeterWh; winSec = 0;
  sV = sI = sP = sPF = 0; nWin = 0; lab1 = 0;
}

void closeWindow() {
  double dRef = eRefWh - winRef0, dMet = eMeterWh - winMeter0;
  float n = nWin ? (float)nWin : 1.0f;
  float V = sV / n, I = sI / n, P = sP / n, PF = sPF / n;
  float ePct = 0, uPct = 0, dU = 0;
  bool valid = dRef >= MIN_REF_WH;
  if (valid) {
    ePct = (float)((dMet - dRef) / dRef * 100.0);
    uPct = -ePct;
    dU = uPct - prevU; prevU = uPct; lastU = uPct;
    if (uPct > TOL_PCT) violateCount++; else violateCount = 0;
  }
  level = (violateCount >= PERSIST_N) ? 2 : (violateCount >= 1 ? 1 : 0);
  setLeds(level);
  int label = (labelManual >= 0) ? labelManual : ((nWin > 0 && lab1 * 2 > nWin) ? 1 : 0);

  Serial.print(runId);        Serial.print(',');
  Serial.print(winId++);      Serial.print(',');
  Serial.print(totalSec, 1);  Serial.print(',');
  Serial.print(V, 1);         Serial.print(',');
  Serial.print(I, 3);         Serial.print(',');
  Serial.print(P, 1);         Serial.print(',');
  Serial.print(PF, 3);        Serial.print(',');
  Serial.print(50.0, 2);      Serial.print(',');
  Serial.print(dRef, 3);      Serial.print(',');
  Serial.print(dMet, 3);      Serial.print(',');
  if (valid) { Serial.print(ePct, 3); Serial.print(','); Serial.print(uPct, 3); Serial.print(','); Serial.print(dU, 3); }
  else { Serial.print(F("nan,nan,nan")); }
  Serial.print(',');
  Serial.print(violateCount); Serial.print(',');
  Serial.print(statusText(level)); Serial.print(',');
  Serial.println(label);
  resetWindow();
}

void handleFrame(const Frame& fr) {
  if (haveLast && (fr.emWh + 0.01f < eMeterWh || fr.ms < lastMeterMs)) haveLast = false;   // meter restarted
  if (!haveLast) {
    eMeterWh = fr.emWh; eRefWh = fr.emWh;
    pSum = vSum = iSum = pfSum = 0; nMeas = 0;
    resetWindow(); lastMeterMs = fr.ms; haveLast = true;
    return;
  }
  double dt = (fr.ms - lastMeterMs) / 1000.0;
  lastMeterMs = fr.ms;
  if (dt <= 0 || dt > 10.0) { pSum = vSum = iSum = pfSum = 0; nMeas = 0; return; }

  float n = nMeas ? (float)nMeas : 1.0f;
  float pAvg = pSum / n, vAvg = vSum / n, iAvg = iSum / n, pfAvg = pfSum / n;
  pSum = vSum = iSum = pfSum = 0; nMeas = 0;

  eRefWh += (double)pAvg * dt / 3600.0 * fr.acc;     // same time base as the meter
  eMeterWh = fr.emWh;
  sV += vAvg; sI += iAvg; sP += pAvg; sPF += pfAvg; nWin++;
  if (fr.lab) lab1++;
  winSec += dt; totalSec += dt;
  if (winSec >= WINDOW_S) closeWindow();
}

// ---------------------------------- COMMANDS --------------------------------
void handleCommand(String c) {
  c.trim();
  if (!c.length()) return;
  char k = toupper(c[0]);
  if (k == 'L' && c.length() >= 2) { labelManual = (c[1] == '1') ? 1 : 0; Serial.print(F("# label ")); Serial.println(labelManual); }
  else if (k == 'R' && c.length() >= 2) { runId = c.substring(1).toInt(); Serial.print(F("# run ")); Serial.println(runId); }
  else if (k == 'Z') { eRefWh = eMeterWh = 0; haveLast = false; violateCount = 0; prevU = 0; level = 0; setLeds(0); resetWindow(); Serial.println(F("# zeroed")); }
}

// ---------------------------------- SETUP ----------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(LED_G, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_R, OUTPUT);
  setLeds(0);
  lcd.begin(16, 2);
  lcdLine(0, "EXTERNAL VERIFIER"); lcdLine(1, "Proteus stand-in");
  delay(1500);
  Serial.println(F("# External verifier (Proteus stand-in for ESP32 + ATM90E26)"));
  Serial.print(F("# WINDOW_S="));  Serial.println(WINDOW_S);
  Serial.print(F("# TOL_PCT="));   Serial.println(TOL_PCT);
  Serial.print(F("# PERSIST_N=")); Serial.println(PERSIST_N);
  Serial.print(F("# VOLT_PER_COUNT=")); Serial.println(VOLT_PER_COUNT, 5);
  Serial.print(F("# AMP_PER_COUNT="));  Serial.println(AMP_PER_COUNT, 5);
  Serial.println(F("run,win,t_s,V,I,P,PF,F,dRef_Wh,dMeter_Wh,e_pct,U_pct,dU,count,status,label"));
  drawLcd();
}

// ----------------------------------- LOOP ----------------------------------
void loop() {
  measureRef();                         // ~200 ms, keeps draining the link
  pollLink();
  if (frameReady) { frameReady = false; handleFrame(pending); }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { handleCommand(cmd); cmd = ""; } else cmd += c;
  }
  drawLcd();
}
