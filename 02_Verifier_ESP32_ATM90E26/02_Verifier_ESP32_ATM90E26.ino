/*
  ============================================================================
  EXTERNAL METER INTEGRITY VERIFIER  (System 2)  -  ESP32 + ATM90E26
  Thesis: AI-Assisted External Meter Integrity Verification System
  ============================================================================

  WHAT IT DOES
    1. Receives the energy REGISTERED by the meter under test over UART2
       (frames from the Mega: $M,seq,ms,V,I,P,E_meter_Wh,balance,relay,label,accel*).
    2. Obtains an INDEPENDENT reference measurement of the same load:
         USE_SIM_REFERENCE = 1 : synthesised from the frame (+ instrument noise)
                                 so the algorithm runs with no ATM90E26 hardware.
         USE_SIM_REFERENCE = 0 : read from the ATM90E26 over SPI.
    3. Integrates reference energy, then every WINDOW_S seconds compares
         e%  = (E_meter - E_ref) / E_ref * 100
         U%  = (E_ref - E_meter) / E_ref * 100      (positive = under-registration)
    4. Raises an alert only when U% exceeds TOL_PCT for PERSIST_N consecutive
       windows (threshold + persistence baseline).
    5. Prints one CSV row per window (this is the dataset for the AI stage).

  STATUS OUTPUT : LEDs  green = NORMAL, yellow = SUSPECT, red = ALERT

  PINS (ESP32 DevKit, 30-pin)
    UART2 RX  GPIO16  <- meter TX1 via 1k/2k divider (5 V -> 3.3 V)
    UART2 TX  GPIO17  (unused, reserved)
    SPI CS    GPIO5     SCK GPIO18     MISO GPIO19     MOSI GPIO23   (-> ATM90E26)
    LED green GPIO25    yellow GPIO26    red GPIO27   (each via 330 ohm)

  SERIAL COMMANDS (USB serial monitor, 115200)
    L0 / L1   set experiment label: 0 = normal, 1 = known under-registration
    R<n>      set run id (e.g. R7)  - keeps windows of one experiment together
    Z         zero the energy accumulators and window state

  !! HARDWARE NOT VERIFIED !!  The ATM90E26 section follows the register map
  as recalled from the datasheet. Every line marked VERIFY must be checked
  against the Microchip datasheet and your module's documentation before you
  trust the numbers, and the module needs its own configuration/calibration.
  ============================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ----------------------------- BACKEND SELECT ------------------------------
#define USE_SIM_REFERENCE 1       // 1 = simulated reference, 0 = ATM90E26 over SPI

// --------------------------------- PINS ------------------------------------
const int PIN_RX2   = 16;
const int PIN_TX2   = 17;
const int PIN_CS    = 5;
const int PIN_SCK   = 18;
const int PIN_MISO  = 19;
const int PIN_MOSI  = 23;
const int PIN_LED_G = 25;
const int PIN_LED_Y = 26;
const int PIN_LED_R = 27;

// ----------------------------- DETECTION CONFIG ----------------------------
const float   WINDOW_S    = 60.0;   // comparison window (seconds of meter time)
const float   MIN_REF_WH  = 0.5;    // skip window if reference energy is below this
const float   TOL_PCT     = 3.0;    // PLACEHOLDER: set from your normal-run data
const uint8_t PERSIST_N   = 3;      // consecutive violating windows before ALERT

// Simulated reference instrument noise (1-sigma, percent). Used only if
// USE_SIM_REFERENCE = 1. Replace with the measured ATM90E26 uncertainty.
const float   SIM_NOISE_PCT = 0.3;

// ----------------------------- ATM90E26 SETTINGS ---------------------------
#define ATM_SPI_MODE  SPI_MODE3     // VERIFY: try SPI_MODE0 if reads return 0x0000/0xFFFF
const uint32_t ATM_SPI_HZ = 200000;

// Register addresses (VERIFY each against the datasheet)
const uint8_t REG_SOFTRESET = 0x00;
const uint8_t REG_SYSSTATUS = 0x01;
const uint8_t REG_LASTDATA  = 0x06;
const uint8_t REG_IRMS      = 0x48;
const uint8_t REG_URMS      = 0x49;
const uint8_t REG_PMEAN     = 0x4A;
const uint8_t REG_FREQ      = 0x4C;
const uint8_t REG_POWERF    = 0x4D;

// Scale factors from raw register to engineering units (VERIFY), then the
// calibration factors you determine against a calibrated reference instrument.
const float ATM_V_LSB = 0.01f;     // volts per count
const float ATM_I_LSB = 0.001f;    // amps per count
const float ATM_P_LSB = 1.0f;      // watts per count (depends on configuration)
const float ATM_F_LSB = 0.01f;     // hertz per count
const float ATM_PF_LSB = 0.001f;
float CAL_V = 1.0f, CAL_I = 1.0f, CAL_P = 1.0f;

// ---------------------------------- TYPES ----------------------------------
struct Frame {
  uint32_t seq, ms;
  float v, i, p, emWh, bal, acc;
  int relay, lab;
};

struct Ref {
  float v, i, p, pf, f;
  bool ok;
};

// ---------------------------------- STATE ----------------------------------
char     lineBuf[160];
uint8_t  lineLen = 0;

bool     haveLast = false;
uint32_t lastMeterMs = 0;
uint32_t lastEspMs   = 0;
double   eRefWh = 0.0;        // integrated reference energy
double   eMeterWh = 0.0;      // meter-registered energy (from frames)

// window bookkeeping
double   winRef0 = 0, winMeter0 = 0;
double   winSec = 0;          // elapsed time in window (meter time)
double   sumV = 0, sumI = 0, sumP = 0, sumPF = 0, sumF = 0;
uint32_t nSamp = 0;
uint32_t lab1 = 0;          // frames in this window carrying label=1
float    prevU = 0.0f;
uint8_t  violateCount = 0;
uint32_t winId = 0;
int      runId = 0;
int      labelManual = -1;    // -1 = use label from frame
double   totalSec = 0;

// ------------------------------- RANDOM NOISE ------------------------------
float randn() {
  float u1 = (float)random(1, 10000) / 10000.0f;
  float u2 = (float)random(0, 10000) / 10000.0f;
  return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)PI * u2);
}

// ------------------------------ ATM90E26 DRIVER ----------------------------
// Frame format assumed: first byte = (R/W<<7 | address), then 16 data bits.
// VERIFY against the datasheet SPI timing section.
uint16_t atmRead(uint8_t addr) {
  SPI.beginTransaction(SPISettings(ATM_SPI_HZ, MSBFIRST, ATM_SPI_MODE));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(10);
  SPI.transfer(0x80 | (addr & 0x7F));
  delayMicroseconds(4);
  uint8_t hi = SPI.transfer(0x00);
  uint8_t lo = SPI.transfer(0x00);
  delayMicroseconds(10);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return ((uint16_t)hi << 8) | lo;
}

void atmWrite(uint8_t addr, uint16_t val) {
  SPI.beginTransaction(SPISettings(ATM_SPI_HZ, MSBFIRST, ATM_SPI_MODE));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(10);
  SPI.transfer(addr & 0x7F);
  SPI.transfer((uint8_t)(val >> 8));
  SPI.transfer((uint8_t)(val & 0xFF));
  delayMicroseconds(10);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void atmInit() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  atmWrite(REG_SOFTRESET, 0x789A);   // VERIFY soft-reset key
  delay(100);
  // >>> INSERT YOUR MODULE'S CONFIGURATION / CALIBRATION SEQUENCE HERE <<<
  // (metering constant, gains, thresholds, checksum registers per vendor docs)
  Serial.print(F("# ATM SysStatus=0x"));
  Serial.println(atmRead(REG_SYSSTATUS), HEX);
  Serial.print(F("# ATM Urms raw="));
  Serial.println(atmRead(REG_URMS));
}

Ref readAtm() {
  Ref r;
  r.v  = atmRead(REG_URMS) * ATM_V_LSB * CAL_V;
  r.i  = atmRead(REG_IRMS) * ATM_I_LSB * CAL_I;
  int16_t pm = (int16_t)atmRead(REG_PMEAN);
  r.p  = fabsf((float)pm * ATM_P_LSB * CAL_P);
  r.f  = atmRead(REG_FREQ) * ATM_F_LSB;
  int16_t pfRaw = (int16_t)atmRead(REG_POWERF);
  r.pf = fabsf((float)pfRaw * ATM_PF_LSB);
  r.ok = (r.v > 50.0f);              // crude sanity: mains should be present
  return r;
}

// ------------------------------ SIM REFERENCE ------------------------------
Ref readSim(const Frame& fr) {
  Ref r;
  float nz = SIM_NOISE_PCT / 100.0f;
  r.v  = fr.v * (1.0f + nz * randn());
  r.i  = fr.i * (1.0f + nz * randn());
  r.p  = fr.p * (1.0f + nz * randn());
  if (r.p < 0) r.p = 0;
  r.pf = (r.v * r.i > 1.0f) ? r.p / (r.v * r.i) : 0.0f;
  if (r.pf > 1.0f) r.pf = 1.0f;
  r.f  = 50.0f + 0.02f * randn();
  r.ok = true;
  return r;
}

// ------------------------------- FRAME PARSER ------------------------------
bool parseFrame(char* s, Frame& f) {
  if (strncmp(s, "$M,", 3) != 0) return false;
  char* star = strchr(s, '*');
  if (!star) return false;
  *star = '\0';
  double vals[10];
  int n = 0;
  char* save = NULL;
  for (char* tok = strtok_r(s + 3, ",", &save); tok && n < 10; tok = strtok_r(NULL, ",", &save)) {
    vals[n++] = atof(tok);
  }
  if (n != 10) return false;
  f.seq  = (uint32_t)vals[0];
  f.ms   = (uint32_t)vals[1];
  f.v    = (float)vals[2];
  f.i    = (float)vals[3];
  f.p    = (float)vals[4];
  f.emWh = (float)vals[5];
  f.bal  = (float)vals[6];
  f.relay = (int)vals[7];
  f.lab  = (int)vals[8];
  f.acc  = (float)vals[9];
  return true;
}

// -------------------------------- STATUS LEDS ------------------------------
void setLeds(uint8_t level) {   // 0 normal, 1 suspect, 2 alert
  digitalWrite(PIN_LED_G, level == 0);
  digitalWrite(PIN_LED_Y, level == 1);
  digitalWrite(PIN_LED_R, level == 2);
}

const char* statusText(uint8_t level) {
  return level == 0 ? "NORMAL" : (level == 1 ? "SUSPECT" : "ALERT_POSSIBLE_UNDER_REGISTRATION");
}

// ------------------------------ WINDOW HANDLING ----------------------------
void resetWindow() {
  winRef0 = eRefWh;
  winMeter0 = eMeterWh;
  winSec = 0;
  sumV = sumI = sumP = sumPF = sumF = 0;
  nSamp = 0;
  lab1 = 0;
}

void closeWindow() {
  int labelFromFrame = (nSamp > 0 && lab1 * 2 > nSamp) ? 1 : 0;   // majority of frames
  double dRef = eRefWh - winRef0;
  double dMet = eMeterWh - winMeter0;
  float n = (nSamp > 0) ? (float)nSamp : 1.0f;
  float V = sumV / n, I = sumI / n, P = sumP / n, PF = sumPF / n, F = sumF / n;

  float ePct = 0, uPct = 0, dU = 0;
  uint8_t level = 0;
  bool valid = (dRef >= MIN_REF_WH);

  if (valid) {
    ePct = (float)((dMet - dRef) / dRef * 100.0);
    uPct = (float)((dRef - dMet) / dRef * 100.0);
    dU = uPct - prevU;
    prevU = uPct;
    if (uPct > TOL_PCT) violateCount++;
    else violateCount = 0;
    if (violateCount >= PERSIST_N) level = 2;
    else if (violateCount >= 1)    level = 1;
  } else {
    // not enough energy for a meaningful comparison: keep state, do not alert
    level = (violateCount >= PERSIST_N) ? 2 : (violateCount >= 1 ? 1 : 0);
  }
  setLeds(level);

  int label = (labelManual >= 0) ? labelManual : labelFromFrame;

  Serial.print(runId);              Serial.print(',');
  Serial.print(winId++);            Serial.print(',');
  Serial.print(totalSec, 1);        Serial.print(',');
  Serial.print(V, 1);               Serial.print(',');
  Serial.print(I, 3);               Serial.print(',');
  Serial.print(P, 1);               Serial.print(',');
  Serial.print(PF, 3);              Serial.print(',');
  Serial.print(F, 2);               Serial.print(',');
  Serial.print(dRef, 3);            Serial.print(',');
  Serial.print(dMet, 3);            Serial.print(',');
  Serial.print(valid ? ePct : NAN, 3);   Serial.print(',');
  Serial.print(valid ? uPct : NAN, 3);   Serial.print(',');
  Serial.print(valid ? dU : NAN, 3);     Serial.print(',');
  Serial.print(violateCount);       Serial.print(',');
  Serial.print(statusText(level));  Serial.print(',');
  Serial.println(label);

  resetWindow();
}

// ------------------------------ FRAME HANDLING -----------------------------
void handleFrame(const Frame& fr) {
  // meter restarted -> resynchronise
  if (haveLast && (fr.emWh + 0.01f < eMeterWh || fr.ms < lastMeterMs)) {
    Serial.println(F("# meter reset detected: resynchronising"));
    haveLast = false;
  }

#if USE_SIM_REFERENCE
  Ref ref = readSim(fr);
  float accel = fr.acc;
#else
  Ref ref = readAtm();
  float accel = 1.0f;                       // physical test runs in real time
  static bool warned = false;
  if (!warned && fr.acc != 1.0f) {
    Serial.println(F("# WARNING: meter TIME_ACCEL != 1 but reference is real time"));
    warned = true;
  }
#endif

  uint32_t espNow = millis();
  if (!haveLast) {
    eMeterWh = fr.emWh;
    eRefWh = fr.emWh;                       // align start so only changes matter
    resetWindow();
    lastMeterMs = fr.ms;
    lastEspMs = espNow;
    haveLast = true;
    return;
  }

#if USE_SIM_REFERENCE
  double dt = (fr.ms - lastMeterMs) / 1000.0;     // meter (simulation) time
#else
  double dt = (espNow - lastEspMs) / 1000.0;      // real time
#endif
  lastMeterMs = fr.ms;
  lastEspMs = espNow;
  if (dt <= 0 || dt > 10.0) return;               // ignore gaps

  if (ref.ok) eRefWh += (double)ref.p * dt / 3600.0 * accel;
  eMeterWh = fr.emWh;

  sumV += ref.v; sumI += ref.i; sumP += ref.p; sumPF += ref.pf; sumF += ref.f;
  nSamp++;
  if (fr.lab) lab1++;
  winSec += dt;
  totalSec += dt;

  if (winSec >= WINDOW_S) closeWindow();
}

// ------------------------------ SERIAL COMMANDS ----------------------------
void handleCommand(String c) {
  c.trim();
  if (c.length() == 0) return;
  char k = toupper(c[0]);
  if (k == 'L' && c.length() >= 2) {
    labelManual = (c[1] == '1') ? 1 : 0;
    Serial.print(F("# label set to ")); Serial.println(labelManual);
  } else if (k == 'R' && c.length() >= 2) {
    runId = c.substring(1).toInt();
    Serial.print(F("# run id set to ")); Serial.println(runId);
  } else if (k == 'Z') {
    eRefWh = eMeterWh = 0; haveLast = false; violateCount = 0; prevU = 0;
    resetWindow();
    Serial.println(F("# accumulators zeroed"));
  }
}

// ---------------------------------- SETUP ----------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  setLeds(0);
  randomSeed(esp_random());

#if !USE_SIM_REFERENCE
  atmInit();
#endif

  Serial.println(F("# External meter integrity verifier"));
  Serial.print(F("# WINDOW_S="));   Serial.println(WINDOW_S);
  Serial.print(F("# TOL_PCT="));    Serial.println(TOL_PCT);
  Serial.print(F("# PERSIST_N="));  Serial.println(PERSIST_N);
  Serial.print(F("# MIN_REF_WH=")); Serial.println(MIN_REF_WH);
  Serial.print(F("# BACKEND="));    Serial.println(USE_SIM_REFERENCE ? F("SIM") : F("ATM90E26"));
  Serial.println(F("run,win,t_s,V,I,P,PF,F,dRef_Wh,dMeter_Wh,e_pct,U_pct,dU,count,status,label"));
}

// ----------------------------------- LOOP ----------------------------------
void loop() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        Frame fr;
        if (parseFrame(lineBuf, fr)) handleFrame(fr);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // overflow: drop line
    }
  }

  static String cmd;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { handleCommand(cmd); cmd = ""; }
    else cmd += c;
  }
}
