/*
  ============================================================================
  PREPAID SINGLE-PHASE ENERGY METER  -  "METER UNDER TEST" (System 1)  v3
  Arduino Mega 2560  |  Proteus VSM simulation
  Thesis: AI-Assisted External Meter Integrity Verification System
  ============================================================================

  PIN MAP
    LCD (LM044L 16x2) : RS=D22  E=D23  D4=D24  D5=D25  D6=D26  D7=D27  (RW=GND)
    Keypad 4x3        : rows A,B,C,D = D30,D31,D32,D33
                        columns 1,2,3 = D34,D35,D36
    Relay driver      : D7 -> 10k -> BC547 base
    Voltage sense     : A0  (transformer + divider, 2.5 V bias)
    Current sense     : A1  (ACS712, 2.5 V at zero current)
    Serial log        : TX0 (D1), 115200 baud (Proteus Virtual Terminal / USB)
    Link to verifier  : TX1 (D18), 115200 baud -> 5V/3.3V divider -> ESP32 RX2

  KEYS
    Normal mode : '#'  = start token entry
    Token mode  : 0-9  = type digit,  '*' = backspace (exit if empty),
                  '#'  = submit (12 digits)

  EQUATIONS (see design record)
    Vrms = sqrt( mean[(v - mean v)^2] ) * VOLT_PER_COUNT
    Irms = sqrt( mean[(i - mean i)^2] ) * AMP_PER_COUNT
    P    = | mean[(v - mean v)(i - mean i)] | * VOLT_PER_COUNT * AMP_PER_COUNT
    PF   = P / (Vrms * Irms)
    dE   = P * dt / 1000            [kWh, dt in hours]
    Cost = dE * TARIFF              [Naira]
    Balance_new = Balance_old - Cost
  ============================================================================
*/

#include <LiquidCrystal.h>

// ------------------------------- PINS --------------------------------------
const uint8_t PIN_RELAY = 7;
const uint8_t PIN_V     = A0;
const uint8_t PIN_I     = A1;

LiquidCrystal lcd(22, 23, 24, 25, 26, 27);   // RS, E, D4, D5, D6, D7

const uint8_t ROW_PINS[4] = {30, 31, 32, 33};   // rows A,B,C,D
const uint8_t COL_PINS[3] = {34, 35, 36};       // columns 1,2,3
const char KEYMAP[4][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// --------------------------- CALIBRATION CONSTANTS -------------------------
// ADC
const float ADC_VREF  = 5.0;       // volts
const float ADC_STEPS = 1023.0;

// Voltage channel: 230 V -> TR1 (ratio 20:1) -> divider (R1 68k, R2 10k)
const float TRAFO_RATIO = 20.0;                    // primary : secondary
const float DIV_RATIO   = (68.0 + 10.0) / 10.0;    // (R1+R2)/R2 = 7.8
const float VOLT_PER_COUNT = (ADC_VREF / ADC_STEPS) * DIV_RATIO * TRAFO_RATIO;

// Current channel: ACS712-05B = 185 mV/A (use 0.100 for 20A, 0.066 for 30A)
const float ACS_V_PER_A   = 0.185;
const float AMP_PER_COUNT = (ADC_VREF / ADC_STEPS) / ACS_V_PER_A;

// Measurement window: 10 whole mains cycles
const float         MAINS_HZ    = 50.0;
const uint8_t       CYCLES      = 10;
const unsigned long WINDOW_US   = (unsigned long)(CYCLES * 1000000.0 / MAINS_HZ);
const unsigned long MAX_SAMPLES = 3500UL;          // guards 32-bit sums

const float I_NOISE_FLOOR = 0.03;                  // A, below this = no load

// ------------------------- TEST / DATA-GENERATION OPTIONS -------------------
// USE_TEST_INPUTS = true : no transformer / ACS712 needed. Voltage is fixed and
//   the load is chosen with keys 0-3 in normal mode (0 to 3 lamps of
//   TEST_LAMP_WATTS each). Set false to use the A0/A1 sensor circuits.
const bool  USE_TEST_INPUTS = false;   // false = real A0/A1 sensor circuits
const float TEST_VOLTAGE    = 230.0;   // V
const float TEST_LAMP_WATTS = 100.0;   // W per lamp

// Known under-registration for labelled data: the meter registers this
// percentage LESS energy than was really delivered. 0 = healthy meter.
const float UNDER_REG_PERCENT = 0.0;

// ------------------------------- BILLING -----------------------------------
const float TARIFF          = 100.0;    // Naira per kWh
const float INITIAL_BALANCE = 1000.0;   // Naira (set 0.0 to demo token recharge)
const float LOW_BALANCE     = 100.0;    // Naira, low-credit warning below this

// DEMO TIME ACCELERATION: 1.0 = real time. 600.0 means 1 simulated second is
// billed as 600 seconds (10 min) so balance visibly falls in Proteus.
// Set to 1.0 for true-time validation. Keep identical for the reference system.
const float TIME_ACCEL = 600.0;

// ------------------------------- TOKENS ------------------------------------
// Controlled token table (demonstration only, not STS).
struct Token { const char* code; uint16_t credit; };
const Token TOKENS[] = {
  {"123456789012", 1000},
  {"111111111111",  500},
  {"222222222222", 2000},
  {"333333333333",  100},
  {"444444444444",   50},
  {"555555555555", 5000}
};
const uint8_t N_TOKENS = sizeof(TOKENS) / sizeof(TOKENS[0]);
bool tokenUsed[N_TOKENS];

// ------------------------------- STATE -------------------------------------
float vrms = 0, irms = 0, pAct = 0, pf = 0;
uint32_t frameSeq = 0;
void sendFrame();
float energyKWh   = 0;       // energy REGISTERED by the meter (E_meter)
float energyRefKWh = 0;      // energy actually delivered (what a reference sees)
uint8_t lampsOn   = 0;       // test-input mode: number of lamps on
float totalCost   = 0;       // accumulated cost
float balance     = INITIAL_BALANCE;
bool  relayOn     = false;
unsigned long lastBillMs = 0;
unsigned long lastLogMs  = 0;

// UI
enum Mode { MODE_NORMAL, MODE_TOKEN, MODE_MESSAGE };
Mode mode = MODE_NORMAL;
char tokenBuf[13];
uint8_t tokenLen = 0;
unsigned long msgUntil = 0;
unsigned long tokenLastKeyMs = 0;

// Key queue
char keyQ[8];
uint8_t qHead = 0, qTail = 0;

// ------------------------------ KEYPAD -------------------------------------
void pushKey(char k) {
  uint8_t next = (qHead + 1) % 8;
  if (next != qTail) { keyQ[qHead] = k; qHead = next; }
}

bool popKey(char* k) {
  if (qHead == qTail) return false;
  *k = keyQ[qTail];
  qTail = (qTail + 1) % 8;
  return true;
}

// Called every ~10 ms. A key is accepted when two consecutive scans agree
// (debounce) and it differs from the last accepted state (edge detect).
void scanKeypad() {
  static char lastRaw = 0;
  static char stable  = 0;
  char raw = 0;
  for (uint8_t r = 0; r < 4; r++) {
    digitalWrite(ROW_PINS[r], LOW);
    delayMicroseconds(5);
    for (uint8_t c = 0; c < 3; c++) {
      if (digitalRead(COL_PINS[c]) == LOW) raw = KEYMAP[r][c];
    }
    digitalWrite(ROW_PINS[r], HIGH);
  }
  if (raw == lastRaw && raw != stable) {
    stable = raw;
    if (raw != 0) pushKey(raw);
  }
  lastRaw = raw;
}

// ------------------------------ MEASUREMENT --------------------------------
void measure() {
  if (USE_TEST_INPUTS) {
    unsigned long t0 = millis();
    while (millis() - t0 < 200) { scanKeypad(); delay(10); }
    if (!relayOn || lampsOn == 0) {
      vrms = TEST_VOLTAGE; irms = 0; pAct = 0; pf = 0;
    } else {
      vrms = TEST_VOLTAGE;
      pAct = lampsOn * TEST_LAMP_WATTS;
      irms = pAct / vrms;
      pf   = 1.0;
    }
    return;
  }
  long sumV = 0, sumI = 0, sumVV = 0, sumII = 0, sumVI = 0;
  unsigned long n = 0;
  unsigned long t0 = micros();
  unsigned long lastScan = t0;

  while ((micros() - t0) < WINDOW_US && n < MAX_SAMPLES) {
    int v = analogRead(PIN_V) - 512;
    int i = analogRead(PIN_I) - 512;
    sumV  += v;
    sumI  += i;
    sumVV += (long)v * v;
    sumII += (long)i * i;
    sumVI += (long)v * i;
    n++;
    if ((micros() - lastScan) >= 10000UL) {
      lastScan = micros();
      scanKeypad();
    }
  }
  if (n < 10) return;

  float N  = (float)n;
  float mV = sumV / N;
  float mI = sumI / N;
  float varV = sumVV / N - mV * mV;
  float varI = sumII / N - mI * mI;
  float cov  = sumVI / N - mV * mI;
  if (varV < 0) varV = 0;
  if (varI < 0) varI = 0;

  vrms = sqrt(varV) * VOLT_PER_COUNT;
  irms = sqrt(varI) * AMP_PER_COUNT;
  pAct = fabs(cov) * VOLT_PER_COUNT * AMP_PER_COUNT;   // fabs: polarity independent

  if (!relayOn || irms < I_NOISE_FLOOR) {
    irms = 0; pAct = 0; pf = 0;
  } else {
    float s = vrms * irms;
    pf = (s > 0.001) ? pAct / s : 0.0;      // guard: no voltage reading -> PF 0, not NAN
    if (pf > 1.0) pf = 1.0;
  }
}

// -------------------------------- BILLING ----------------------------------
void updateRelay() {
  bool want = (balance > 0.0);
  if (want != relayOn) {
    relayOn = want;
    digitalWrite(PIN_RELAY, relayOn ? HIGH : LOW);
  }
}

void updateBilling() {
  unsigned long now = millis();
  float dt_h = ((now - lastBillMs) / 3600000.0) * TIME_ACCEL;
  lastBillMs = now;
  if (!relayOn) return;

  float dETrue  = pAct * dt_h / 1000.0;                      // delivered, kWh
  float dEMeter = dETrue * (1.0 - UNDER_REG_PERCENT / 100.0); // registered, kWh
  float cost    = dEMeter * TARIFF;                          // Naira
  if (cost <= 0.0) { energyRefKWh += dETrue; return; }
  if (cost >= balance) {                  // credit exhausted this step
    float ratio = balance / cost;
    dETrue  *= ratio;
    dEMeter *= ratio;
    cost = balance;
    balance = 0.0;
  } else {
    balance -= cost;
  }
  energyKWh    += dEMeter;
  energyRefKWh += dETrue;
  totalCost    += cost;
}

// ---------------------------------- LCD ------------------------------------
void lcdLine(uint8_t row, const char* s) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", s);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void showMessage(const char* a, const char* b, unsigned long ms) {
  mode = MODE_MESSAGE;
  msgUntil = millis() + ms;
  lcdLine(0, a);
  lcdLine(1, b);
}

void drawNormal() {
  char l0[24], l1[24], f[12];
  bool blink = ((millis() / 1000) % 2) != 0;

  if (balance <= 0.0) {
    lcdLine(0, "BALANCE: N0.00");
    lcdLine(1, blink ? "LOAD CUT OFF" : "Press # recharge");
    return;
  }

  uint8_t page = (millis() / 3000) % 3;
  if (page == 0) {
    dtostrf(balance, 1, 2, f);
    snprintf(l0, sizeof(l0), "Bal:N%s", f);
    dtostrf(energyKWh, 1, 4, f);
    snprintf(l1, sizeof(l1), "E:%s kWh", f);
  } else if (page == 1) {
    dtostrf(vrms, 1, 1, f);
    snprintf(l0, sizeof(l0), "V: %s V", f);
    dtostrf(irms, 1, 3, f);
    snprintf(l1, sizeof(l1), "I: %s A", f);
  } else {
    dtostrf(pAct, 1, 1, f);
    snprintf(l0, sizeof(l0), "P: %s W", f);
    dtostrf(pf, 1, 2, f);
    snprintf(l1, sizeof(l1), "PF:%s T:N%d", f, (int)TARIFF);
    if (USE_TEST_INPUTS) snprintf(l1, sizeof(l1), "Lamps:%d key 0-3", lampsOn);
  }
  if (balance < LOW_BALANCE && blink) {
    snprintf(l1, sizeof(l1), "* LOW BALANCE! *");
  }
  lcdLine(0, l0);
  lcdLine(1, l1);
}

void drawToken() {
  char l1[24];
  lcdLine(0, "Enter Token:");
  snprintf(l1, sizeof(l1), "%s_", tokenBuf);
  lcdLine(1, l1);
}

void uiUpdate() {
  if (mode == MODE_MESSAGE) {
    if (millis() > msgUntil) mode = MODE_NORMAL;
    else return;
  }
  if (mode == MODE_TOKEN) {
    if (millis() - tokenLastKeyMs > 30000UL) {   // 30 s inactivity
      mode = MODE_NORMAL;
    } else {
      drawToken();
      return;
    }
  }
  drawNormal();
}

// ------------------------------ TOKEN ENTRY --------------------------------
void submitToken() {
  if (tokenLen != 12) {
    showMessage("INVALID TOKEN", "Need 12 digits", 2000);
    return;
  }
  int idx = -1;
  for (uint8_t k = 0; k < N_TOKENS; k++) {
    if (strcmp(tokenBuf, TOKENS[k].code) == 0) { idx = k; break; }
  }
  if (idx < 0) {
    showMessage("INVALID TOKEN", "Not recognised", 2000);
  } else if (tokenUsed[idx]) {
    showMessage("TOKEN USED", "Already redeemed", 2000);
  } else {
    tokenUsed[idx] = true;
    balance += TOKENS[idx].credit;
    updateRelay();                       // reconnect load if it was cut off
    char l1[24];
    snprintf(l1, sizeof(l1), "Added: N%u", TOKENS[idx].credit);
    showMessage("TOKEN ACCEPTED", l1, 2500);
    Serial.print(F("#TOKEN_ACCEPTED,credit="));
    Serial.println(TOKENS[idx].credit);
  }
}

void handleKey(char k) {
  if (mode == MODE_MESSAGE) return;

  if (mode == MODE_NORMAL) {
    if (USE_TEST_INPUTS && k >= '0' && k <= '3') { lampsOn = k - '0'; return; }
    if (k == '#') {
      mode = MODE_TOKEN;
      tokenLen = 0;
      tokenBuf[0] = '\0';
      tokenLastKeyMs = millis();
    }
    return;
  }

  // MODE_TOKEN
  tokenLastKeyMs = millis();
  if (k >= '0' && k <= '9') {
    if (tokenLen < 12) {
      tokenBuf[tokenLen++] = k;
      tokenBuf[tokenLen] = '\0';
    }
  } else if (k == '*') {
    if (tokenLen > 0) {
      tokenBuf[--tokenLen] = '\0';
    } else {
      mode = MODE_NORMAL;
    }
  } else if (k == '#') {
    submitToken();
    tokenLen = 0;
    tokenBuf[0] = '\0';
  }
}

// ------------------------------ SERIAL LOG ---------------------------------
void logSerial() {
  if (millis() - lastLogMs < 1000UL) return;
  lastLogMs = millis();
  sendFrame();
  Serial.print(millis() / 1000.0, 1);  Serial.print(',');
  Serial.print(vrms, 1);               Serial.print(',');
  Serial.print(irms, 3);               Serial.print(',');
  Serial.print(pAct, 1);               Serial.print(',');
  Serial.print(pf, 3);                 Serial.print(',');
  Serial.print(energyKWh, 5);          Serial.print(',');
  Serial.print(energyRefKWh, 5);       Serial.print(',');
  Serial.print(totalCost, 3);          Serial.print(',');
  Serial.print(balance, 3);            Serial.print(',');
  Serial.print(relayOn ? 1 : 0);       Serial.print(',');
  Serial.println(UNDER_REG_PERCENT > 0.0 ? 1 : 0);   // label: 1 = under-registering
}

// --------------------- FRAME TO THE EXTERNAL VERIFIER ----------------------
// $M,seq,ms,V,I,P,E_meter_Wh,balance,relay,label,time_accel*
// 'label' is for dataset labelling only; the verifier never uses it to decide.
void sendFrame() {
  Serial1.print(F("$M,"));
  Serial1.print(frameSeq++);                        Serial1.print(',');
  Serial1.print(millis());                          Serial1.print(',');
  Serial1.print(vrms, 1);                           Serial1.print(',');
  Serial1.print(irms, 3);                           Serial1.print(',');
  Serial1.print(pAct, 1);                           Serial1.print(',');
  Serial1.print(energyKWh * 1000.0, 3);             Serial1.print(',');
  Serial1.print(balance, 2);                        Serial1.print(',');
  Serial1.print(relayOn ? 1 : 0);                   Serial1.print(',');
  Serial1.print(UNDER_REG_PERCENT > 0.0 ? 1 : 0);   Serial1.print(',');
  Serial1.print(TIME_ACCEL, 1);
  Serial1.println('*');
}

// ---------------------------------- SETUP ----------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  for (uint8_t r = 0; r < 4; r++) {
    pinMode(ROW_PINS[r], OUTPUT);
    digitalWrite(ROW_PINS[r], HIGH);
  }
  for (uint8_t c = 0; c < 3; c++) pinMode(COL_PINS[c], INPUT_PULLUP);

  lcd.begin(16, 2);
  lcdLine(0, "PREPAID METER");
  lcdLine(1, "UNDER TEST v1.0");
  delay(1500);

  if (TIME_ACCEL != 1.0) {
    char b[24];
    snprintf(b, sizeof(b), "Time accel x%d", (int)TIME_ACCEL);
    lcdLine(0, "DEMO MODE");
    lcdLine(1, b);
    delay(1500);
  }

  for (uint8_t k = 0; k < N_TOKENS; k++) tokenUsed[k] = false;

  updateRelay();
  lastBillMs = millis();
  lastLogMs  = millis();

  Serial.println(F("# Prepaid meter under test v1.0"));
  Serial.print(F("# TARIFF_N_per_kWh=")); Serial.println(TARIFF);
  Serial.print(F("# TIME_ACCEL="));       Serial.println(TIME_ACCEL);
  Serial.print(F("# UNDER_REG_PERCENT=")); Serial.println(UNDER_REG_PERCENT);
  Serial.print(F("# USE_TEST_INPUTS="));  Serial.println(USE_TEST_INPUTS ? 1 : 0);
  Serial.print(F("# VOLT_PER_COUNT="));   Serial.println(VOLT_PER_COUNT, 5);
  Serial.print(F("# AMP_PER_COUNT="));    Serial.println(AMP_PER_COUNT, 5);
  Serial.println(F("t_s,Vrms,Irms,P_W,PF,E_meter_kWh,E_ref_kWh,Cost_N,Balance_N,Relay,Label"));
}

// ----------------------------------- LOOP ----------------------------------
void loop() {
  measure();            // ~200 ms (10 mains cycles); scans keypad meanwhile
  updateBilling();
  updateRelay();

  char k;
  while (popKey(&k)) handleKey(k);

  uiUpdate();
  logSerial();
}
