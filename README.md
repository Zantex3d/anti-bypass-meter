# Build Guide and Test Plan: Prepaid Meter + External Verifier

## 0. What is in the pack

| File | Purpose |
|---|---|
| `01_Meter_Mega2560.ino` | Meter under test (Arduino Mega, Proteus) |
| `02_Verifier_ESP32_ATM90E26.ino` | Hardware verifier (ESP32 + ATM90E26) |
| `03_Analysis_and_AI.py` | Window analysis, threshold baseline, decision tree / random forest |
| `04_Value_Sheet.xlsx` | Every component value and calculation (formulas) |
| `04_Verifier_Proteus_Mega.ino` | Verifier stand-in for Proteus (second Mega) |
| `05_Schematic_Sheet1_Meter.svg` | Sheet 1: meter schematic |
| `06_Schematic_Sheet2_Verifier.svg` | Sheet 2: ESP32 + ATM90E26 hardware |
| `07_Schematic_Sheet3_Verifier_Proteus.svg` | Sheet 3: verifier you build in Proteus |

## 1. Meter (Sheet 1): build and check

Compile in the Arduino IDE (*Sketch → Export Compiled Binary*), put the `.hex` (without bootloader) in the Mega's *Program File*, clock 16 MHz.

| Stage | Add | Check |
|---|---|---|
| 1 | Mega, LCD, RV1, keypad | `Bal:N1000.00` appears |
| 2 | Keypad | Press `#`, type `123456789012`, press `#`: `TOKEN ACCEPTED` |
| 3 | Relay driver (R5, R6, Q1, D1, B1, RL1) | Coil voltmeter about 11.8 V once the balance screen shows |
| 4 | AC1, R7, TR1 divider | AC meter across secondary about 11.5 V; LCD `V:` about 230 V |
| 5 | ACS1, lamps, switches | Close SW1: about 0.43 A, 100 W; SW2, SW3 add 100 W each |
| 6 | Let it run | Balance falls; at ₦0 relay opens, `LOAD CUT OFF`; recharge reconnects |

Expected values (100 W lamps, 230 V) are on the value sheet: 0.435 / 0.870 / 1.304 A and 100 / 200 / 300 W. With `TIME_ACCEL = 600` the cost rate is ₦1.67 / 3.33 / 5.00 per simulated second.

**Keypad:** `#` starts token entry, `*` is backspace (exits when empty), `#` submits.
**Tokens:** `123456789012` ₦1000, `111111111111` ₦500, `222222222222` ₦2000, `333333333333` ₦100, `444444444444` ₦50, `555555555555` ₦5000. Each works once per run.

## 3. Verifier in Proteus (Sheet 3)

Proteus has no ATM90E26 or ESP32 model, so a second Mega (U3) acts as both: it measures the load with its own sensors and checks the meter. Steps:

1. Cut the wire from ACS1 IP− to the lamp bus. Wire ACS1 IP− → ACS2 IP+ and ACS2 IP− → lamp bus.
2. Build TR2, R10–R13, C5 exactly like the meter's voltage block.
3. Wire the LCD2, LEDs and VT2 as drawn. Join the GND of U1 and U3.
4. Wire **U1 D18 (TX1) → U3 D19 (RX1)**.
5. Load `04_Verifier_Proteus_Mega.ino` into U3.

Check: U3's LCD shows `NORMAL` after the first minute of simulation time. In VT2 you get one CSV row per 60 s window.

**Test under-registration:** set `UNDER_REG_PERCENT = 10.0` in the meter sketch, rebuild the `.hex`, run with lamps on. U3 goes SUSPECT after the first bad window and ALERT after three (red LED).

## 4. Hardware verifier (Sheet 2): bring-up and calibration

Treat this stage as **unverified until you have done each step**; the ATM90E26 section of the firmware follows the datasheet register map as recalled and is marked VERIFY in the code.

1. Start with `USE_SIM_REFERENCE 1` on the ESP32 and the meter frames connected: confirm CSV windows print and the LEDs work.
2. Switch to `USE_SIM_REFERENCE 0`. Confirm SPI works: `# ATM SysStatus` and `Urms raw` print non-zero, non-`0xFFFF` values. If not, try `SPI_MODE0`.
3. Add your module's configuration and calibration sequence where the code says so (vendor documentation).
4. **Calibrate against a reference instrument** (calibrated wattmeter or clamp meter) at a known load, then set `CAL_V`, `CAL_I`, `CAL_P`. Repeat at three load levels. Record the residual error: it is the reference uncertainty the thesis must report.
5. Set the meter's `TIME_ACCEL` to `1.0`; the hardware reference runs in real time.

**Safety:** mains tests only after the above work at low voltage, in a protected, fused, supervised arrangement. The ESP32 never touches mains. Never open-circuit a CT secondary under load.

## 5. Experiments and data

Use the same run structure in simulation and hardware:

1. Type `R<n>` in the verifier terminal to start a run (new number each time) and `L0` or `L1` to set the label.
2. Run each condition for at least 10 windows. Cover 1, 2 and 3 lamps (and near-zero load).
3. Normal runs: `UNDER_REG_PERCENT = 0`. Under-registration runs: 5, 10, 15, 20 %.
4. Save the terminal output to CSV files (one per session).
5. Work out the tolerance from normal runs (value sheet, `Tolerance` tab, or the Python script).
6. Analysis:

```
python 03_Analysis_and_AI.py analyze --csv run1.csv run2.csv run3.csv --persist 3
```

It splits the data **by run** (no leakage), reports detection rate, false-positive rate, precision, recall, F1 and the confusion matrix for the threshold method, a decision tree and a random forest.

To test the pipeline before you have data: `python 03_Analysis_and_AI.py synthetic --out s.csv`, then `analyze --csv s.csv`. Those numbers are **not** thesis results.

## 6. Mapping to your 12-week plan

| Weeks | Pack items |
|---|---|
| 1–2 | Literature; Stage 1–3 of section 2 |
| 3–4 | Sections 1 and 2 complete (tokens, balance, relay, lamps) |
| 5–6 | Section 4 steps 1–4 (ESP32 + ATM90E26, calibration) |
| 7–8 | Section 3 and the detection settings on the `Detection` tab |
| 9–10 | Section 5: runs and labelled data |
| 11 | `03_Analysis_and_AI.py` on real data; compare with baseline |
| 12 | Report limits (below) |

## 7. Limits to state in the thesis

- Proteus validates the prepaid logic and algorithm, not physical measurement accuracy.
- The Proteus verifier is an Arduino stand-in; the ESP32 / ATM90E26 results come only from the hardware stage.
- `TIME_ACCEL` makes simulated billing fast; hardware tests use 1.0.
- Tokens are a lookup table, not STS. Tariff and tolerance values are placeholders until measured.
- Labelled under-registration is deliberately induced (software), not a naturally occurring field fault.
