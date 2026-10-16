#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 sensor;

// ---------- UNO safe ----------
#define BUFFER_SIZE 40
#define NEW_SAMPLES 8

uint16_t irBuf[BUFFER_SIZE];
uint16_t redBuf[BUFFER_SIZE];

int32_t spo2 = 0;
int8_t  validSPO2 = 0;
int32_t heartRate = 0;
int8_t  validHeartRate = 0;

// ---------- Timing ----------
static const unsigned long PRESENT_HOLD_MS = 250;
static const unsigned long WARMUP_MS       = 900;
static const unsigned long LOST_GRACE_MS   = 900;
static const unsigned long LCD_PERIOD_MS   = 350;

unsigned long lastLcdMs = 0;
unsigned long fingerPresentSince = 0;
unsigned long fingerLastSeenMs   = 0;
unsigned long warmupStartMs      = 0;

bool measuring = false;
bool bufferPrimed = false;

int lastGoodBPM  = -1;
int lastGoodSpO2 = -1;

// ---------- Finger detect using RAW IR (no big arrays) ----------
uint32_t irBaseRaw = 0;
uint32_t threshOnRaw  = 0;
uint32_t threshOffRaw = 0;

// Tune margins (RAW):
// If finger NOT detected: lower ON margin (e.g. 15000)
// If detects finger w/o finger: increase ON margin (e.g. 60000)
static const uint32_t MARGIN_ON_RAW  = 25000;
static const uint32_t MARGIN_OFF_RAW = 12000;

// ---------- Scaling to 16-bit for your uint16 algorithm ----------
static inline uint16_t to16(uint32_t v) {
  v >>= 1;                 // stronger than >>2; helps avoid always "--"
  if (v > 65535UL) v = 65535UL;
  return (uint16_t)v;
}

bool waitForSample(uint16_t timeoutMs) {
  unsigned long t0 = millis();
  while (!sensor.available()) {
    sensor.check();
    if (millis() - t0 > timeoutMs) return false;
  }
  return true;
}

uint32_t readIrRawAvg(uint8_t n) {
  uint64_t sum = 0;
  uint8_t got = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (!waitForSample(300)) break;
    sum += sensor.getIR();
    sensor.nextSample();
    got++;
  }
  if (!got) return 0;
  return (uint32_t)(sum / got);
}

void showNoFinger() {
  lcd.setCursor(0,0); lcd.print(F("No finger       "));
  lcd.setCursor(0,1); lcd.print(F("Place finger..  "));
}

void showMeasuring() {
  lcd.setCursor(0,0); lcd.print(F("Measuring...    "));
  lcd.setCursor(0,1); lcd.print(F("Hold still      "));
}

void showValues(int bpm, int s) {
  lcd.setCursor(0,0);
  lcd.print(F("BPM: "));
  if (bpm < 0) lcd.print(F("-- "));
  else {
    if (bpm < 100) lcd.print(' ');
    if (bpm < 10)  lcd.print(' ');
    lcd.print(bpm);
  }
  lcd.print(F("      "));

  lcd.setCursor(0,1);
  lcd.print(F("SpO2: "));
  if (s < 0) lcd.print(F("--%     "));
  else {
    if (s < 100) lcd.print(' ');
    if (s < 10)  lcd.print(' ');
    lcd.print(s);
    lcd.print(F("%     "));
  }
}

void resetAll() {
  measuring = false;
  bufferPrimed = false;
  fingerPresentSince = 0;
  warmupStartMs = 0;
  lastGoodBPM = -1;
  lastGoodSpO2 = -1;
  showNoFinger();
}

bool fillInitialBuffer() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (!waitForSample(350)) return false;
    redBuf[i] = to16(sensor.getRed());
    irBuf[i]  = to16(sensor.getIR());
    sensor.nextSample();
  }
  return true;
}

bool slideAndAdd() {
  for (int i = 0; i < BUFFER_SIZE - NEW_SAMPLES; i++) {
    redBuf[i] = redBuf[i + NEW_SAMPLES];
    irBuf[i]  = irBuf[i + NEW_SAMPLES];
  }
  for (int i = BUFFER_SIZE - NEW_SAMPLES; i < BUFFER_SIZE; i++) {
    if (!waitForSample(350)) return false;
    redBuf[i] = to16(sensor.getRed());
    irBuf[i]  = to16(sensor.getIR());
    sensor.nextSample();
  }
  return true;
}

void computeAndStore() {
  maxim_heart_rate_and_oxygen_saturation(
    irBuf, BUFFER_SIZE,
    redBuf,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );

  if (validHeartRate && heartRate >= 30 && heartRate <= 220) lastGoodBPM = (int)heartRate;
  if (validSPO2 && spo2 >= 0 && spo2 <= 100) lastGoodSpO2 = (int)spo2;
}

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Pulse Oximeter  "));
  lcd.setCursor(0,1); lcd.print(F("Init MAX30102   "));
  delay(400);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    lcd.clear(); lcd.print(F("MAX30102 FAIL"));
    while (1);
  }

  // IMPORTANT: avoid saturation (common cause of invalid while finger ON)
  // adcRange set higher (16384) and LED amplitude moderate
  sensor.setup(60, 4, 2, 200, 411, 16384);
  sensor.setPulseAmplitudeIR(0x50);
  sensor.setPulseAmplitudeRed(0x50);
  sensor.setPulseAmplitudeGreen(0);

  // RAW baseline calibration (no finger)
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Calibrating...  "));
  lcd.setCursor(0,1); lcd.print(F("No finger       "));

  uint64_t sum = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 1200) {
    uint32_t ir = readIrRawAvg(1);
    if (ir == 0) continue;
    sum += ir;
    n++;
  }
  irBaseRaw = (n > 0) ? (uint32_t)(sum / (uint64_t)n) : 0;
  threshOnRaw  = irBaseRaw + MARGIN_ON_RAW;
  threshOffRaw = irBaseRaw + MARGIN_OFF_RAW;

  lcd.clear();
  resetAll();
}

void loop() {
  // Finger detect with hysteresis (RAW)
  uint32_t irNowRaw = readIrRawAvg(2);
  static bool fingerState = false;

  if (!fingerState) {
    if (irNowRaw > threshOnRaw) fingerState = true;
  } else {
    if (irNowRaw < threshOffRaw) fingerState = false;
  }

  if (fingerState) {
    fingerLastSeenMs = millis();
    if (fingerPresentSince == 0) fingerPresentSince = millis();
  } else {
    fingerPresentSince = 0;
  }

  // Not measuring yet
  if (!measuring) {
    if (!fingerState) {
      if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
        showNoFinger();
        lastLcdMs = millis();
      }
      return;
    }

    if (millis() - fingerPresentSince < PRESENT_HOLD_MS) {
      if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
        showMeasuring();
        lastLcdMs = millis();
      }
      return;
    }

    measuring = true;
    warmupStartMs = millis();
    bufferPrimed = false;
    lastGoodBPM = -1;
    lastGoodSpO2 = -1;
  }

  // Measuring: reset only if missing long enough
  if (!fingerState && (millis() - fingerLastSeenMs > LOST_GRACE_MS)) {
    resetAll();
    return;
  }

  // Warmup
  if (millis() - warmupStartMs < WARMUP_MS) {
    if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
      showMeasuring();
      lastLcdMs = millis();
    }
    return;
  }

  // Prime buffer once
  if (!bufferPrimed) {
    if (!fillInitialBuffer()) { resetAll(); return; }
    computeAndStore();
    showValues(lastGoodBPM, lastGoodSpO2);
    lastLcdMs = millis();
    bufferPrimed = true;
    return;
  }

  // Continuous updates
  if (!slideAndAdd()) { resetAll(); return; }
  computeAndStore();

  if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
    showValues(lastGoodBPM, lastGoodSpO2);
    lastLcdMs = millis();
  }
}
