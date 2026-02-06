#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ====== Pins (FIXED: your original) ======
#define ONE_WIRE_BUS     15
#define VOLTAGE_PIN      34
#define I2C_SDA          21
#define I2C_SCL          22
#define MOSFET_GATE_PIN  25
#define BUTTON_PIN       33   // INPUT_PULLUP, pressed = LOW
#define LED_R_PIN        26
#define LED_G_PIN        27
#define LED_B_PIN        14

// ====== Scheduling periods (task rates) ======
static const uint32_t INA_PERIOD_MS      = 100;    // read INA219 every 100ms
static const uint32_t LOG_PERIOD_MS      = 500;    // print every 500ms
static const uint32_t TEMP_PERIOD_MS     = 1000;   // start temp conversion every 1s (async read later)

// DS18B20 conversion time (worst-case 12-bit ~750ms)
static const uint32_t DS18B20_CONV_MS    = 750;

// ====== Other timings ======
static const uint32_t COOLING_MS         = 5000;   // stay in COOLING before back to IDLE
static const uint32_t INIT_WARMUP_MS     = 1500;   // small warmup time

// ====== PROTECT LED blink timing ======
static const uint32_t PROTECT_BLINK_TOTAL_MS  = 2000;  // blink red for 2 seconds
static const uint32_t PROTECT_BLINK_PERIOD_MS = 200;   // blink interval (200ms)

// ====== Voltage sensing (just display) ======
static const float ADC_VREF              = 3.3f;
static const int   ADC_MAX               = 4095;

// ====== Power present (adapter/battery) detection ======
static const float BUSV_VALID_V          = 1.0f;     // BusV must be above this to allow scenario
static const uint32_t BUSV_STABLE_MS     = 300;      // require stable BusV for this long

// ====== Scenario A: press -> increasing load (PWM) -> auto cutoff ======
static const uint32_t RAMP_TO_MAX_MS     = 6000;   // time to reach max duty while holding
static const float    DUTY_MIN           = 0.20f;  // starting duty (20%)
static const float    DUTY_MAX           = 1.00f;  // max duty (100%)

// ====== Stress logic (sensor-based) ======
static const float I_TH_mA               = 15.0f;     // tuned to your current range
static const float SCORE_LIMIT_mAs       = 120.0f;    // tuned to trip around ~10s at ~36mA
static const float TEMP_LIMIT_C          = 55.0f;     // safety temp limit (later tune)

// Optional: safety cap
static const uint32_t MAX_SCENARIO_MS    = 20000;     // force protect after 20s

// ====== PWM settings (ESP32 Arduino core v3.x compatible) ======
static const int PWM_FREQ_HZ             = 20000;   // 20kHz
static const int PWM_RES_BITS            = 10;      // 0..1023

// ====== WARN throttling ======
static uint32_t lastWarnMs               = 0;
static const uint32_t WARN_PERIOD_MS     = 500;

// ====== INA219 current offset calibration ======
static float I_OFFSET_mA = 0.0f;
static bool  offset_calibrated = false;
static const int OFFSET_SAMPLE_COUNT = 50;     // 50 samples * 20ms = 1s 정도
static const uint32_t OFFSET_ARM_DELAY_MS = 300; // IDLE 진입 후 안정화 대기(간단)

// ====== State machine ======
enum State {
  INIT,
  IDLE,
  LOAD_ON,
  STRESS,
  PROTECT,
  COOLING
};

static State gState = INIT;
static uint32_t stateEnterMs = 0;

// Hold-last temperature
static float lastTempC = NAN;

// Scenario timers / accumulators
static uint32_t scenarioStartMs = 0;
static uint32_t lastScoreMs = 0;
static float stressScore_mAs = 0.0f;

// PWM duty (0..1)
static float dutyCmd = 0.0f;

// Power-stable tracking
static uint32_t busVValidSinceMs = 0;

// ====== Sensors ======
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
Adafruit_INA219 ina219;

// ====== Cached sensor values (updated by scheduler) ======
static float gBusV  = 0.0f;
static float gCurmA = 0.0f;      // corrected current (raw - offset)
static float gCurRawmA = 0.0f;   // raw current (for logging/debug)
static float gP_mW  = 0.0f;

// ====== DS18B20 async handling ======
static bool     tempReqPending = false;
static uint32_t tempReqMs      = 0;
static uint32_t lastTempKickMs = 0;

// ====== Scheduler timestamps ======
static uint32_t lastInaMs = 0;
static uint32_t lastLogMs = 0;

// ====== LED helper ======
static void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r ? HIGH : LOW);
  digitalWrite(LED_G_PIN, g ? HIGH : LOW);
  digitalWrite(LED_B_PIN, b ? HIGH : LOW);
}

// ====== 1ms Tick Timer (HW timer ISR) ======
static hw_timer_t* gTimer = nullptr;
static volatile uint32_t gTickMs = 0;

static void ARDUINO_ISR_ATTR onTickISR() {
  gTickMs++;
}

static uint32_t nowMs() {
  return gTickMs;
}

static void startTickTimer_1ms() {
  gTimer = timerBegin(1000000); // 1 tick = 1us
  if (!gTimer) {
    Serial.println("ERR: timerBegin failed");
    return;
  }

  timerAttachInterrupt(gTimer, &onTickISR);
  timerAlarm(gTimer, 1000, true, 0); // 1000us => 1ms tick
  timerStart(gTimer);
}

// ====== Helpers ======
static const char* stateName(State s) {
  switch (s) {
    case INIT:     return "INIT";
    case IDLE:     return "IDLE";
    case LOAD_ON:  return "LOAD_ON";
    case STRESS:   return "STRESS";
    case PROTECT:  return "PROTECT";
    case COOLING:  return "COOLING";
    default:       return "UNKNOWN";
  }
}

static bool isButtonPressed() {
  return (digitalRead(BUTTON_PIN) == LOW);
}

// ---- PWM control ----
static void pwmInit() {
  ledcAttach(MOSFET_GATE_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(MOSFET_GATE_PIN, 0);
}

static void setMosfetDuty(float duty01) {
  if (duty01 < 0.0f) duty01 = 0.0f;
  if (duty01 > 1.0f) duty01 = 1.0f;

  dutyCmd = duty01;
  const int maxDuty = (1 << PWM_RES_BITS) - 1;
  int duty = (int)(dutyCmd * maxDuty + 0.5f);

  ledcWrite(MOSFET_GATE_PIN, duty);
}

static void setMosfetOff() {
  setMosfetDuty(0.0f);
}

static void readIna(float &busV, float &curRawmA, float &curCorrmA, float &p_mW) {
  busV     = ina219.getBusVoltage_V();
  curRawmA = ina219.getCurrent_mA();

  float corr = curRawmA - (offset_calibrated ? I_OFFSET_mA : 0.0f);
  if (corr < 0.0f) corr = 0.0f;
  curCorrmA = corr;

  p_mW = ina219.getPower_mW();
}

static bool isBusVValidStable(uint32_t now, float busV) {
  if (busV >= BUSV_VALID_V) {
    if (busVValidSinceMs == 0) busVValidSinceMs = now;
    return (now - busVValidSinceMs) >= BUSV_STABLE_MS;
  } else {
    busVValidSinceMs = 0;
    return false;
  }
}

static void updateStressScore(uint32_t now, float curmA_corr) {
  if (lastScoreMs == 0) lastScoreMs = now;
  uint32_t dtMs = now - lastScoreMs;
  lastScoreMs = now;

  float dtS = dtMs / 1000.0f;
  float excess = curmA_corr - I_TH_mA;

  if (excess > 0.0f) {
    stressScore_mAs += excess * dtS;
  } else {
    const float decayPerSec = 10.0f; // mA*s per second
    stressScore_mAs -= decayPerSec * dtS;
    if (stressScore_mAs < 0.0f) stressScore_mAs = 0.0f;
  }
}

static float computeDutyFromHold(uint32_t now) {
  if (scenarioStartMs == 0) return 0.0f;

  uint32_t heldMs = now - scenarioStartMs;
  float alpha = 1.0f;

  if (RAMP_TO_MAX_MS > 0) {
    alpha = (float)heldMs / (float)RAMP_TO_MAX_MS;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
  }

  return DUTY_MIN + (DUTY_MAX - DUTY_MIN) * alpha;
}

static void printProtectReason(uint32_t now, float curmA_corr, const char* reason) {
  float heldSec = (scenarioStartMs == 0) ? 0.0f : (now - scenarioStartMs) / 1000.0f;

  Serial.print("PROTECT: ");
  Serial.print(reason);
  Serial.print(" | I_corr=");
  Serial.print(curmA_corr, 1);
  Serial.print("mA (TH ");
  Serial.print(I_TH_mA, 1);
  Serial.print(") | Score=");
  Serial.print(stressScore_mAs, 1);
  Serial.print("mA*s (LIM ");
  Serial.print(SCORE_LIMIT_mAs, 1);
  Serial.print(") | Hold=");
  Serial.print(heldSec, 1);
  Serial.println("s -> MOSFET OFF");
}

static void printWarnNoSupply(uint32_t now, float busV) {
  if (now - lastWarnMs < WARN_PERIOD_MS) return;
  lastWarnMs = now;

  Serial.print("WARN: NO_SUPPLY (BusV=");
  Serial.print(busV, 3);
  Serial.println("V). Ignore button, stay IDLE.");
}

static void printLog(uint32_t now, float busV, float curRawmA, float curCorrmA, float p_mW) {
  int adcRaw = analogRead(VOLTAGE_PIN);
  float adcV = (float)adcRaw * (ADC_VREF / (float)ADC_MAX);

  float heldSec = 0.0f;
  if (scenarioStartMs != 0) heldSec = (now - scenarioStartMs) / 1000.0f;

  Serial.print("S: ");
  Serial.print(stateName(gState));
  Serial.print(" | MOSFET_Duty: ");
  Serial.print(dutyCmd * 100.0f, 0);
  Serial.print("% | Temp: ");
  if (isnan(lastTempC)) Serial.print("N/A");
  else Serial.print(lastTempC, 2);
  Serial.print(" C | ADCraw: ");
  Serial.print(adcRaw);
  Serial.print(" | InputV(ADC): ");
  Serial.print(adcV, 2);
  Serial.print(" V | BusV: ");
  Serial.print(busV, 3);

  Serial.print(" V | I_raw: ");
  Serial.print(curRawmA, 2);
  Serial.print(" mA | I_off: ");
  Serial.print(I_OFFSET_mA, 2);
  Serial.print(" mA | I_corr: ");
  Serial.print(curCorrmA, 2);

  Serial.print(" mA | P: ");
  Serial.print(p_mW, 1);
  Serial.print(" mW | Hold: ");
  Serial.print(heldSec, 1);
  Serial.print(" s | StressScore: ");
  Serial.print(stressScore_mAs, 1);
  Serial.print(" mA*s (TH ");
  Serial.print(I_TH_mA, 1);
  Serial.print(", LIM ");
  Serial.print(SCORE_LIMIT_mAs, 1);
  Serial.println(")");
}

static void enterState(State next, uint32_t now) {
  gState = next;
  stateEnterMs = now;

  switch (gState) {
    case INIT:
    case IDLE:
    case PROTECT:
    case COOLING:
      setMosfetOff();
      break;
    case LOAD_ON:
    case STRESS:
      break;
  }

  if (gState == LOAD_ON) {
    scenarioStartMs = now;
    lastScoreMs = scenarioStartMs;
    stressScore_mAs = 0.0f;
    setMosfetDuty(DUTY_MIN);
  }

  if (gState == COOLING) {
    scenarioStartMs = 0;
    lastScoreMs = 0;
    stressScore_mAs = 0.0f;
  }

  if (gState == IDLE) {
    scenarioStartMs = 0;
    lastScoreMs = 0;
    stressScore_mAs = 0.0f;
    dutyCmd = 0.0f;
    setMosfetOff();
  }

  setLED(false, false, false);   // 🔴🟢🔵 전부 OFF → 이전 상태 잔류 제거

  // LED states (PROTECT blink handled in loop)
switch (gState) {
  case IDLE:     setLED(false, true,  false); break; // Green
  case LOAD_ON:  setLED(true,  true,  false); break; // Yellow
  case STRESS:   setLED(true,  true,  false); break; // Yellow  <-- 원하는 동작
  case PROTECT:  setLED(true,  false, false); break; // Red (will blink)
  case COOLING:  setLED(false, false, true ); break; // Blue
  case INIT:     setLED(false, false, false); break; // Off
}


  Serial.print("STATE => ");
  Serial.println(stateName(gState));
}

// ====== INA219 offset calibration (IDLE, MOSFET OFF, button not pressed) ======
static void calibrateCurrentOffset() {
  float sum = 0.0f;

  for (int i = 0; i < OFFSET_SAMPLE_COUNT; i++) {
    sum += ina219.getCurrent_mA();
    delay(20);
  }

  I_OFFSET_mA = sum / (float)OFFSET_SAMPLE_COUNT;
  offset_calibrated = true;

  Serial.print("[CAL] I_OFFSET_mA = ");
  Serial.print(I_OFFSET_mA, 3);
  Serial.println(" mA");
}

// ====== DS18B20 async scheduler ======
static void tempKickIfDue(uint32_t now) {
  if (!tempReqPending && (now - lastTempKickMs >= TEMP_PERIOD_MS)) {
    lastTempKickMs = now;
    ds18b20.requestTemperatures();
    tempReqPending = true;
    tempReqMs = now;
  }
}

static void tempReadIfReady(uint32_t now) {
  if (tempReqPending && (now - tempReqMs >= DS18B20_CONV_MS)) {
    lastTempC = ds18b20.getTempCByIndex(0);
    tempReqPending = false;
  }
}

// ====== PROTECT LED blink handler ======
static void updateProtectBlinkLED(uint32_t now) {
  uint32_t elapsed = now - stateEnterMs;
  bool on = ((elapsed / PROTECT_BLINK_PERIOD_MS) % 2) == 0;
  setLED(on, false, false);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("BOOT: setup start");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation((gpio_num_t)VOLTAGE_PIN, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);

  ina219.begin();

  pwmInit();
  setMosfetOff();

  startTickTimer_1ms();

  delay(20);
  Serial.print("BOOT: tick=");
  Serial.println(nowMs());

  uint32_t now = nowMs();
  enterState(INIT, now);

  Serial.println("BOOT: setup done");
}

void loop() {
  uint32_t now = nowMs();

  // ---- Scheduled tasks ----
  if (now - lastInaMs >= INA_PERIOD_MS) {
    lastInaMs = now;

    readIna(gBusV, gCurRawmA, gCurmA, gP_mW);

    if (gState == STRESS) {
      updateStressScore(now, gCurmA); // corrected current
    }
  }

  tempKickIfDue(now);
  tempReadIfReady(now);

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    printLog(now, gBusV, gCurRawmA, gCurmA, gP_mW);
  }

  if (gState == PROTECT) {
    updateProtectBlinkLED(now);
  }

  // ---- State machine uses cached values ----
  bool pressed = isButtonPressed();
  bool supplyOK = isBusVValidStable(now, gBusV);

  // ---- Offset calibration arm (IDLE 안정화 이후 1회만) ----
  if (gState == IDLE && !offset_calibrated && !pressed && dutyCmd <= 0.0001f) {
    if ((now - stateEnterMs) >= OFFSET_ARM_DELAY_MS) {
      calibrateCurrentOffset();
      // 보정 직후 다음 주기부터 gCurmA가 (raw - offset)로 들어옴
    }
  }

  switch (gState) {
    case INIT:
      if (now - stateEnterMs >= INIT_WARMUP_MS) {
        enterState(IDLE, now);
      }
      break;

    case IDLE:
      if (pressed) {
        if (!supplyOK) {
          printWarnNoSupply(now, gBusV);
          setMosfetOff();
        } else {
          enterState(LOAD_ON, now);
        }
      }
      break;

    case LOAD_ON:
      if (!pressed) {
        enterState(IDLE, now);
        break;
      }
      if (!supplyOK) {
        printWarnNoSupply(now, gBusV);
        enterState(IDLE, now);
        break;
      }
      enterState(STRESS, now);
      break;

    case STRESS: {
      if (!pressed) {
        enterState(IDLE, now);
        break;
      }
      if (!supplyOK) {
        printWarnNoSupply(now, gBusV);
        enterState(IDLE, now);
        break;
      }

      float duty = computeDutyFromHold(now);
      setMosfetDuty(duty);

      bool tempValid = (!isnan(lastTempC) && lastTempC > -100.0f);
      if (tempValid && lastTempC >= TEMP_LIMIT_C) {
        printProtectReason(now, gCurmA, "TEMP_LIMIT");
        enterState(PROTECT, now);
        break;
      }

      if (stressScore_mAs >= SCORE_LIMIT_mAs) {
        printProtectReason(now, gCurmA, "SCORE_LIMIT");
        enterState(PROTECT, now);
        break;
      }

      if (scenarioStartMs != 0 && (now - scenarioStartMs) >= MAX_SCENARIO_MS) {
        printProtectReason(now, gCurmA, "MAX_SCENARIO_MS");
        enterState(PROTECT, now);
        break;
      }
      break;
    }

    case PROTECT:
      if (now - stateEnterMs >= PROTECT_BLINK_TOTAL_MS) {
        enterState(COOLING, now);
      }
      break;

    case COOLING:
      if (now - stateEnterMs >= COOLING_MS) {
        enterState(IDLE, now);
      }
      break;
  }
}
