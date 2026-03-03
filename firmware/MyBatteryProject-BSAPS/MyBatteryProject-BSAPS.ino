#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ====== Pins ======
#define ONE_WIRE_BUS     15
#define VOLTAGE_PIN      34
#define I2C_SDA          21
#define I2C_SCL          22
#define MOSFET_GATE_PIN  25
#define BUTTON_PIN       33   // INPUT_PULLUP, pressed = LOW
#define LED_R_PIN        26
#define LED_G_PIN        27
#define LED_B_PIN        14

// ====== Logging mode ======
// 0: portfolio (compact)
// 1: debug (prints raw/offset/corr/adc)
#define LOG_DEBUG 0

// ====== Display cleanup ======
static const float I_SNAP_OFF_mA = 0.5f;  // OFF states: Ieff below this -> print 0.00

// ====== Scheduling periods ======
static const uint32_t INA_PERIOD_MS      = 100;
static const uint32_t LOG_PERIOD_MS      = 500;
static const uint32_t TEMP_PERIOD_MS     = 1000;
static const uint32_t DS18B20_CONV_MS    = 750;

// ====== Timings ======
static const uint32_t COOLING_MS         = 5000;
static const uint32_t INIT_WARMUP_MS     = 1500;

// ====== PROTECT LED blink timing ======
static const uint32_t PROTECT_BLINK_TOTAL_MS  = 2000;
static const uint32_t PROTECT_BLINK_PERIOD_MS = 200;

// ====== Voltage sensing (display only) ======
static const float ADC_VREF              = 3.3f;
static const int   ADC_MAX               = 4095;

// ====== Power present detection ======
static const float BUSV_VALID_V          = 1.0f;
static const uint32_t BUSV_STABLE_MS     = 300;

// ====== Scenario ======
static const uint32_t RAMP_TO_MAX_MS     = 6000;
static const float    DUTY_MIN           = 0.20f;
static const float    DUTY_MAX           = 1.00f;

// ====== Stress logic ======
static const float I_TH_mA               = 15.0f;
static const float SCORE_LIMIT_mAs       = 120.0f;
static const float TEMP_LIMIT_C          = 55.0f;
static const uint32_t MAX_SCENARIO_MS    = 20000;

// ====== PWM settings (ESP32 Arduino core v3.x) ======
static const int PWM_FREQ_HZ             = 20000;
static const int PWM_RES_BITS            = 10;

// ====== WARN throttling ======
static uint32_t lastWarnMs               = 0;
static const uint32_t WARN_PERIOD_MS     = 800;

// ====== INA219 current offset calibration (NON-BLOCKING) ======
static float I_OFFSET_mA = 0.0f;
static bool  offset_calibrated = false;

static bool  offset_calibrating = false;
static int   offset_count = 0;
static float offset_sum = 0.0f;

static const int OFFSET_SAMPLE_COUNT = 50;        // 50 samples * 100ms = 5s
static const uint32_t OFFSET_ARM_DELAY_MS = 300;  // IDLE enter 후 대기

// ====== State machine ======
enum State {
  INIT,
  IDLE,
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

// ====== Cached sensor values ======
static float gBusV  = 0.0f;
static float gCurmA = 0.0f;      // corrected current (raw - offset), can be negative
static float gCurRawmA = 0.0f;   // raw current
static float gP_mW  = 0.0f;      // debug only

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

// ====== 1ms Tick Timer (ESP32 core 3.3.5 compatible API) ======
static hw_timer_t* gTimer = nullptr;
static volatile uint32_t gTickMs = 0;

static void ARDUINO_ISR_ATTR onTickISR() {
  gTickMs++;
}

static uint32_t nowMs() {
  return gTickMs;
}

static void startTickTimer_1ms() {
  gTimer = timerBegin(1000000); // 1MHz
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

  // keep negative (debug); decision uses Ieff
  curCorrmA = curRawmA - (offset_calibrated ? I_OFFSET_mA : 0.0f);

  p_mW = ina219.getPower_mW();
}

static float currentEffective_mA(float curCorrmA) {
  return (curCorrmA > 0.0f) ? curCorrmA : 0.0f;
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

// ---- Non-blocking offset calibration ----
static void startOffsetCalibration() {
  offset_calibrating = true;
  offset_sum = 0.0f;
  offset_count = 0;
}

static void stepOffsetCalibration(float curRawmA) {
  if (!offset_calibrating) return;

  offset_sum += curRawmA;
  offset_count++;

  if (offset_count >= OFFSET_SAMPLE_COUNT) {
    I_OFFSET_mA = offset_sum / (float)OFFSET_SAMPLE_COUNT;
    offset_calibrated = true;
    offset_calibrating = false;

    Serial.print("[CAL] I_OFFSET_mA=");
    Serial.print(I_OFFSET_mA, 3);
    Serial.println(" mA");
  }
}

static void updateStressScore(uint32_t now, float curCorrmA) {
  if (lastScoreMs == 0) lastScoreMs = now;
  uint32_t dtMs = now - lastScoreMs;
  lastScoreMs = now;

  float dtS = dtMs / 1000.0f;

  float Ieff = currentEffective_mA(curCorrmA);
  float excess = Ieff - I_TH_mA;

  if (excess > 0.0f) {
    stressScore_mAs += excess * dtS;
  } else {
    const float decayPerSec = 10.0f;
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

static bool isTempValid(float tC) {
  if (isnan(tC)) return false;
  if (tC == DEVICE_DISCONNECTED_C) return false;
  if (tC < -55.0f || tC > 125.0f) return false;
  return true;
}

static void printProtectReason(uint32_t now, float curCorrmA, const char* reason) {
  float heldSec = (scenarioStartMs == 0) ? 0.0f : (now - scenarioStartMs) / 1000.0f;
  float Ieff = currentEffective_mA(curCorrmA);

  Serial.print("PROTECT: ");
  Serial.print(reason);
  Serial.print(" | Ieff=");
  Serial.print(Ieff, 2);
  Serial.print("mA | Score=");
  Serial.print(stressScore_mAs, 1);
  Serial.print("mA*s | Hold=");
  Serial.print(heldSec, 1);
  Serial.println("s -> MOSFET OFF");
}

static void printWarnNoSupply(uint32_t now, float busV) {
  if (now - lastWarnMs < WARN_PERIOD_MS) return;
  lastWarnMs = now;

  Serial.print("WARN: NO_SUPPLY BusV=");
  Serial.print(busV, 3);
  Serial.println("V");
}

// Portfolio log (compact)
static void printLog(uint32_t now, float busV, float curRawmA, float curCorrmA) {
  float Ieff = currentEffective_mA(curCorrmA);

  // Display cleanup: OFF states show near-zero as 0.00mA (visual only)
  if (gState != STRESS && Ieff < I_SNAP_OFF_mA) {
    Ieff = 0.0f;
  }

  Serial.print("S:");
  Serial.print(stateName(gState));

  Serial.print(" | Duty:");
  Serial.print(dutyCmd * 100.0f, 0);
  Serial.print("%");

  Serial.print(" | T:");
  if (!isTempValid(lastTempC)) Serial.print("N/A");
  else Serial.print(lastTempC, 1);
  Serial.print("C");

  Serial.print(" | BusV:");
  Serial.print(busV, 3);
  Serial.print("V");

  Serial.print(" | Ieff:");
  Serial.print(Ieff, 2);
  Serial.print("mA");

  Serial.print(" | Score:");
  Serial.print(stressScore_mAs, 1);
  Serial.print("mA*s");

  Serial.print(" | Hold:");
  if (gState == STRESS && scenarioStartMs != 0) {
    float heldSec = (now - scenarioStartMs) / 1000.0f;
    Serial.print(heldSec, 1);
    Serial.print("s");
  } else {
    Serial.print("N/A");
  }

#if LOG_DEBUG
  int adcRaw = analogRead(VOLTAGE_PIN);
  float adcV = (float)adcRaw * (ADC_VREF / (float)ADC_MAX);

  Serial.print(" | Iraw:");
  Serial.print(curRawmA, 2);
  Serial.print("mA | Ioff:");
  Serial.print(I_OFFSET_mA, 3);
  Serial.print("mA | Icorr:");
  Serial.print(curCorrmA, 2);
  Serial.print("mA | ADC:");
  Serial.print(adcV, 2);
  Serial.print("V");
#endif

  Serial.println();
}

static void applyLEDForState(State s) {
  switch (s) {
    case IDLE:     setLED(false, true,  false); break;
    case STRESS:   setLED(true,  true,  false); break;
    case PROTECT:  setLED(true,  false, false); break; // blink overrides
    case COOLING:  setLED(false, false, true ); break;
    case INIT:     setLED(false, false, false); break;
    default:       setLED(false, false, false); break;
  }
}

static void enterState(State next, uint32_t now) {
  gState = next;
  stateEnterMs = now;

  if (gState == INIT || gState == IDLE || gState == PROTECT || gState == COOLING) {
    setMosfetOff();
  }

  if (gState == STRESS) {
    scenarioStartMs = now;
    lastScoreMs = scenarioStartMs;
    stressScore_mAs = 0.0f;
    setMosfetDuty(DUTY_MIN);
  }

  // IDLE에서만 사건 종료 처리(Score 리셋)
  if (gState == IDLE) {
    scenarioStartMs = 0;
    lastScoreMs = 0;
    stressScore_mAs = 0.0f;
    dutyCmd = 0.0f;
    setMosfetOff();
  }

  // COOLING에서는 Score 유지 (사건 결과 보존)
  if (gState == COOLING) {
    scenarioStartMs = 0;
    lastScoreMs = 0;
    dutyCmd = 0.0f;
    setMosfetOff();
  }

  applyLEDForState(gState);

  Serial.print("STATE => ");
  Serial.println(stateName(gState));
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
  delay(100);

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

  Serial.println("BOOT: ready");
  enterState(INIT, nowMs());
}

void loop() {
  uint32_t now = nowMs();

  // ---- Scheduled INA ----
  if (now - lastInaMs >= INA_PERIOD_MS) {
    lastInaMs = now;

    readIna(gBusV, gCurRawmA, gCurmA, gP_mW);

    if (gState == IDLE && offset_calibrating) {
      stepOffsetCalibration(gCurRawmA);
    }

    if (gState == STRESS) {
      updateStressScore(now, gCurmA);
    }
  }

  // ---- Temp async ----
  tempKickIfDue(now);
  tempReadIfReady(now);

  // ---- Logging ----
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    printLog(now, gBusV, gCurRawmA, gCurmA);
  }

  // ---- PROTECT blink ----
  if (gState == PROTECT) {
    updateProtectBlinkLED(now);
  }

  // ---- State machine ----
  bool pressed = isButtonPressed();
  bool supplyOK = isBusVValidStable(now, gBusV);

  // ---- Offset calibration arm (IDLE 안정화 이후 1회) ----
  if (gState == IDLE && !offset_calibrated && !pressed && dutyCmd <= 0.0001f) {
    if ((now - stateEnterMs) >= OFFSET_ARM_DELAY_MS) {
      if (supplyOK && !offset_calibrating) startOffsetCalibration();
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
          enterState(STRESS, now);
        }
      }
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

      if (isTempValid(lastTempC) && lastTempC >= TEMP_LIMIT_C) {
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