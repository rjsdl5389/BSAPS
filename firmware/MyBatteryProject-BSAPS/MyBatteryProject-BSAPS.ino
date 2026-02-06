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
#define LED_R_PIN  26
#define LED_G_PIN  27
#define LED_B_PIN  14


// ====== Timing ======
static const uint32_t LOG_PERIOD_MS      = 500;    // print every 500ms
static const uint32_t TEMP_PERIOD_MS     = 1000;   // update temp every 1s (hold last for logs)
static const uint32_t COOLING_MS         = 5000;   // stay in COOLING before back to IDLE
static const uint32_t INIT_WARMUP_MS     = 1500;   // small warmup time

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
static uint32_t lastLogMs = 0;
static uint32_t lastTempMs = 0;

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

// ---- PWM control (Arduino-ESP32 core v3.x style) ----
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

static void readIna(float &busV, float &curmA, float &p_mW) {
  busV  = ina219.getBusVoltage_V();
  curmA = ina219.getCurrent_mA();
  p_mW  = ina219.getPower_mW();
}

static bool isBusVValidStable(uint32_t nowMs, float busV) {
  if (busV >= BUSV_VALID_V) {
    if (busVValidSinceMs == 0) busVValidSinceMs = nowMs;
    return (nowMs - busVValidSinceMs) >= BUSV_STABLE_MS;
  } else {
    busVValidSinceMs = 0;
    return false;
  }
}

static void updateTemperature(uint32_t nowMs) {
  if (nowMs - lastTempMs < TEMP_PERIOD_MS) return;
  lastTempMs = nowMs;

  ds18b20.requestTemperatures();
  lastTempC = ds18b20.getTempCByIndex(0);
}

static void updateStressScore(uint32_t nowMs, float curmA) {
  if (lastScoreMs == 0) lastScoreMs = nowMs;
  uint32_t dtMs = nowMs - lastScoreMs;
  lastScoreMs = nowMs;

  float dtS = dtMs / 1000.0f;
  float excess = curmA - I_TH_mA;

  if (excess > 0.0f) {
    stressScore_mAs += excess * dtS;
  } else {
    // small decay (demo-friendly)
    const float decayPerSec = 10.0f; // mA*s per second
    stressScore_mAs -= decayPerSec * dtS;
    if (stressScore_mAs < 0.0f) stressScore_mAs = 0.0f;
  }
}

static float computeDutyFromHold(uint32_t nowMs) {
  if (scenarioStartMs == 0) return 0.0f;

  uint32_t heldMs = nowMs - scenarioStartMs;
  float alpha = 1.0f;

  if (RAMP_TO_MAX_MS > 0) {
    alpha = (float)heldMs / (float)RAMP_TO_MAX_MS;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
  }

  return DUTY_MIN + (DUTY_MAX - DUTY_MIN) * alpha;
}

static void printProtectReason(uint32_t nowMs, float curmA, const char* reason) {
  float heldSec = (scenarioStartMs == 0) ? 0.0f : (nowMs - scenarioStartMs) / 1000.0f;

  Serial.print("PROTECT: ");
  Serial.print(reason);
  Serial.print(" | I=");
  Serial.print(curmA, 1);
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

static void printWarnNoSupply(uint32_t nowMs, float busV) {
  if (nowMs - lastWarnMs < WARN_PERIOD_MS) return;
  lastWarnMs = nowMs;

  Serial.print("WARN: NO_SUPPLY (BusV=");
  Serial.print(busV, 3);
  Serial.println("V). Ignore button, stay IDLE.");
}

static void printLog(uint32_t nowMs, float busV, float curmA, float p_mW) {
  if (nowMs - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = nowMs;

  int adcRaw = analogRead(VOLTAGE_PIN);
  float adcV = (float)adcRaw * (ADC_VREF / (float)ADC_MAX);

  float heldSec = 0.0f;
  if (scenarioStartMs != 0) heldSec = (nowMs - scenarioStartMs) / 1000.0f;

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
  Serial.print(" V | I: ");
  Serial.print(curmA, 2);
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

static void enterState(State next) {
  gState = next;
  stateEnterMs = millis();

  switch (gState) {
    case INIT:
    case IDLE:
    case PROTECT:
    case COOLING:
      setMosfetOff();
      break;
    case LOAD_ON:
    case STRESS:
      // duty controlled in logic
      break;
  }

  if (gState == LOAD_ON) {
    scenarioStartMs = millis();
    lastScoreMs = scenarioStartMs;
    stressScore_mAs = 0.0f;
    setMosfetDuty(DUTY_MIN);
  }

  if (gState == PROTECT) {
    // nothing here; PROTECT reason already printed before entering
  }

  if (gState == COOLING) {
    // Clean for next demo: keep cooling, but reset hold/score display
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

  switch (gState) {
  case IDLE:     setLED(false, true,  false); break; // Green
  case LOAD_ON:  setLED(true,  true,  false); break; // Yellow
  case STRESS:   setLED(true,  true,  false); break; // Yellow
  case PROTECT:  setLED(true,  false, false); break; // Red
  case COOLING:  setLED(false, false, true ); break; // Blue
  case INIT:     setLED(false, false, false); break; // Off
}


  Serial.print("STATE => ");
  Serial.println(stateName(gState));
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation((gpio_num_t)VOLTAGE_PIN, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);

  ds18b20.begin();
  ina219.begin();

  pwmInit();
  setMosfetOff();

  enterState(INIT);
}

void loop() {
  uint32_t now = millis();

  updateTemperature(now);

  float busV = 0, curmA = 0, p_mW = 0;
  readIna(busV, curmA, p_mW);

  printLog(now, busV, curmA, p_mW);

  bool pressed = isButtonPressed();
  bool supplyOK = isBusVValidStable(now, busV);

  switch (gState) {
    case INIT:
      if (now - stateEnterMs >= INIT_WARMUP_MS) {
        enterState(IDLE);
      }
      break;

    case IDLE:
      if (pressed) {
        if (!supplyOK) {
          printWarnNoSupply(now, busV);
          setMosfetOff();
        } else {
          enterState(LOAD_ON);
        }
      }
      break;

    case LOAD_ON:
      if (!pressed) {
        enterState(IDLE);
        break;
      }
      if (!supplyOK) {
        printWarnNoSupply(now, busV);
        enterState(IDLE);
        break;
      }
      enterState(STRESS);
      break;

    case STRESS: {
      if (!pressed) {
        enterState(IDLE);
        break;
      }
      if (!supplyOK) {
        printWarnNoSupply(now, busV);
        enterState(IDLE);
        break;
      }

      float duty = computeDutyFromHold(now);
      setMosfetDuty(duty);

      updateStressScore(now, curmA);

      bool tempValid = (!isnan(lastTempC) && lastTempC > -100.0f);
      if (tempValid && lastTempC >= TEMP_LIMIT_C) {
        printProtectReason(now, curmA, "TEMP_LIMIT");
        enterState(PROTECT);
        break;
      }

      if (stressScore_mAs >= SCORE_LIMIT_mAs) {
        printProtectReason(now, curmA, "SCORE_LIMIT");
        enterState(PROTECT);
        break;
      }

      if (scenarioStartMs != 0 && (now - scenarioStartMs) >= MAX_SCENARIO_MS) {
        printProtectReason(now, curmA, "MAX_SCENARIO_MS");
        enterState(PROTECT);
        break;
      }

      break;
    }

    case PROTECT:
      enterState(COOLING);
      break;

    case COOLING:
      if (now - stateEnterMs >= COOLING_MS) {
        enterState(IDLE);
      }
      break;
  }
}

static void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r ? HIGH : LOW);
  digitalWrite(LED_G_PIN, g ? HIGH : LOW);
  digitalWrite(LED_B_PIN, b ? HIGH : LOW);
}
