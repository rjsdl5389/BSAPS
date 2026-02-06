# ESP32 Pin Map

| GPIO | Signal | Direction | Mode / Interface | Connected Device | Notes |
|------|--------|-----------|------------------|------------------|-------|
| 15 | ONE_WIRE_BUS | I/O | OneWire | DS18B20 | 데이터 라인 |
| 34 | VOLTAGE_PIN | Input | ADC1 | Voltage Divider | 12-bit, ADC_11db |
| 21 | I2C_SDA | I/O | I2C SDA | INA219 | |
| 22 | I2C_SCL | I/O | I2C SCL | INA219 | |
| 25 | MOSFET_GATE | Output | PWM (LEDC) | MOSFET Gate | 20 kHz, 10-bit |
| 33 | BUTTON | Input | INPUT_PULLUP | User Button | Pressed = LOW |
| 26 | LED_R | Output | Digital | RGB LED (R) | HIGH = ON |
| 27 | LED_G | Output | Digital | RGB LED (G) | HIGH = ON |
| 14 | LED_B | Output | Digital | RGB LED (B) | HIGH = ON |
