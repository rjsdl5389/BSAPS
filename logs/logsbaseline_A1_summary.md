# Baseline A1 Summary

- Offset (INA219): 0.106 mA
- Scenario: Press-and-hold button -> PWM duty ramps 20% to 100% in ~6s
- Steady current (Ieff @ 100% duty): ~35.6–35.9 mA
- Trip condition: SCORE_LIMIT (120.0 mA*s)
- Actual trip score: 121.0 mA*s
- Hold time to protect: ~9.5 s
- Result: MOSFET OFF -> PROTECT (blink) -> COOLING (5s) -> IDLE
- Bus voltage during run: ~12.028–12.052 V
- Temperature during run: ~25.6 C (stable)