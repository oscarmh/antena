# Firmware Migration — AS5600 + PWM → Feetech STS3250 SCServo

This document describes the changes needed to adapt the Discovery Drive firmware to use Feetech STS3250 smart servos instead of the original AS5600 hall sensors + PWM motor drivers.

## What Changes, What Stays

### Unchanged (keep as-is)
- Web UI (web_server.cpp)
- rotctl WiFi protocol (rotctl_wifi.cpp)
- Stellarium poller (stellarium_poller.cpp)
- Wind tracking and wind stow logic
- Auto-home
- Safety monitor loop
- INA219 power monitoring
- WiFi manager
- Logger
- Serial manager (EasyComm II)
- Outer PID error calculation (`angle_shortest_error_az`, `angle_error_el`)
- Kalman filter for smooth tracking
- Latch logic, direction lock, oscillation detection

### Removed
- AS5600 I2C reads (Wire.beginTransmission, Wire.requestFrom, etc.)
- PWM motor output (analogWrite)
- Direction pin control (digitalWrite on dirPin)
- TCA9548A I2C mux (not needed — STS3250 uses UART bus with IDs)
- I2C mutex shared between motor controller and INA219

### Added
- SCServo library (Feetech/SCServo on Arduino Library Manager)
- Serial2 UART for servo bus (1 Mbit/s, half-duplex via URT-2)
- `SMS_STS scServo` object in MotorSensorController

---

## Servo ID Assignment

| Servo | ID | Axis |
|---|---|---|
| STS3250 AZ | 1 | Azimuth |
| STS3250 EL | 2 | Elevation |

Set IDs using Feetech Debug Tool or URT-2 USB before installation.

---

## Position Mapping

AS5600 and STS3250 both use 12-bit resolution over 360°:
- 0 ticks = 0°
- 4095 ticks = 360°
- Conversion: `ticks = degrees × (4095.0f / 360.0f)`
- Same resolution as original: `0.0879°/tick`

---

## Changes Required

### 1. `motor_controller.h`

```cpp
// REMOVE:
#include <Wire.h>
static constexpr int _el_hall_i2c_addr = 0x36;
static constexpr int _az_hall_i2c_addr = 0x40;
static constexpr int _pwm_pin_az = 35;
static constexpr int _ccw_pin_az = 36;
static constexpr int _pwm_pin_el = 40;
static constexpr int _ccw_pin_el = 41;
static constexpr int FREQ = 20000;
SemaphoreHandle_t _i2cMutex;
// also remove: setI2CMutex() declaration, i2cErrorFlag_az/el

// ADD:
#include <SCServo.h>
static constexpr int _az_servo_id = 1;
static constexpr int _el_servo_id = 2;
SMS_STS scServo;
// rename i2cErrorFlag_az/el → commErrorFlag_az/el

// AxisConfig struct: replace pwmPin, dirPin, hallI2cAddr → servoId (int)
```

### 2. `motor_controller.cpp` — `begin()` lines 76–87

```cpp
// REMOVE all of:
pinMode(_pwm_pin_az, OUTPUT);
digitalWrite(_pwm_pin_az, 1);
// ... (all 8 pin setup lines)
analogWriteFrequency(_pwm_pin_az, FREQ);
analogWriteFrequency(_pwm_pin_el, FREQ);

// ADD:
Serial2.begin(1000000, SERIAL_8N1, 18, 17); // RX=GPIO18, TX=GPIO17
scServo.pSerial = &Serial2;
```

### 3. `motor_controller.cpp` — `ReadRawAngle()` lines 1416–1459

```cpp
// REPLACE entire body with:
float MotorSensorController::ReadRawAngle(int servo_id) {
    int rawPos = scServo.ReadPos(servo_id);
    if (rawPos < 0) return -999; // comms error
    return rawPos * (360.0f / 4095.0f);
}
```

### 4. `motor_controller.cpp` — `checkMagnetPresence()` lines 1461–1511

```cpp
// REPLACE entire body with:
int MotorSensorController::checkMagnetPresence(int servo_id) {
    int ping = scServo.Ping(servo_id);
    return (ping == servo_id) ? 32 : 0; // bit 5 = present (matches existing flag logic)
}
```

### 5. `motor_controller.cpp` — `applyMotorOutput()` lines 1155–1166

```cpp
// REPLACE body with:
void MotorSensorController::applyMotorOutput(const AxisConfig& cfg, AxisState& axis,
                                              int targetSpeed, double error, int minSpeed) {
    if (axis.setPointState->load() && !global_fault && !axis.isMotorLatched->load()) {
        int targetTicks = (int)((*axis.setpoint) * (4095.0f / 360.0f));
        targetTicks = constrain(targetTicks, 0, 4095);
        int servoSpeed = map(abs((int)(error * 10)), 0, 1800, 200, 3000);
        servoSpeed = constrain(servoSpeed, 200, 3000);
        scServo.WritePosEx(cfg.servoId, targetTicks, servoSpeed, 50);
    } else {
        // Hold current position
        int currTicks = scServo.ReadPos(cfg.servoId);
        if (currTicks >= 0) scServo.WritePosEx(cfg.servoId, currTicks, 0, 0);
    }
}
```

### 6. `motor_controller.cpp` — `resolveDirection()` lines 1108–1144

Remove all `digitalWrite(cfg.dirPin, ...)` calls. Direction is implicit in the signed position passed to `WritePosEx`.

### 7. `motor_controller.cpp` — `setPWM()` lines 1253–1255

Stub out or redirect — all motor output now goes through `applyMotorOutput()`.

### 8. `motor_controller.cpp` — Bare `analogWrite` calls

Replace all 14 bare `analogWrite` calls (lines 229, 230, 1958–1961, 1987, 1991–1992, 2086–2087, 2136, 2138) with SCServo hold-position commands.

### 9. `motor_controller.cpp` — `handleCalibrationMode()`

Replace time-based open-loop drive with incremental position steps:
```cpp
// Instead of: analogWrite(pwmPin, speed) for duration
// Use: scServo.WritePosEx(id, currPos ± delta, speed, acc)
```

### 10. `discovery_drive.ino` lines 83–86

```cpp
Wire.begin(_SDA_PIN, _SCL_PIN); // KEEP — INA219 still on I2C
Wire.setClock(250000L);          // KEEP
Wire.setTimeOut(3000);           // KEEP

// ADD:
Serial2.begin(1000000, SERIAL_8N1, 18, 17);
motorSensorCtrl.scServo.pSerial = &Serial2;

// REMOVE:
// SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutex();
// motorSensorCtrl.setI2CMutex(i2cMutex);  ← motor ctrl no longer uses I2C
// ina219Manager.setI2CMutex(i2cMutex);    ← INA219 gets its own mutex internally
```

---

## PID Tuning Notes

The STS3250 has its own internal PID controller. The outer PID loop in Discovery Drive firmware now acts as a **position supervisor** rather than a direct motor driver:

- **P term**: maps position error → servo speed parameter in `WritePosEx`. Start with default `P_az=50`, `P_el=100`
- **I term**: set `I_az=0`, `I_el=0` — the servo handles integral internally
- **D term**: set `D_az=0`, `D_el=0` — the servo handles derivative internally

The servo's internal acceleration (`acc` parameter in `WritePosEx`) replaces the `PID_RAMP_TAU` ramp. Start with `acc=50` for gentle starts (antenna-safe).

---

## Known Issues / Risks

1. **14 bare `analogWrite` calls** must all be found and replaced — grep for `analogWrite` in motor_controller.cpp before building
2. **`playOdeToJoy()`** uses PWM frequency modulation on motor pins — stub it out or replace with buzzer
3. **Calibration mode** needs adaptation from time-based to position-based incremental movement
4. **EMA jump rejection** (5° threshold) may trigger on commanded fast moves — consider raising to 15° or disabling during active setpoint changes
5. **`i2cErrorFlag_az/el`** referenced in `serial_manager.cpp` and `web_server.cpp` — rename to `commErrorFlag_az/el` in all three files

---

## Arduino Library Dependencies

```
SCServo          (Feetech — install via Arduino Library Manager)
Adafruit INA219  (unchanged from original)
ArduinoJson      (unchanged)
ESPAsyncWebServer (unchanged)
LittleFS         (unchanged)
```
