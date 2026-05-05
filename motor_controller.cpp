/*
 * Firmware for the discovery-drive satellite dish rotator.
 * Motor Controller - Manage the movement and safety of the motors.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "motor_controller.h"
#include "weather_poller.h"  // Include for wind safety integration

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

MotorSensorController::MotorSensorController(Preferences& prefs, INA219Manager& ina219Manager, Logger& logger) 
    : _preferences(prefs), ina219Manager(ina219Manager), _logger(logger) {
    
    // Create mutexes for thread-safe access
    _setPointMutex = xSemaphoreCreateMutex();
    _getAngleMutex = xSemaphoreCreateMutex();
    _correctedAngleMutex = xSemaphoreCreateMutex();
    _errorMutex = xSemaphoreCreateMutex();
    _el_startAngleMutex = xSemaphoreCreateMutex();
    _windStowMutex = xSemaphoreCreateMutex();
    _offsetMutex = xSemaphoreCreateMutex();
    _calMutex = xSemaphoreCreateMutex();
    
    // Initialize wind tracking
    _lastManualSetpointTime = millis();
}

void MotorSensorController::begin() {
    // Load configuration parameters from preferences
    P_el = _preferences.getInt("P_el", 100);
    P_az = _preferences.getInt("P_az", 50);
    MIN_EL_SPEED = _preferences.getInt("MIN_EL_SPEED", 50);
    MIN_AZ_SPEED = _preferences.getInt("MIN_AZ_SPEED", 100);
    _MIN_AZ_TOLERANCE = _preferences.getFloat("MIN_AZ_TOL", 0.1);
    _MIN_EL_TOLERANCE = _preferences.getFloat("MIN_EL_TOL", 0.1);
    _maxPowerFaultAz = _preferences.getInt("MAX_PWR_AZ", 8);
    _maxPowerFaultEl = _preferences.getInt("MAX_PWR_EL", 7);
    _maxPowerFaultTotal = _preferences.getInt("MAX_PWR_TOT", 10);
    _minVoltageThreshold = _preferences.getInt("MIN_VOLTAGE", 6);
    I_el = _preferences.getFloat("I_el", 0.0f);
    I_az = _preferences.getFloat("I_az", 2.0f);
    D_el = _preferences.getFloat("D_el", 0.0f);
    D_az = _preferences.getFloat("D_az", 35.0f);
    _az_offset = _preferences.getFloat("az_offset", 0.0);
    _el_offset = _preferences.getFloat("el_offset", 0.0);
    _directionLockEnabled = _preferences.getBool("dirLock", true);
    _extendedElEnabled = _preferences.getBool("extendedEl", false);
    // Flip mode requires extended elevation; force off if extended is off, regardless
    // of stored value (defends against stale prefs from a previous session).
    _flipModeEnabled = _extendedElEnabled.load() && _preferences.getBool("flipMode", false);
    _autoHomeEnabled = _preferences.getBool("autoHome", false);
    _autoHomeTimeoutMs = _preferences.getInt("autoHomeMins", 3) * 60000UL;
    _smoothTrackingEnabled = _preferences.getBool("smoothTrack", false);
    _kalmanQ = _preferences.getFloat("smKalQ", 1.0f);
    _kalmanR = _preferences.getFloat("smKalR", 1.0f);
    MIN_SMOOTH_AZ_SPEED = _preferences.getInt("smMinAzSpd", 220);
    MIN_SMOOTH_EL_SPEED = _preferences.getInt("smMinElSpd", 220);
    _logger.info("Angle offsets loaded - AZ: " + String(_az_offset, 3) + "°, EL: " + String(_el_offset, 3) + "°");

    // Configure motor control pins
    pinMode(_pwm_pin_az, OUTPUT);
    digitalWrite(_pwm_pin_az, 1);
    pinMode(_pwm_pin_el, OUTPUT);
    digitalWrite(_pwm_pin_el, 1);
    pinMode(_ccw_pin_az, OUTPUT);
    digitalWrite(_ccw_pin_az, 0);
    pinMode(_ccw_pin_el, OUTPUT);
    digitalWrite(_ccw_pin_el, 0);

    // Set PWM Frequency
    analogWriteFrequency(_pwm_pin_az, FREQ);
    analogWriteFrequency(_pwm_pin_el, FREQ);

    // Load motor speed settings
    max_dual_motor_az_speed = _preferences.getInt("maxDMAzSpeed", MAX_AZ_SPEED);
    max_dual_motor_el_speed = _preferences.getInt("maxDMElSpeed", MAX_EL_SPEED);
    max_single_motor_az_speed = _preferences.getInt("maxSMAzSpeed", 0);
    max_single_motor_el_speed = _preferences.getInt("maxSMElSpeed", 0);
    singleMotorMode = _preferences.getBool("singleMotorMode", false);

    // Check magnet presence for both sensors
    int az_magnetStatus = checkMagnetPresence(_az_hall_i2c_addr);
    delay(500);
    int el_magnetStatus = checkMagnetPresence(_el_hall_i2c_addr);

    // Log magnet status
    _logger.info("AZ Magnet Detected (MD): " + String((az_magnetStatus & 32) > 0));
    _logger.info("AZ Magnet Too Weak (ML): " + String((az_magnetStatus & 16) > 0));
    _logger.info("AZ Magnet Too Strong (MH): " + String((az_magnetStatus & 8) > 0));
    _logger.info("EL Magnet Detected (MD): " + String((el_magnetStatus & 32) > 0));
    _logger.info("EL Magnet Too Weak (ML): " + String((el_magnetStatus & 16) > 0));
    _logger.info("EL Magnet Too Strong (MH): " + String((el_magnetStatus & 8) > 0));

    // Set magnet fault flags
    if ((az_magnetStatus & 32) == 0) {
        _logger.error("NO AZ MAGNET DETECTED!");
        magnetFault = true;
    }
    
    if ((el_magnetStatus & 32) == 0) {
        _logger.error("NO EL MAGNET DETECTED!");
        magnetFault = true;
    }

    // Initialize azimuth positioning — run EMA filter multiple times to settle
    float degAngleAz = 0;
    for (int i = 0; i < _numAvg; i++) {
        degAngleAz = readFilteredAngle(_az_hall_i2c_addr, _az);
        delayMicroseconds(100);
    }
    _az_startAngle = 10; // Avoid 0 to prevent backlash switching between 0 and 359
    setCorrectedAngleAz(correctAngle(getAdjustedAzStartAngle(), degAngleAz));
    needs_unwind = _preferences.getInt("needs_unwind", 0);

    // Initialize elevation positioning — run EMA filter multiple times to settle
    float degAngleEl = 0;
    for (int i = 0; i < _numAvg; i++) {
        degAngleEl = readFilteredAngle(_el_hall_i2c_addr, _el);
        delayMicroseconds(100);
    }
    setElStartAngle(_preferences.getFloat("el_cal", degAngleEl));

    _logger.info("EL START ANGLE: " + String(getElStartAngle()));
    setCorrectedAngleEl(correctAngle(getAdjustedElStartAngle(), degAngleEl));

    // Set home position (flip-mode aware so the dish doesn't slew to the wrong axis
    // pose at boot when flip mode was persisted on)
    setSetPointAzInternal(0);
    setSetPointElInternal(getHomeElInternal());

    // Initialize manual setpoint time
    _lastManualSetpointTime = millis();

    // Initialize per-axis configs
    _azCfg = { _pwm_pin_az, _ccw_pin_az, _az_hall_i2c_addr,
               &P_az, &I_az, &D_az, &MIN_AZ_SPEED, &_MIN_AZ_TOLERANCE,
               EMERGENCY_STOW_P_AZ, /*isAzimuth=*/true, /*invertDir=*/false };
    _elCfg = { _pwm_pin_el, _ccw_pin_el, _el_hall_i2c_addr,
               &P_el, &I_el, &D_el, &MIN_EL_SPEED, &_MIN_EL_TOLERANCE,
               EMERGENCY_STOW_P_EL, /*isAzimuth=*/false, /*invertDir=*/false };

    // Wire AxisState pointers to existing class-level members
    _az.setPointState   = &setPointState_az;
    _az.isMotorLatched  = &_isAzMotorLatched;
    _az.i2cErrorFlag    = &i2cErrorFlag_az;
    _az.setpoint        = &_setpoint_az;
    _az.error           = &_error_az;
    _az.correctedAngle  = &_correctedAngle_az;

    _el.setPointState   = &setPointState_el;
    _el.isMotorLatched  = &_isElMotorLatched;
    _el.i2cErrorFlag    = &i2cErrorFlag_el;
    _el.setpoint        = &_setpoint_el;
    _el.error           = &_error_el;
    _el.correctedAngle  = &_correctedAngle_el;
}

// =============================================================================
// WEATHER INTEGRATION
// =============================================================================

void MotorSensorController::setWeatherPoller(WeatherPoller* weatherPoller) {
    _weatherPoller = weatherPoller;
    _logger.info("Weather poller integration enabled");
}

void MotorSensorController::setI2CMutex(SemaphoreHandle_t mutex) {
    _i2cMutex = mutex;
}

// =============================================================================
// MAIN CONTROL LOOPS
// =============================================================================

void MotorSensorController::runControlLoop() {
    // Measure real elapsed time for PID calculations
    unsigned long now = millis();
    float dt = PID_DT_DEFAULT;
    if (_lastControlLoopTime > 0) {
        dt = (now - _lastControlLoopTime) / 1000.0f;
        if (dt <= 0.0f) dt = PID_DT_DEFAULT;
        if (dt > PID_DT_MAX) dt = PID_DT_MAX;
    }
    _lastControlLoopTime = now;

    // Update wind stow status first
    updateWindStowStatus();

    // Update wind tracking status
    updateWindTrackingStatus();

    // Check auto-home after idle timeout
    checkAutoHome();

    // Read and process azimuth angle (single read + EMA filter)
    float degAngleAz = readFilteredAngle(_az_hall_i2c_addr, _az);
    setCorrectedAngleAz(correctAngle(getAdjustedAzStartAngle(), degAngleAz));

    if (!calMode) {
        calcIfNeedsUnwind(getCorrectedAngleAz());
    }

    // Read and process elevation angle (single read + EMA filter)
    float degAngleEl = readFilteredAngle(_el_hall_i2c_addr, _el);
    setCorrectedAngleEl(correctAngle(getAdjustedElStartAngle(), degAngleEl));

    // Get current setpoints and update flags
    float current_setpoint_az = getSetPointAz();
    float current_setpoint_el = getSetPointEl();

    // During force stop, keep motors halted and preserve update flags for when it expires
    unsigned long stopUntil = _forceStopUntil.load();
    if (stopUntil > 0 && millis() < stopUntil) {
        analogWrite(_pwm_pin_az, 255);
        analogWrite(_pwm_pin_el, 255);
        return;
    }

    bool setPointAzUpdated = _setPointAzUpdated;
    bool setPointElUpdated = _setPointElUpdated;

    // Reset update flags
    if (_setPointAzUpdated) _setPointAzUpdated = false;
    if (_setPointElUpdated) _setPointElUpdated = false;

    // Update direction lock tracking
    updateDirectionLock(current_setpoint_az, current_setpoint_el,
                        setPointAzUpdated, setPointElUpdated);

    // Calculate control errors
    angle_shortest_error_az(current_setpoint_az, getCorrectedAngleAz());
    angle_error_el(current_setpoint_el, getCorrectedAngleEl());

    // Execute control logic
    if (stopUntil > 0) _forceStopUntil = 0;
    if (calMode) {
        handleCalibrationMode();
    } else if (_smoothTrackingEnabled.load() && !_windStowActive) {
        // Smooth tracking: Kalman filter interpolates setpoint, P-controller drives motor
        // Seed Kalman from current setpoints if not yet initialized
        if (!_kalmanAz.initialized) kalmanUpdate(_kalmanAz, current_setpoint_az, true);
        if (!_kalmanEl.initialized) kalmanUpdate(_kalmanEl, current_setpoint_el, false);
        // Kalman predict every tick
        kalmanPredict(_kalmanAz, dt, true);
        kalmanPredict(_kalmanEl, dt, false);
        // Kalman update when setpoint changes
        if (setPointAzUpdated) kalmanUpdate(_kalmanAz, current_setpoint_az, true);
        if (setPointElUpdated) kalmanUpdate(_kalmanEl, current_setpoint_el, false);
        // Synthetic measurements: gently converge position back toward raw setpoint
        // when no real updates are arriving (prevents overshoot drift)
        float syntheticR = _kalmanR * KALMAN_SYNTHETIC_R_MULT;
        unsigned long now = millis();
        if (!setPointAzUpdated && _kalmanAz.lastUpdateMs > 0 &&
            (now - _kalmanAz.lastUpdateMs) > KALMAN_VEL_DECAY_MS) {
            kalmanUpdate(_kalmanAz, current_setpoint_az, true, syntheticR);
        }
        if (!setPointElUpdated && _kalmanEl.lastUpdateMs > 0 &&
            (now - _kalmanEl.lastUpdateMs) > KALMAN_VEL_DECAY_MS) {
            kalmanUpdate(_kalmanEl, current_setpoint_el, false, syntheticR);
        }
        // Recompute errors using Kalman-predicted position as the effective setpoint
        angle_shortest_error_az(_kalmanAz.pos, getCorrectedAngleAz());
        angle_error_el(_kalmanEl.pos, getCorrectedAngleEl());
        // Tolerance check — stop motor when within tolerance of Kalman target
        setPointState_az = (fabs(getErrorAz()) > _MIN_AZ_TOLERANCE);
        setPointState_el = (fabs(getErrorEl()) > _MIN_EL_TOLERANCE);
        // Direction lock override: keep motor active while tracking a moving target
        if (_directionLockEnabled.load()) {
            if (_az.dirLockDirection != 0) setPointState_az = true;
            if (_el.dirLockDirection != 0) setPointState_el = true;
        }
        // Unlatch when Kalman prediction moves error back above tolerance
        if (_isAzMotorLatched.load() && fabs(getErrorAz()) > _MIN_AZ_TOLERANCE) {
            _isAzMotorLatched = false;
            _az.prevError = 0;
        }
        if (_isElMotorLatched.load() && fabs(getErrorEl()) > _MIN_EL_TOLERANCE) {
            _isElMotorLatched = false;
            _el.prevError = 0;
        }
        // Latching (sign-flip overshoot detection)
        updateLatch(_azCfg, _az, setPointAzUpdated);
        updateLatch(_elCfg, _el, setPointElUpdated);
        // Single motor mode priority and speed limits
        updateMotorPriority(setPointAzUpdated, setPointElUpdated);
        actuateMotor(_azCfg, _az, MIN_SMOOTH_AZ_SPEED, dt);
        actuateMotor(_elCfg, _el, MIN_SMOOTH_EL_SPEED, dt);
    } else {
        updateMotorControl(current_setpoint_az, current_setpoint_el, setPointAzUpdated, setPointElUpdated);
        updateMotorPriority(setPointAzUpdated, setPointElUpdated);
        actuateMotor(_azCfg, _az, MIN_AZ_SPEED, dt);
        actuateMotor(_elCfg, _el, MIN_EL_SPEED, dt);
    }

    handleOscillationDetection();
}

void MotorSensorController::runSafetyLoop() {
    String errorText;
    errorText.reserve(256);
    bool hasNewErrors = false;

    // Check fault conditions
    if (badAngleFlag) {
        global_fault = true;
        errorText += "Bad angle.\n";
        hasNewErrors = true;
    }

    if (magnetFault) {
        global_fault = true;
        errorText += "MAGNET NOT DETECTED.\n";
        hasNewErrors = true;
    }

    if (i2cErrorFlag_az) {
        global_fault = true;
        errorText += "Communications error in AZ i2c communications.\n";
        hasNewErrors = true;
    }

    if (i2cErrorFlag_el) {
        global_fault = true;
        errorText += "Communications error in EL i2c communications.\n";
        hasNewErrors = true;
    }

    // Check elevation bounds (if not in calibration mode)
    if (!calMode) {
        float currentEl = getCorrectedAngleEl();
        if ((currentEl > 100 && currentEl < (_extendedElEnabled ? 250 : 350)) || isnan(currentEl)) {
            outOfBoundsFault = true;
        }

        if (outOfBoundsFault) {
            global_fault = true;
            errorText += "EL went out of bounds. Value: " + String(currentEl) + "\n";
            hasNewErrors = true;
        }
    }

    // Check azimuth over-spin
    if (!calMode) {
        // Load once atomically and use that value consistently
        int current_unwind = needs_unwind.load(std::memory_order_relaxed);

        if (abs(current_unwind) > 1) overSpinFault = true;

        if (overSpinFault) {
            global_fault = true;
            errorText += "Needs_unwind went beyond 1, AZ has over spun. Needs_unwind value: " + String(current_unwind) + "\n";
            hasNewErrors = true;
        }
    }

    // Skip power and voltage checks during emergency wind stow
    if (!_windStowActive) {
        // Check power consumption (only when NOT in emergency wind stow)
        // Must exceed threshold continuously for OVER_POWER_FAULT_MS before faulting
        float powerValue = ina219Manager.getPower();
        int activePowerThreshold = getActivePowerThreshold();

        if (_overPowerBackoff) {
            // Axes are backing off before we throw the fault
            bool azDone = !_overPowerBackoffAz || _isAzMotorLatched.load() || !setPointState_az.load();
            bool elDone = !_overPowerBackoffEl || _isElMotorLatched.load() || !setPointState_el.load();
            bool timedOut = (millis() - _overPowerBackoffStart >= OVER_POWER_BACKOFF_MS);

            if ((azDone && elDone) || timedOut) {
                if (timedOut) {
                    _logger.warn("OVER POWER: Backoff timed out - faulting anyway");
                } else {
                    _logger.warn("OVER POWER: Backoff complete - now faulting");
                }
                _overPowerBackoff = false;
                _overPowerBackoffAz = false;
                _overPowerBackoffEl = false;
                overPowerFault = true;
            }
        } else {
            if (powerValue > activePowerThreshold) {
                if (_overPowerStartTime == 0) {
                    _overPowerStartTime = millis();
                } else if (millis() - _overPowerStartTime >= OVER_POWER_FAULT_MS) {
                    // Sustained over-power detected — capture which threshold was exceeded
                    bool azActive = setPointState_az.load() && !_isAzMotorLatched.load() && !global_fault.load();
                    bool elActive = setPointState_el.load() && !_isElMotorLatched.load() && !global_fault.load();
                    _overPowerFaultValue = activePowerThreshold;
                    if (azActive && elActive) _overPowerFaultLabel = "TOTAL";
                    else if (azActive) _overPowerFaultLabel = "AZ";
                    else if (elActive) _overPowerFaultLabel = "EL";
                    else _overPowerFaultLabel = "TOTAL";

                    // Back off each faulting axis by 20° opposite to its direction of travel
                    float currentAz = getCorrectedAngleAz();
                    float currentEl = getCorrectedAngleEl();
                    double errorAz = getErrorAz();
                    double errorEl = getErrorEl();
                    _overPowerBackoff = true;
                    _overPowerBackoffStart = millis();
                    _overPowerBackoffAz = false;
                    _overPowerBackoffEl = false;

                    if (azActive) {
                        float backoffAz = (errorAz >= 0)
                            ? currentAz - OVER_POWER_BACKOFF_DEG
                            : currentAz + OVER_POWER_BACKOFF_DEG;
                        backoffAz = fmod(backoffAz, 360.0f);
                        if (backoffAz < 0) backoffAz += 360.0f;
                        setSetPointAzInternal(backoffAz);
                        _overPowerBackoffAz = true;
                        _logger.warn("OVER POWER: Backing off AZ " +
                                     String(currentAz, 1) + "° -> " + String(backoffAz, 1) + "°");
                    }

                    if (elActive) {
                        float backoffEl = (errorEl >= 0)
                            ? currentEl - OVER_POWER_BACKOFF_DEG
                            : currentEl + OVER_POWER_BACKOFF_DEG;
                        float minEl = _extendedElEnabled ? -90.0f : 0.0f;
                        backoffEl = constrain(backoffEl, minEl, 90.0f);
                        setSetPointElInternal(backoffEl);
                        _overPowerBackoffEl = true;
                        _logger.warn("OVER POWER: Backing off EL " +
                                     String(currentEl, 1) + "° -> " + String(backoffEl, 1) + "°");
                    }

                    // Stop non-backing-off axes immediately
                    if (!_overPowerBackoffAz) {
                        _isAzMotorLatched = true;
                        setPWM(_pwm_pin_az, 255);
                    }
                    if (!_overPowerBackoffEl) {
                        _isElMotorLatched = true;
                        setPWM(_pwm_pin_el, 255);
                    }

                    _logger.warn("OVER POWER: " + String(_overPowerFaultLabel) +
                                 " threshold (" + String(_overPowerFaultValue) +
                                 "W) exceeded. Backing off before faulting.");
                }
            } else {
                _overPowerStartTime = 0;
            }
        }

        if (overPowerFault) {
            global_fault = true;
            errorText += "Power exceeded " + String(_overPowerFaultLabel) + " threshold (" +
                         String(_overPowerFaultValue) + "W) for over 1s. Rotator may be stuck or jammed. Power: " +
                         String(powerValue) + "W\n";
            hasNewErrors = true;
        }

        // Check voltage level (only when NOT in emergency wind stow)
        float loadVoltageValue = ina219Manager.getLoadVoltage();
        if (loadVoltageValue < getMinVoltageThreshold()) lowVoltageFault = true;

        if (lowVoltageFault) {
            global_fault = true;
            errorText += "Voltage too low. Voltage: " + String(loadVoltageValue) + "V\n";
            hasNewErrors = true;
        }
    } else {
        // During emergency wind stow, log power consumption but don't fault
        float powerValue = ina219Manager.getPower();
        float loadVoltageValue = ina219Manager.getLoadVoltage();
        
        static unsigned long lastStowPowerLog = 0;
        if (millis() - lastStowPowerLog > 2000) { // Log every 2 seconds during stow
            _logger.info("EMERGENCY STOW - Power: " + String(powerValue, 1) + "W, Voltage: " + 
                        String(loadVoltageValue, 1) + "V (safety limits bypassed)");
            lastStowPowerLog = millis();
        }
        
        // Reset power/voltage faults during emergency stow to allow movement
        overPowerFault = false;
        lowVoltageFault = false;
    }

    // Emergency stop on fault (except in calibration mode, wind stow mode, and wind tracking mode)
    if (global_fault && !calMode && !_windStowActive && !_windTrackingActive) {
        setPWM(_pwm_pin_el, 255);
        setPWM(_pwm_pin_az, 255);
        errorText += "EMERGENCY ALL STOP. RESTART ESP32 TO CLEAR FAULTS.\n";
        hasNewErrors = true;
        slowPrint(errorText, 0);
    }
}


// =============================================================================
// WIND SAFETY METHODS
// =============================================================================

void MotorSensorController::updateWindStowStatus() {
    if (_weatherPoller == nullptr) {
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - _lastWindStowUpdate < WIND_STOW_UPDATE_INTERVAL) {
        return;
    }
    _lastWindStowUpdate = currentTime;
    
    // Get all data atomically once
    WindSafetyData windSafetyData = _weatherPoller->getWindSafetyData();
    
    if (windSafetyData.emergencyStowActive) {
        setWindStowActive(true, windSafetyData.stowReason, windSafetyData.currentStowDirection);
        performWindStow();
    } else {
        setWindStowActive(false, "", 0.0);
    }
}


void MotorSensorController::setWindStowActive(bool active, const char* reason, float direction) {
    if (_windStowMutex != NULL && xSemaphoreTake(_windStowMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool wasActive = _windStowActive;

        _windStowActive = active;
        safeCopy(_windStowReason, reason, sizeof(_windStowReason));
        _windStowDirection = direction;

        if (active && !wasActive) {
            _logger.warn("EMERGENCY WIND STOW ACTIVATED: " + String(reason) +
                       " - Moving to safe direction: " + String(direction, 1) + "°");
            _logger.warn("POWER SAFETY OVERRIDES ENABLED - Power and voltage limits bypassed for emergency stow");
            _logger.warn("Using emergency motor gains - AZ P=" + String(EMERGENCY_STOW_P_AZ) + 
                       ", EL P=" + String(EMERGENCY_STOW_P_EL));
            
            // Clear any existing power/voltage faults to allow emergency movement
            overPowerFault = false;
            lowVoltageFault = false;
            
        } else if (!active && wasActive) {
            _logger.info("Emergency wind stow deactivated - normal operation and safety limits resumed");
        }
        
        xSemaphoreGive(_windStowMutex);
    }
}

void MotorSensorController::performWindStow() {
    if (!_windStowActive) {
        return;
    }

    // Copy values out under mutex, then release BEFORE taking _setPointMutex
    // (prevents nested mutex deadlock: _windStowMutex → _setPointMutex)
    float stowAz = 0.0f;
    float stowEl = 0.0f;
    bool gotValues = false;

    if (_windStowMutex != NULL && xSemaphoreTake(_windStowMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        stowAz = _windStowDirection;
        stowEl = 0.0f;
        gotValues = true;
        xSemaphoreGive(_windStowMutex);
    }

    if (gotValues) {
        setSetPointAzInternal(stowAz);
        setSetPointElInternal(stowEl);
    }
}

bool MotorSensorController::isWindStowActive() {
    return _windStowActive.load();
}

// NEW: Returns all wind stow data atomically under single mutex lock
WindStowState MotorSensorController::getWindStowState() {
    WindStowState state;

    if (_windStowMutex != NULL && xSemaphoreTake(_windStowMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state.active = _windStowActive.load();
        safeCopy(state.reason, _windStowReason, sizeof(state.reason));
        state.direction = _windStowDirection;
        xSemaphoreGive(_windStowMutex);
    }

    return state;
}


// =============================================================================
// WIND TRACKING METHODS
// =============================================================================

void MotorSensorController::updateWindTrackingStatus() {
    if (_weatherPoller == nullptr) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Update wind tracking at regular intervals (every 10 seconds)
    if (currentTime - _lastWindTrackingUpdate < WIND_TRACKING_UPDATE_INTERVAL) {
        return;
    }
    _lastWindTrackingUpdate = currentTime;
    
    // Check if wind tracking should be active
    bool shouldActivate = shouldActivateWindTracking();
    
    if (shouldActivate && !_windTrackingActive) {
        _logger.info(">>> ACTIVATING wind tracking - 60 second timeout reached <<<");
        setWindTrackingActive(true);
        // Immediately perform wind tracking to move to current position
        _logger.debug("Calling performWindTracking() immediately after activation");
        performWindTracking();
    } else if (!shouldActivate && _windTrackingActive) {
        _logger.info(">>> DEACTIVATING wind tracking - conditions no longer met <<<");
        setWindTrackingActive(false);
    } else if (_windTrackingActive) {
        // Continue normal wind tracking
        _logger.debug("Continuing wind tracking (already active)");
        performWindTracking();
    }
}

WindTrackingState MotorSensorController::getWindTrackingState() {
    WindTrackingState state;
    state.active = _windTrackingActive.load();
    state.lastDirection = _lastWindTrackingDirection;

    WeatherPoller* wp = _weatherPoller;  // Local copy

    if (!state.active) {
        safeCopy(state.status, "Inactive", sizeof(state.status));
    } else if (wp == nullptr || !wp->isDataValid()) {
        safeCopy(state.status, "Active (No weather data)", sizeof(state.status));
    } else {
        WeatherData weatherData = wp->getWeatherData();
        snprintf(state.status, sizeof(state.status), "Active - Wind: %.1f°, Target: %.1f°",
                 weatherData.currentWindDirection, state.lastDirection);
    }

    return state;
}

bool MotorSensorController::shouldActivateWindTracking() {
    WeatherPoller* wp = _weatherPoller;  // Local copy
    if (wp == nullptr) {
        return false;
    }
    
    if (!wp->isWindBasedHomeEnabled()) {
        return false;
    }
    
    if (_windStowActive) {
        _logger.debug("Wind tracking blocked: Emergency wind stow active");
        return false;
    }
    
    if (calMode) {
        _logger.debug("Wind tracking blocked: Calibration mode active");
        return false;
    }
    
    unsigned long currentTime = millis();
    unsigned long timeSinceManual = currentTime - _lastManualSetpointTime;
    
    if (timeSinceManual < MANUAL_SETPOINT_TIMEOUT) {
        unsigned long remainingTime = MANUAL_SETPOINT_TIMEOUT - timeSinceManual;
        _logger.debug("Wind tracking blocked: Manual timeout not reached (" + 
                     String(timeSinceManual/1000) + "s elapsed, " + 
                     String(remainingTime/1000) + "s remaining)");
        return false;
    }
    
    if (!wp->isDataValid()) {
        _logger.debug("Wind tracking blocked: Weather data not valid");
        String error = wp->getLastError();
        if (error.length() > 0) {
            _logger.debug("  Weather error: " + error);
        }
        return false;
    }
    
    _logger.debug("Wind tracking CAN activate - all conditions met");
    return true;
}

void MotorSensorController::setWindTrackingActive(bool active) {
    bool wasActive = _windTrackingActive.load();
    _windTrackingActive = active;
    
    if (active && !wasActive) {
        // Reset the last direction to force movement on first activation
        _lastWindTrackingDirection = -999.0;  // Invalid direction to force first update
        _logger.info("Wind tracking ACTIVATED - will move to current wind home position");
        _logger.debug("Reset last wind direction to force initial movement");
    } else if (!active && wasActive) {
        _logger.info("Wind tracking DEACTIVATED");
    }
}

void MotorSensorController::performWindTracking() {
    if (!_windTrackingActive || _weatherPoller == nullptr) {
        return;
    }
    
    // Get current weather data
    WeatherData weatherData = _weatherPoller->getWeatherData();
    if (!weatherData.dataValid) {
        _logger.debug("Wind tracking skipped: Weather data not valid");
        return;
    }
    
    // Calculate optimal direction based on current wind
    float optimalDirection = _weatherPoller->calculateOptimalStowDirection(weatherData.currentWindDirection);
    
    _logger.debug("Wind tracking check - Current wind: " + String(weatherData.currentWindDirection, 1) + 
                 "°, Optimal: " + String(optimalDirection, 1) + 
                 "°, Last: " + String(_lastWindTrackingDirection, 1) + "°");
    
    // Always move to the current optimal wind position (no threshold check)
    if (fabs(optimalDirection - _lastWindTrackingDirection) > 0.01f) {
        float directionChange = abs(optimalDirection - _lastWindTrackingDirection);
        
        String reason = (_lastWindTrackingDirection == -999.0) ? 
            "INITIAL wind home positioning" : 
            "Wind direction change (" + String(directionChange, 1) + "°)";
        
        _logger.info("WIND TRACKING UPDATE - " + reason);
        _logger.info("  Current wind direction: " + String(weatherData.currentWindDirection, 1) + "°");
        _logger.info("  Optimal dish direction: " + String(optimalDirection, 1) + "°");
        _logger.info("  Previous dish direction: " + String(_lastWindTrackingDirection, 1) + "°");
        
        // Update setpoints using internal methods to avoid triggering manual command tracking
        float trackingEl = getHomeElInternal();  // forward horizon — flip-mode aware
        setSetPointAzInternal(optimalDirection);
        setSetPointElInternal(trackingEl);

        _lastWindTrackingDirection = optimalDirection;
        _logger.info("  New setpoints: Az=" + String(optimalDirection, 1) + "°, El=" + String(trackingEl, 1) + "°");
        
    } else {
        _logger.debug("Wind tracking: No movement needed (direction unchanged)");
    }
}

bool MotorSensorController::isWindTrackingActive() {
    return _windTrackingActive.load();
}

const char* MotorSensorController::getWindTrackingStatus() {
    static char buf[64];
    WindTrackingState state = getWindTrackingState();
    safeCopy(buf, state.status, sizeof(buf));
    return buf;
}

void MotorSensorController::setDirectionLockEnabled(bool enabled) {
    _directionLockEnabled = enabled;
    _preferences.putBool("dirLock", enabled);
    if (!enabled) {
        _az.dirLockDirection = 0;
        _el.dirLockDirection = 0;
        _az.dirLockHasTracked = false;
        _el.dirLockHasTracked = false;
        _az.dirLockChangeCount = 0;
        _el.dirLockChangeCount = 0;
        _dirLock_initialized = false;
    }
    _logger.info("Direction lock " + String(enabled ? "enabled" : "disabled"));
}

void MotorSensorController::setSafeMode(bool enabled) {
    safeMode = enabled;
    _logger.info("Safe mode " + String(enabled ? "ON" : "OFF"));
}

void MotorSensorController::setExtendedElEnabled(bool enabled) {
    _extendedElEnabled = enabled;
    _preferences.putBool("extendedEl", enabled);
    _logger.info("Extended elevation " + String(enabled ? "enabled (-90 to 90)" : "disabled (0 to 90)"));

    // Flip mode is meaningless without the [-90, +90] range, so cascade-disable.
    if (!enabled && _flipModeEnabled.load()) {
        setFlipModeEnabled(false);
    }
}

void MotorSensorController::setFlipModeEnabled(bool enabled) {
    if (enabled && !_extendedElEnabled.load()) {
        _logger.warn("Flip mode requires Negative Elevation to be enabled - ignoring");
        return;
    }
    _flipModeEnabled = enabled;
    _preferences.putBool("flipMode", enabled);
    _logger.info("Flip mode " + String(enabled ? "enabled (0 to 180)" : "disabled"));

    // On enable, slew to flip 0° (= internal +90°, zenith) so the user starts a pass
    // from a known position. setSetPointEl runs all the usual external-input guards
    // (wind stow, over-power backoff) and clears auto-home/wind-tracking.
    if (enabled) {
        setSetPointEl(flipToInternal(0.0f));
    }
}

// =============================================================================
// AUTO-HOME METHODS
// =============================================================================

void MotorSensorController::setAutoHomeEnabled(bool enabled) {
    _autoHomeEnabled = enabled;
    _preferences.putBool("autoHome", enabled);
    if (!enabled) {
        _autoHomeActive = false;
    }
    _logger.info("Auto-home " + String(enabled ? "enabled" : "disabled"));
}

void MotorSensorController::setAutoHomeTimeout(int minutes) {
    if (minutes >= 1 && minutes <= 60) {
        _autoHomeTimeoutMs = minutes * 60000UL;
        _preferences.putInt("autoHomeMins", minutes);
        _logger.info("Auto-home timeout set to: " + String(minutes) + " minutes");
    }
}

int MotorSensorController::getAutoHomeTimeout() const {
    return _autoHomeTimeoutMs / 60000;
}

void MotorSensorController::checkAutoHome() {
    if (!_autoHomeEnabled) return;
    if (_windStowActive || calMode) return;

    unsigned long timeSinceManual = millis() - _lastManualSetpointTime;

    if (timeSinceManual < _autoHomeTimeoutMs) {
        _autoHomeActive = false;
        return;
    }

    if (_autoHomeActive) return;

    // Determine home position (same logic as /submitHome)
    float homeAz = 0.0f;
    float homeEl = getHomeElInternal();
    if (_weatherPoller != nullptr && _weatherPoller->isWindBasedHomeEnabled()) {
        homeAz = _weatherPoller->getWindBasedHomePosition();
    }

    int timeoutMins = _autoHomeTimeoutMs / 60000;
    _logger.info("Auto-home triggered after " + String(timeoutMins) + " minutes idle - homing to (" +
                 String(homeAz, 1) + ", " + String(homeEl, 1) + ")");

    setSetPointAzInternal(homeAz);
    setSetPointElInternal(homeEl);
    _autoHomeActive = true;
}

void MotorSensorController::updateDirectionLockAxis(AxisState& axis, float setpoint,
                                                      bool setPointUpdated, bool hasWraparound) {
    unsigned long now = millis();

    // Stale timeout: clear direction if no updates for too long (stationary target)
    if (axis.dirLockLastUpdate > 0 && (now - axis.dirLockLastUpdate > DIR_LOCK_STALE_MS)) {
        if (axis.dirLockDirection != 0) axis.dirLockHasTracked = false;
        axis.dirLockDirection = 0;
        axis.dirLockChangeCount = 0;
    }

    if (setPointUpdated) {
        float delta = setpoint - axis.dirLockPrevSetpoint;
        // Wraparound handling for AZ axis
        if (hasWraparound) {
            if (delta > 180.0f) delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
        }

        if (fabs(delta) > DIR_LOCK_JUMP_THRESHOLD) {
            // Large jump — new target, clear direction for normal P-control slew
            axis.dirLockDirection = 0;
            axis.dirLockHasTracked = false;
            axis.dirLockChangeCount = 0;
        } else if (fabs(delta) >= DIR_LOCK_MIN_DELTA) {
            int newDir = (delta > 0) ? 1 : -1;
            if (axis.dirLockDirection == 0) {
                // No direction set yet — lock immediately
                axis.dirLockDirection = newDir;
                axis.dirLockChangeCount = 0;
            } else if (newDir != axis.dirLockDirection) {
                // Opposing direction — large deltas (>= tolerance) flip immediately,
                // small deltas require consecutive confirmations to filter ADS-B jitter
                if (fabs(delta) >= DIR_LOCK_CONFIDENT_DELTA) {
                    axis.dirLockHasTracked = false;
                    axis.dirLockDirection = newDir;
                    axis.dirLockChangeCount = 0;
                } else {
                    axis.dirLockChangeCount++;
                    if (axis.dirLockChangeCount >= DIR_LOCK_CHANGE_CONFIRMS) {
                        axis.dirLockHasTracked = false;
                        axis.dirLockDirection = newDir;
                        axis.dirLockChangeCount = 0;
                    }
                }
            } else {
                // Same direction — reset change counter
                axis.dirLockChangeCount = 0;
            }
        }
        // If delta < MIN_DELTA, keep previous direction (micro-jitter filter)

        axis.dirLockPrevSetpoint = setpoint;
        axis.dirLockLastUpdate = now;
    }
}

void MotorSensorController::updateDirectionLock(float setpoint_az, float setpoint_el,
                                                 bool setPointAzUpdated, bool setPointElUpdated) {
    // Clear state when feature disabled or in special modes
    if (!_directionLockEnabled.load() || calMode || _windStowActive) {
        _az.dirLockDirection = 0;
        _el.dirLockDirection = 0;
        _az.dirLockHasTracked = false;
        _el.dirLockHasTracked = false;
        _az.dirLockChangeCount = 0;
        _el.dirLockChangeCount = 0;
        _dirLock_initialized = false;
        return;
    }


    // First call: seed previous setpoints without setting direction
    if (!_dirLock_initialized) {
        _az.dirLockPrevSetpoint = setpoint_az;
        _el.dirLockPrevSetpoint = setpoint_el;
        _az.dirLockLastUpdate = millis();
        _el.dirLockLastUpdate = millis();
        _dirLock_initialized = true;
        return;
    }

    updateDirectionLockAxis(_az, setpoint_az, setPointAzUpdated, /*hasWraparound=*/true);
    updateDirectionLockAxis(_el, setpoint_el, setPointElUpdated, /*hasWraparound=*/false);
}

// =============================================================================
// MODIFIED SETPOINT METHODS TO HANDLE WIND STOW AND TRACKING
// =============================================================================

void MotorSensorController::setSetPointAz(float value) {
    // Block external setpoint changes during over-power backoff
    if (_overPowerBackoff) {
        _logger.warn("Azimuth setpoint change blocked - over-power backoff in progress");
        return;
    }

    // Block external setpoint changes during wind stow (except during calibration)
    if (_windStowActive && !calMode) {
        _logger.warn("Azimuth setpoint change blocked - wind stow active");
        return;
    }
    
    // Record manual setpoint command time and deactivate wind tracking
    _lastManualSetpointTime = millis();
    
    _logger.debug("AZ setpoint: " + String(value, 2) + "°");
    if (_weatherPoller != nullptr && _weatherPoller->isWindBasedHomeEnabled()) {
        _logger.debug("  Wind home will activate in " + String(MANUAL_SETPOINT_TIMEOUT/1000) + " seconds");
    }
    
    if (_windTrackingActive) {
        _logger.info("  Deactivating wind tracking due to manual command");
        setWindTrackingActive(false);
    }

    _autoHomeActive = false;

    setSetPointAzInternal(value);
}

void MotorSensorController::setSetPointEl(float value) {
    // Block external setpoint changes during over-power backoff
    if (_overPowerBackoff) {
        _logger.warn("Elevation setpoint change blocked - over-power backoff in progress");
        return;
    }

    // Block external setpoint changes during wind stow (except during calibration)
    if (_windStowActive && !calMode) {
        _logger.warn("Elevation setpoint change blocked - wind stow active");
        return;
    }
    
    // Record manual setpoint command time and deactivate wind tracking
    _lastManualSetpointTime = millis();
    
    _logger.debug("EL setpoint: " + String(value, 2) + "°");
    if (_weatherPoller != nullptr && _weatherPoller->isWindBasedHomeEnabled()) {
        _logger.debug("  Wind home will activate in " + String(MANUAL_SETPOINT_TIMEOUT/1000) + " seconds");
    }
    
    if (_windTrackingActive) {
        _logger.info("  Deactivating wind tracking due to manual command");
        setWindTrackingActive(false);
    }

    _autoHomeActive = false;

    setSetPointElInternal(value);
}


void MotorSensorController::setSetPointAzInternal(float value) {
    if (_setPointMutex != NULL && xSemaphoreTake(_setPointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (value != _setpoint_az) {
            _setpoint_az = value;
            _setPointAzUpdated = true;
        }
        xSemaphoreGive(_setPointMutex);
    }
}

void MotorSensorController::setSetPointElInternal(float value) {
    if (_setPointMutex != NULL && xSemaphoreTake(_setPointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (value != _setpoint_el) {
            _setpoint_el = value;
            _setPointElUpdated = true;
        }
        xSemaphoreGive(_setPointMutex);
    }
}

// =============================================================================
// UNIFIED MOTOR CONTROL PIPELINE
// =============================================================================

void MotorSensorController::actuateMotor(const AxisConfig& cfg, AxisState& axis, int minSpeed, float dt) {
    // During emergency wind stow, lock EL motor once it has homed
    if (_windStowActive && !cfg.isAzimuth) {
        if (axis.isMotorLatched->load() || !axis.setPointState->load()) {
            // EL has reached home (latched or within tolerance) — keep it stopped
            setPWM(cfg.pwmPin, 255);
            axis.currentSpeed = minSpeed;
            *axis.isMotorLatched = true;
            return;
        }
    }

    double rawError = getAxisError(axis);
    int effectiveP = _windStowActive ? cfg.emergencyP : *cfg.P;
    float effectiveI = _windStowActive ? 0.0f : *cfg.I;  // no integral during stow
    float effectiveD = _windStowActive ? 0.0f : *cfg.D;

    // PID computation using measured dt
    double pTerm = rawError * effectiveP;
    double rawD = (rawError - axis.prevRawError) / dt;
    axis.filteredDTerm = PID_D_FILTER_ALPHA * rawD + (1.0 - PID_D_FILTER_ALPHA) * axis.filteredDTerm;
    double dTerm = axis.filteredDTerm * effectiveD;
    axis.prevRawError = rawError;
    axis.errorIntegral += rawError * dt;
    axis.errorIntegral = constrain(axis.errorIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    double iTerm = axis.errorIntegral * effectiveI;

    // Only apply D term when error is diverging (dTerm same sign as error).
    // This accelerates the motor to catch fast-moving targets without
    // causing braking oscillation near the target.
    bool errorDiverging = (rawError >= 0 && dTerm > 0) || (rawError < 0 && dTerm < 0);
    double pidOutput = pTerm + iTerm + (errorDiverging ? dTerm : 0.0);

    // Exponential ramp on acceleration and direction changes to protect gears/motors.
    // Deceleration (output magnitude decreasing, same sign) is unrestricted for fast response.
    bool signChanged = (pidOutput >= 0) != (axis.prevPidOutput >= 0) &&
                       fabs(axis.prevPidOutput) > 0.001;
    bool accelerating = fabs(pidOutput) > fabs(axis.prevPidOutput);
    if (accelerating || signChanged) {
        float tau = _windStowActive ? PID_RAMP_TAU_STOW : PID_RAMP_TAU;
        float alpha = 1.0f - expf(-dt / tau);
        double ramped = axis.prevPidOutput + (pidOutput - axis.prevPidOutput) * alpha;
        pidOutput = ramped;
    }
    axis.prevPidOutput = pidOutput;

    DirectionResult dir = resolveDirection(cfg, axis, pidOutput);
    if (dir.shouldStop) return;

    SpeedResult spd = calculateTargetSpeed(cfg, axis, pidOutput, minSpeed);
    if (spd.earlyReturn) return;

    applyMotorOutput(cfg, axis, spd.targetSpeed, pidOutput, minSpeed);
}

MotorSensorController::DirectionResult MotorSensorController::resolveDirection(
        const AxisConfig& cfg, AxisState& axis, double error) {
    bool dirLock = _directionLockEnabled.load() && (axis.dirLockDirection != 0);

    if (dirLock) {
        // Direction locked to setpoint movement direction
        double axisError = getAxisError(axis);
        if ((float)axis.dirLockDirection * axisError < 0) {
            if (axis.dirLockHasTracked && fabs(axisError) < 5.0) {
                // Dish previously reached the correct side and error is small —
                // this is a real overshoot, latch and stop.
                // Large errors (e.g. from unwind) are not overshoot.
                *axis.isMotorLatched = true;
                axis.errorIntegral = 0.0;
                bool fwd = (axis.dirLockDirection * (*cfg.P) > 0) ^ cfg.invertDir;
                digitalWrite(cfg.dirPin, fwd ? LOW : HIGH);
                setPWM(cfg.pwmPin, 255);
                return { true };
            }
            // Dish never reached correct side, or error is large (unwind) —
            // use normal P-control to drive toward target
            bool fwd2 = (error >= 0) ^ cfg.invertDir;
            digitalWrite(cfg.dirPin, fwd2 ? LOW : HIGH);
        } else {
            // Error agrees with locked direction — mark that dish has been on correct side
            axis.dirLockHasTracked = true;
            bool fwd = (axis.dirLockDirection * (*cfg.P) > 0) ^ cfg.invertDir;
            digitalWrite(cfg.dirPin, fwd ? LOW : HIGH);
        }
    } else {
        // Normal: set direction based on error sign
        bool fwd = (error >= 0) ^ cfg.invertDir;
        digitalWrite(cfg.dirPin, fwd ? LOW : HIGH);
    }

    return { false };
}

MotorSensorController::SpeedResult MotorSensorController::calculateTargetSpeed(
        const AxisConfig& cfg, AxisState& axis, double error, int minSpeed) {
    // P-controller: map error magnitude to PWM speed
    int targetSpeed = minSpeed - constrain(abs(error), axis.maxAdjustedSpeed, minSpeed);
    targetSpeed = max(targetSpeed, axis.maxAdjustedSpeed);

    return { targetSpeed, false };
}

void MotorSensorController::applyMotorOutput(const AxisConfig& cfg, AxisState& axis,
                                              int targetSpeed, double error, int minSpeed) {
    // Normal motor control
    if (axis.setPointState->load() && !global_fault && !axis.isMotorLatched->load()) {
        axis.currentSpeed = targetSpeed;
        setPWM(cfg.pwmPin, targetSpeed);
    } else {
        // Stop motor
        setPWM(cfg.pwmPin, 255);
        axis.currentSpeed = minSpeed;
    }
}

void MotorSensorController::updateMotorControl(float currentSetPointAz, float currentSetPointEl, bool setPointAzUpdated, bool setPointElUpdated) {
    setPointState_az = (fabs(getErrorAz()) > _MIN_AZ_TOLERANCE);
    setPointState_el = (fabs(getErrorEl()) > _MIN_EL_TOLERANCE);

    // Direction lock override: keep motor active while tracking a moving target,
    // even when error is within tolerance. Without this, small tracking deltas
    // (~0.1°) fall within tolerance, causing the motor to wait until error
    // accumulates, then jerk forward and overshoot repeatedly.
    if (_directionLockEnabled.load()) {
        if (_az.dirLockDirection != 0) setPointState_az = true;
        if (_el.dirLockDirection != 0) setPointState_el = true;
    }

    // Apply latching logic for each axis
    updateLatch(_azCfg, _az, setPointAzUpdated);
    updateLatch(_elCfg, _el, setPointElUpdated);
}

void MotorSensorController::updateLatch(const AxisConfig& cfg, AxisState& axis,
                                         bool setPointUpdated) {
    // Reset latch parameters on setpoint changes
    if (setPointUpdated) {
        *axis.isMotorLatched = false;
        axis.prevError = 0;
        axis.errorIntegral = 0.0;
        axis.prevRawError = 0.0;
        axis.filteredDTerm = 0.0;
        return;
    }

    // Detect overshoot (error sign flip) — motor passed through the target.
    // Only count sign flips when error is small — a flip at large error is likely
    // a 360° wraparound glitch, not a real overshoot.
    double currentError = getAxisError(axis);
    bool signFlipped = (axis.prevError * currentError < 0) &&
                       (fabs(axis.prevError) > 0.0001) &&
                       (fabs(currentError) > 0.0001) &&
                       (fabs(currentError) < 5.0);

    // Latch motor on target reached or overshoot
    if (!axis.setPointState->load() || signFlipped) {
        *axis.isMotorLatched = true;
        axis.errorIntegral = 0.0;
    }

    axis.prevError = currentError;
}

void MotorSensorController::updateMotorPriority(bool setPointAzUpdated, bool setPointElUpdated) {
    // Handle single motor mode priority
    if (singleMotorMode) {
        // Determine priority based on normalized error when setpoint changes
        if (setPointAzUpdated || setPointElUpdated) {
            float errorAz = fabs(getErrorAz());
            float errorEl = fabs(getErrorEl());
            // Guard against NaN from bad sensor data
            if (isnan(errorAz) || isnan(errorEl)) {
                _az_priority = !isnan(errorAz);  // Prioritize whichever axis has valid data
            } else {
                // Normalize by motor speeds (1.5RPM for az, 0.25RPM for el)
                _az_priority = (errorAz / 1.5 > errorEl / 0.25);
            }
        }

        // Execute priority control
        if (_az_priority) {
            if (setPointState_az) {
                setPointState_el = false;
            } else {
                _az_priority = false;
            }
        } else {
            if (setPointState_el) {
                setPointState_az = false;
            } else {
                _az_priority = true;
            }
        }
    }

    // Adjust maximum speeds based on motor activity
    _az.maxAdjustedSpeed = setPointState_el ? max_dual_motor_az_speed : max_single_motor_az_speed;
    _el.maxAdjustedSpeed = setPointState_az ? max_dual_motor_el_speed : max_single_motor_el_speed;
}

void MotorSensorController::setPWM(int pin, int PWM) {
    analogWrite(pin, PWM);
}

// =============================================================================
// REMAINING METHODS (unchanged from original - angle calculation, sensor reading, etc.)
// =============================================================================

// =============================================================================
// ANGLE CALCULATION AND ERROR HANDLING (unchanged from original)
// =============================================================================

void MotorSensorController::angle_shortest_error_az(float target_angle, float current_angle) {
    // Normalize angles to 0-360 range
    target_angle = fmod(target_angle, 360);
    if (target_angle < 0) target_angle += 360;

    current_angle = fmod(current_angle, 360);
    if (current_angle < 0) current_angle += 360;

    // Handle edge cases at 0/360 boundary based on unwinding state
    if (needs_unwind >= 1 && current_angle < 90) {
        current_angle += 360;
    } else if (needs_unwind <= -1 && current_angle > 270) {
        current_angle -= 360;
    }

    // Calculate shortest path error
    float error = target_angle - current_angle;
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    // Handle unwinding requirements
    // Use tolerance for the zero check — Kalman prediction can wrap 0° to ~359.99°
    bool targetNearOrigin = (target_angle < 0.5f || target_angle > 359.5f);
    if (targetNearOrigin || ((current_angle + error) > 360 || (current_angle + error) < 0)) {
        if (needs_unwind <= -1) {
            error = (error > 180) ? error : error + 360;
        } else if (needs_unwind >= 1) {
            error = (error > -180) ? error - 360 : error;
        }
    }

    setErrorAz(error);
}

void MotorSensorController::angle_error_el(float target_angle, float current_angle) {
    // Map both angles to signed [-180, 180] centered at 0° (horizon), then take a
    // linear difference — NO shortest-path wrap. Elevation has a forbidden zone in
    // (90, 270): the dish cannot physically traverse 180°. A wrapped error would
    // pick that path whenever it was nominally shorter (e.g. dish at sensor 269.9,
    // target sensor 90 → wrapped error -179.9 → motor drives backward through the
    // forbidden zone). Linear error always routes motion through 0°.
    target_angle = fmod(target_angle, 360);
    if (target_angle < 0) target_angle += 360;
    if (target_angle > 180) target_angle -= 360;

    current_angle = fmod(current_angle, 360);
    if (current_angle < 0) current_angle += 360;
    if (current_angle > 180) current_angle -= 360;

    setErrorEl(target_angle - current_angle);
}

float MotorSensorController::correctAngle(float startAngle, float inputAngle) {
    float correctedAngle = inputAngle - startAngle;
    if (correctedAngle < 0) {
        correctedAngle += 360;
    } else if (correctedAngle >= 360) {
        correctedAngle -= 360;
    }
    return correctedAngle;
}

void MotorSensorController::calcIfNeedsUnwind(float correctedAngle_az) {
    // Determine current quadrant (1: 0-90, 2: 91-180, 3: 181-270, 4: 271-359)
    _quadrantNumber_az = (correctedAngle_az <= 90) ? 1 :
                        (correctedAngle_az <= 180) ? 2 :
                        (correctedAngle_az <= 270) ? 3 : 4;
    
    // Check for quadrant transitions
    if (_quadrantNumber_az != _previousquadrantNumber_az) {
        if (!calMode) {
            // Handle unwinding logic at 180-degree crossings
            // Use fetch_sub/fetch_add for explicit atomic read-modify-write
            if (_quadrantNumber_az == 2 && _previousquadrantNumber_az == 3) {
                needs_unwind.fetch_sub(1, std::memory_order_relaxed);
            } else if (_quadrantNumber_az == 3 && _previousquadrantNumber_az == 2) {
                needs_unwind.fetch_add(1, std::memory_order_relaxed);
            }
        }
        _previousquadrantNumber_az = _quadrantNumber_az;
    }
}

// =============================================================================
// SENSOR READING AND I2C COMMUNICATION (unchanged from original)
// =============================================================================

float MotorSensorController::readFilteredAngle(int i2c_addr, AxisState& axis) {
    // Do I2C read OUTSIDE the filter mutex — I2C bus has its own _i2cMutex protection.
    // This prevents the control loop from holding _getAngleMutex for the entire I2C
    // transaction (~50ms worst case), which was causing web handler timeouts on tare.
    float rawAngle = ReadRawAngle(i2c_addr);

    if (_getAngleMutex == NULL || xSemaphoreTake(_getAngleMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        _logger.error("Failed to take mutex in readFilteredAngle");
        badAngleFlag = true;
        return 0;
    }

    if (rawAngle == -999) {
        // I2C read failed — return previous filtered angle
        xSemaphoreGive(_getAngleMutex);
        if (!axis.angleFilterInitialized) {
            return 0;
        }
        float prevDeg = atan2f(axis.filteredAngleY, axis.filteredAngleX) * 180.0f / PI;
        if (prevDeg < 0) prevDeg += 360.0f;
        return prevDeg;
    }

    float rawRad = rawAngle * PI / 180.0f;
    float x = cosf(rawRad);
    float y = sinf(rawRad);

    // First reading: seed the filter directly
    if (!axis.angleFilterInitialized) {
        axis.filteredAngleX = x;
        axis.filteredAngleY = y;
        axis.angleFilterInitialized = true;
        xSemaphoreGive(_getAngleMutex);
        return rawAngle;
    }

    // Wraparound-safe jump rejection: compute shortest angular distance
    float filteredRad = atan2f(axis.filteredAngleY, axis.filteredAngleX);
    float diff = atan2f(sinf(rawRad - filteredRad), cosf(rawRad - filteredRad));
    float diffDeg = diff * 180.0f / PI;

    if (fabsf(diffDeg) > ANGLE_JUMP_THRESHOLD) {
        // Reject wild reading
        updateI2CErrorCounter(i2c_addr);
        xSemaphoreGive(_getAngleMutex);
        float prevDeg = filteredRad * 180.0f / PI;
        if (prevDeg < 0) prevDeg += 360.0f;
        return prevDeg;
    }

    // Apply EMA filter
    axis.filteredAngleX = ANGLE_EMA_ALPHA * x + (1.0f - ANGLE_EMA_ALPHA) * axis.filteredAngleX;
    axis.filteredAngleY = ANGLE_EMA_ALPHA * y + (1.0f - ANGLE_EMA_ALPHA) * axis.filteredAngleY;

    float filteredDeg = atan2f(axis.filteredAngleY, axis.filteredAngleX) * 180.0f / PI;
    if (filteredDeg < 0) filteredDeg += 360.0f;

    xSemaphoreGive(_getAngleMutex);
    return filteredDeg;
}

float MotorSensorController::ReadRawAngle(int i2c_addr) {
    uint8_t buffer[2];

    // Take shared I2C bus mutex to prevent contention with INA219
    if (_i2cMutex != NULL && xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return -999;  // I2C bus busy, skip this reading
    }

    // Request angle data
    Wire.beginTransmission(i2c_addr);
    Wire.write(0x0C);
    byte error = Wire.endTransmission(false);

    if (error != 0) {
        if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
        _logger.error("I2C error during transmission to sensor 0x" + String(i2c_addr, HEX) + ": " + String(error));
        updateI2CErrorCounter(i2c_addr);
        return -999;
    }

    delayMicroseconds(25);
    byte bytesReceived = Wire.requestFrom(i2c_addr, 2);

    if (bytesReceived != 2) {
        if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
        _logger.error("I2C error: Requested 2 bytes but received " + String(bytesReceived) + " from 0x" + String(i2c_addr, HEX));
        updateI2CErrorCounter(i2c_addr);
        return -999;
    }

    // Wire.setTimeOut() already handles timeout - just check if data is available
    if (Wire.available() >= 2) {
        Wire.readBytes(buffer, 2);
        if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
        word rawAngleValue = ((uint16_t)buffer[0] << 8) | buffer[1];
        resetI2CErrorCounter(i2c_addr);
        return rawAngleValue * 0.087890625;
    } else {
        if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
        _logger.error("I2C bytes not available from sensor");
        updateI2CErrorCounter(i2c_addr);
        return -999;
    }
}

int MotorSensorController::checkMagnetPresence(int i2c_addr) {
    int magnetStatus = 0;
    byte error;

    // Take shared I2C bus mutex
    if (_i2cMutex != NULL && xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        _logger.error("I2C bus busy during magnet check for 0x" + String(i2c_addr, HEX));
        return -999;
    }

    while (true) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(0x0B); // Status register
        error = Wire.endTransmission();

        if (error != 0) {
            _logger.error("I2C error during transmission to sensor 0x" + String(i2c_addr, HEX) + ": " + String(error));
            updateI2CErrorCounter(i2c_addr);

            // Check consecutive error limit
            if ((i2c_addr == _az_hall_i2c_addr && _az.consecutiveI2cErrors > MAX_CONSECUTIVE_ERRORS) ||
                (i2c_addr == _el_hall_i2c_addr && _el.consecutiveI2cErrors > MAX_CONSECUTIVE_ERRORS)) {
                if (i2c_addr == _az_hall_i2c_addr) i2cErrorFlag_az = true;
                else i2cErrorFlag_el = true;
                if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
                return -999;
            }
            continue;
        }

        Wire.requestFrom(i2c_addr, 1);

        // Wait for data with timeout instead of spinning forever
        unsigned long waitStart = millis();
        while (Wire.available() == 0) {
            if (millis() - waitStart > 1000) {
                _logger.error("I2C timeout waiting for magnet status from 0x" + String(i2c_addr, HEX));
                if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
                return -999;
            }
            delay(1);
        }

        magnetStatus = Wire.read();
        break;
    }

    if (_i2cMutex != NULL) xSemaphoreGive(_i2cMutex);
    resetI2CErrorCounter(i2c_addr);
    return magnetStatus;
}

// =============================================================================
// SETPOINT AND ANGLE ACCESS METHODS (updated with wind stow checks)
// =============================================================================

float MotorSensorController::getSetPointAz() {
    float result = 0;
    if (_setPointMutex != NULL && xSemaphoreTake(_setPointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _setpoint_az;
        xSemaphoreGive(_setPointMutex);
    }
    return result;
}

float MotorSensorController::getSetPointEl() {
    float result = 0;
    if (_setPointMutex != NULL && xSemaphoreTake(_setPointMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _setpoint_el;
        xSemaphoreGive(_setPointMutex);
    }
    return result;
}

void MotorSensorController::setCorrectedAngleAz(float value) {
    if (_correctedAngleMutex != NULL && xSemaphoreTake(_correctedAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _correctedAngle_az = value;
        xSemaphoreGive(_correctedAngleMutex);
    }
}

void MotorSensorController::setCorrectedAngleEl(float value) {
    if (_correctedAngleMutex != NULL && xSemaphoreTake(_correctedAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _correctedAngle_el = value;
        xSemaphoreGive(_correctedAngleMutex);
    }
}

float MotorSensorController::getCorrectedAngleAz() {
    float result = 0;
    if (_correctedAngleMutex != NULL && xSemaphoreTake(_correctedAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _correctedAngle_az;
        xSemaphoreGive(_correctedAngleMutex);
    }
    return result;
}

float MotorSensorController::getCorrectedAngleEl() {
    float result = 0;
    if (_correctedAngleMutex != NULL && xSemaphoreTake(_correctedAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _correctedAngle_el;
        xSemaphoreGive(_correctedAngleMutex);
    }
    return result;
}

double MotorSensorController::getAxisError(const AxisState& axis) {
    double result = 0;
    if (_errorMutex != NULL && xSemaphoreTake(_errorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = *axis.error;
        xSemaphoreGive(_errorMutex);
    }
    return result;
}

double MotorSensorController::getErrorAz() {
    return getAxisError(_az);
}

void MotorSensorController::setErrorAz(float value) {
    if (_errorMutex != NULL && xSemaphoreTake(_errorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _error_az = value;
        xSemaphoreGive(_errorMutex);
    }
}

double MotorSensorController::getErrorEl() {
    return getAxisError(_el);
}

void MotorSensorController::setErrorEl(float value) {
    if (_errorMutex != NULL && xSemaphoreTake(_errorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _error_el = value;
        xSemaphoreGive(_errorMutex);
    }
}

float MotorSensorController::getElStartAngle() {
    float result = 0;
    if (_el_startAngleMutex != NULL && xSemaphoreTake(_el_startAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _el_startAngle;
        xSemaphoreGive(_el_startAngleMutex);
    }
    return result;
}

void MotorSensorController::setElStartAngle(float value) {
    if (_el_startAngleMutex != NULL && xSemaphoreTake(_el_startAngleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _el_startAngle = value;
        xSemaphoreGive(_el_startAngleMutex);
    }

    // NVS write outside mutex — can block during wear-leveling
    _preferences.putFloat("el_cal", value);
}

int MotorSensorController::getMinVoltageThreshold() {
    return _minVoltageThreshold;
}

void MotorSensorController::setMinVoltageThreshold(int value) {
    if (value > 0 && value < 20) {
        _minVoltageThreshold = value;
        _preferences.putInt("MIN_VOLTAGE", value);
        _logger.info("MIN_VOLTAGE_THRESHOLD set to: " + String(value) + "V");
    }
}

int MotorSensorController::getMaxPowerFaultAz() {
    return _maxPowerFaultAz;
}

void MotorSensorController::setMaxPowerFaultAz(int value) {
    if (value >= 1 && value <= 25) {
        _maxPowerFaultAz = value;
        _preferences.putInt("MAX_PWR_AZ", value);
        _logger.info("MAX_FAULT_POWER_AZ set to: " + String(value) + "W");
    }
}

int MotorSensorController::getMaxPowerFaultEl() {
    return _maxPowerFaultEl;
}

void MotorSensorController::setMaxPowerFaultEl(int value) {
    if (value >= 1 && value <= 25) {
        _maxPowerFaultEl = value;
        _preferences.putInt("MAX_PWR_EL", value);
        _logger.info("MAX_FAULT_POWER_EL set to: " + String(value) + "W");
    }
}

int MotorSensorController::getMaxPowerFaultTotal() {
    return _maxPowerFaultTotal;
}

void MotorSensorController::setMaxPowerFaultTotal(int value) {
    if (value >= 1 && value <= 25) {
        _maxPowerFaultTotal = value;
        _preferences.putInt("MAX_PWR_TOT", value);
        _logger.info("MAX_FAULT_POWER_TOTAL set to: " + String(value) + "W");
    }
}

int MotorSensorController::getActivePowerThreshold() {
    bool azActive = setPointState_az.load() && !_isAzMotorLatched.load() && !global_fault.load();
    bool elActive = setPointState_el.load() && !_isElMotorLatched.load() && !global_fault.load();
    if (azActive && elActive) return _maxPowerFaultTotal.load();
    if (azActive) return _maxPowerFaultAz.load();
    if (elActive) return _maxPowerFaultEl.load();
    return _maxPowerFaultTotal.load();
}

// =============================================================================
// CONFIGURATION PARAMETER SETTERS (unchanged from original)
// =============================================================================

int MotorSensorController::getMotorSpeedPctAz() const {
    // PWM 0 = full speed, 255 = stopped. currentSpeed is the actual PWM value.
    // When motor is stopped, currentSpeed = MIN_AZ_SPEED but PWM pin is set to 255.
    // Report based on active state: if not active, 0%.
    if (!setPointState_az.load() || _isAzMotorLatched.load() || global_fault.load()) return 0;
    int pct = (int)(100.0f * (1.0f - (float)_az.currentSpeed / 255.0f));
    return constrain(pct, 0, 100);
}

int MotorSensorController::getMotorSpeedPctEl() const {
    if (!setPointState_el.load() || _isElMotorLatched.load() || global_fault.load()) return 0;
    int pct = (int)(100.0f * (1.0f - (float)_el.currentSpeed / 255.0f));
    return constrain(pct, 0, 100);
}

void MotorSensorController::setPEl(int value) {
    if (value >= 0 && value <= 1000) {
        P_el = value;
        _preferences.putInt("P_el", value);
        _logger.info("P_el set to: " + String(value));
    }
}

void MotorSensorController::setPAz(int value) {
    if (value >= 0 && value <= 1000) {
        P_az = value;
        _preferences.putInt("P_az", value);
        _logger.info("P_az set to: " + String(value));
    }
}

void MotorSensorController::setIEl(float value) {
    if (value >= 0.0f && value <= 1000.0f) {
        I_el = value;
        _preferences.putFloat("I_el", value);
        _logger.info("I_el set to: " + String(value));
    }
}

void MotorSensorController::setIAz(float value) {
    if (value >= 0.0f && value <= 1000.0f) {
        I_az = value;
        _preferences.putFloat("I_az", value);
        _logger.info("I_az set to: " + String(value));
    }
}

void MotorSensorController::setDEl(float value) {
    if (value >= 0.0f && value <= 1000.0f) {
        D_el = value;
        _preferences.putFloat("D_el", value);
        _logger.info("D_el set to: " + String(value));
    }
}

void MotorSensorController::setDAz(float value) {
    if (value >= 0.0f && value <= 1000.0f) {
        D_az = value;
        _preferences.putFloat("D_az", value);
        _logger.info("D_az set to: " + String(value));
    }
}

void MotorSensorController::setMinElSpeed(int value) {
    if (value >= 0 && value <= 255) {
        int oldMin = MIN_EL_SPEED.load();
        float dualElPct = convertSpeedToPercentage((float)max_dual_motor_el_speed, oldMin);
        float singleElPct = convertSpeedToPercentage((float)max_single_motor_el_speed, oldMin);

        MIN_EL_SPEED = value;
        _preferences.putInt("MIN_EL_SPEED", value);

        max_dual_motor_el_speed = convertPercentageToSpeed(dualElPct, value);
        max_single_motor_el_speed = convertPercentageToSpeed(singleElPct, value);
        _preferences.putInt("maxDMElSpeed", max_dual_motor_el_speed);
        _preferences.putInt("maxSMElSpeed", max_single_motor_el_speed);

        _logger.info("MIN_EL_SPEED set to: " + String(value));
    }
}

void MotorSensorController::setMinAzSpeed(int value) {
    if (value >= 0 && value <= 255) {
        int oldMin = MIN_AZ_SPEED.load();
        float dualAzPct = convertSpeedToPercentage((float)max_dual_motor_az_speed, oldMin);
        float singleAzPct = convertSpeedToPercentage((float)max_single_motor_az_speed, oldMin);

        MIN_AZ_SPEED = value;
        _preferences.putInt("MIN_AZ_SPEED", value);

        max_dual_motor_az_speed = convertPercentageToSpeed(dualAzPct, value);
        max_single_motor_az_speed = convertPercentageToSpeed(singleAzPct, value);
        _preferences.putInt("maxDMAzSpeed", max_dual_motor_az_speed);
        _preferences.putInt("maxSMAzSpeed", max_single_motor_az_speed);

        _logger.info("MIN_AZ_SPEED set to: " + String(value));
    }
}

void MotorSensorController::setMinAzTolerance(float value) {
    if (value > 0 && value <= 10.0) {
        _MIN_AZ_TOLERANCE = value;
        _preferences.putFloat("MIN_AZ_TOL", value);
        _logger.info("MIN_AZ_TOLERANCE set to: " + String(value));
    }
}

void MotorSensorController::setMinElTolerance(float value) {
    if (value > 0 && value <= 10.0) {
        _MIN_EL_TOLERANCE = value;
        _preferences.putFloat("MIN_EL_TOL", value);
        _logger.info("MIN_EL_TOLERANCE set to: " + String(value));
    }
}

float MotorSensorController::getAzOffset() {
    float result = 0.0;
    if (_offsetMutex != NULL && xSemaphoreTake(_offsetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _az_offset;
        xSemaphoreGive(_offsetMutex);
    }
    return result;
}

void MotorSensorController::setAzOffset(float offset) {
    // Validate offset range (±180 degrees should be sufficient)
    if (isnan(offset) || offset < -180.0 || offset > 180.0) {
        _logger.warn("AZ offset out of range: " + String(offset, 3) + "° (range: ±180°)");
        return;
    }

    if (_offsetMutex != NULL && xSemaphoreTake(_offsetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _az_offset = offset;
        _setPointAzUpdated = true;
        xSemaphoreGive(_offsetMutex);
    }

    // Reset direction lock — offset shifts the coordinate frame,
    // invalidating the tracked movement direction
    _az.dirLockDirection = 0;
    _az.dirLockHasTracked = false;
    _az.dirLockChangeCount = 0;

    // Explicitly unlatch — previous latch position is no longer valid
    _isAzMotorLatched = false;

    // NVS write outside mutex — can block during wear-leveling
    _preferences.putFloat("az_offset", offset);
    _logger.info("AZ angle offset set to: " + String(offset, 3) + "°");
}

float MotorSensorController::getElOffset() {
    float result = 0.0;
    if (_offsetMutex != NULL && xSemaphoreTake(_offsetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        result = _el_offset;
        xSemaphoreGive(_offsetMutex);
    }
    return result;
}

void MotorSensorController::setElOffset(float offset) {
    // Validate offset range (±5 degrees should be sufficient for elevation)
    if (offset < -5.0 || offset > 5.0) {
        _logger.warn("EL offset out of range: " + String(offset, 3) + "° (range: ±5°)");
        return;
    }

    if (_offsetMutex != NULL && xSemaphoreTake(_offsetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _el_offset = offset;
        _setPointElUpdated = true;
        xSemaphoreGive(_offsetMutex);
    }

    // Reset direction lock — offset shifts the coordinate frame,
    // invalidating the tracked movement direction
    _el.dirLockDirection = 0;
    _el.dirLockHasTracked = false;
    _el.dirLockChangeCount = 0;

    // Explicitly unlatch — previous latch position is no longer valid
    _isElMotorLatched = false;

    // NVS write outside mutex — can block during wear-leveling
    _preferences.putFloat("el_offset", offset);
    _logger.info("EL angle offset set to: " + String(offset, 3) + "°");
}

// Helper methods to get offset-adjusted start angles
float MotorSensorController::getAdjustedAzStartAngle() {
    // Subtract offset from start angle (this adds offset to final corrected angle)
    return _az_startAngle - getAzOffset();
}

float MotorSensorController::getAdjustedElStartAngle() {
    // Subtract offset from start angle (this adds offset to final corrected angle) 
    return getElStartAngle() - getElOffset();
}

// =============================================================================
// CALIBRATION METHODS (unchanged from original)
// =============================================================================

void MotorSensorController::activateCalMode(bool on) {
    if (_calMutex != NULL && xSemaphoreTake(_calMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (on) {
            calMode = true;
            global_fault = false;
            setPWM(_pwm_pin_az, 255);
            setPWM(_pwm_pin_el, 255);
            _logger.info("calMode set to true");
        } else {
            calMode = false;
            _calRunTime = 0;  // Also clear any pending cal movement
            _calAxis[0] = '\0';

            // Reset motor latches and PID state so motors resume moving to setpoint
            _isAzMotorLatched = false;
            _isElMotorLatched = false;
            _az.prevError = 0;
            _az.errorIntegral = 0.0;
            _az.prevRawError = 0.0;
            _az.filteredDTerm = 0.0;
            _el.prevError = 0;
            _el.errorIntegral = 0.0;
            _el.prevRawError = 0.0;
            _el.filteredDTerm = 0.0;

            _logger.info("calMode set to false");
        }
        xSemaphoreGive(_calMutex);
    }
}

void MotorSensorController::calMoveMotor(const char* runTimeStr, const char* axis) {
    if (_calMutex != NULL && xSemaphoreTake(_calMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!calMode) {
            xSemaphoreGive(_calMutex);
            Serial.println("Calibration mode OFF; ignoring calMove request.");
            return;
        }
        _calRunTime = atoi(runTimeStr);
        safeCopy(_calAxis, axis, sizeof(_calAxis));
        xSemaphoreGive(_calMutex);
    }
}

void MotorSensorController::calibrate_elevation() {
    if (calMode) {
        float tareAngle = readFilteredAngle(_el_hall_i2c_addr, _el);
        setElStartAngle(tareAngle);
        Serial.println("EL CAL DONE");
    }
}

void MotorSensorController::handleCalibrationMode() {
    int calRunTime = 0;
    char calAxis[4] = "";
    int calState = 0;
    unsigned long calMoveStartTime = 0;

    // Read calibration state under mutex, and handle state 0→1 transition
    // in the same acquisition to avoid a second mutex take per cycle
    if (_calMutex != NULL && xSemaphoreTake(_calMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        calRunTime = _calRunTime;
        safeCopy(calAxis, _calAxis, sizeof(calAxis));
        calState = _calState;
        calMoveStartTime = _calMoveStartTime;

        // If state 0 with pending move, transition to state 1 immediately
        if (calState == 0 && abs(calRunTime) > 0 && calAxis[0] != '\0') {
            _calMoveStartTime = millis();
            _calState = 1;
            calState = 1;
            calMoveStartTime = _calMoveStartTime;
        }
        xSemaphoreGive(_calMutex);
    }

    if (calState == 0) {
        if (abs(calRunTime) == 0 || calAxis[0] == '\0') {
            analogWrite(_pwm_pin_az, 255);
            analogWrite(_pwm_pin_el, 255);
            digitalWrite(_pwm_pin_az, 1);
            digitalWrite(_pwm_pin_el, 1);
        }
    } else if (calState == 1) {
        int directionPin, pwmPin;
        bool invert = false;

        if (strcasecmp(calAxis, "AZ") == 0) {
            directionPin = _ccw_pin_az;
            pwmPin = _pwm_pin_az;
            invert = _azCfg.invertDir;
        } else if (strcasecmp(calAxis, "EL") == 0) {
            directionPin = _ccw_pin_el;
            pwmPin = _pwm_pin_el;
            invert = _elCfg.invertDir;
        } else {
            _logger.error("Invalid calibration axis: " + String(calAxis));
            if (_calMutex != NULL && xSemaphoreTake(_calMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                _calRunTime = 0;
                _calAxis[0] = '\0';
                _calState = 0;
                xSemaphoreGive(_calMutex);
            }
            return;
        }

        digitalWrite(directionPin, (calRunTime > 0) ^ invert);
        analogWrite(pwmPin, 0);

        unsigned long elapsedTime = millis() - calMoveStartTime;
        if (elapsedTime > (unsigned long)abs(calRunTime)) {
            analogWrite(_pwm_pin_az, 255);
            analogWrite(_pwm_pin_el, 255);

            if (_calMutex != NULL && xSemaphoreTake(_calMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                _calRunTime = 0;
                _calAxis[0] = '\0';
                _calState = 0;
                xSemaphoreGive(_calMutex);
            }
        }
    }
}

// =============================================================================
// UTILITY METHODS (unchanged from original)
// =============================================================================

void MotorSensorController::handleOscillationDetection() {
    // Load the atomic value ONCE and use it consistently throughout
    int current_unwind = needs_unwind.load(std::memory_order_relaxed);
    
    // Update needs_unwind in preferences when changed (outside calibration mode)
    if (current_unwind != _prev_needs_unwind && !calMode) {
        _preferences.putInt("needs_unwind", current_unwind);
        _prev_needs_unwind = current_unwind;

        _logger.warn("THIS SHOULD NOT BE RUNNING CONSTANTLY OR THE EEPROM COULD CORRUPT");

        // Start or continue oscillation detection
        if (!_oscillationTimerActive) {
            _oscillationTimerStart = millis();
            _oscillationTimerActive = true;
            _oscillationCount = 1;
            _logger.info("Oscillation detection timer started");
        } else {
            _oscillationCount++;
            _logger.info("Oscillation count: " + String(_oscillationCount));
            
            // Check for excessive oscillation
            if (_oscillationCount >= 10) {
                float currentAngle = getCorrectedAngleAz();
                float newSetpoint = (currentAngle <= 180.0) ? currentAngle - 1 : currentAngle + 1;
                
                _logger.warn("Excessive oscillation detected! Moving " + String((currentAngle <= 180.0) ? "-1°" : "+1°"));
                setSetPointAzInternal(newSetpoint);
                
                _oscillationTimerActive = false;
                _oscillationCount = 0;
            }
        }
    }

    // Reset timer after 60 seconds
    if (_oscillationTimerActive && (millis() - _oscillationTimerStart >= 60000)) {
        _oscillationTimerActive = false;
        _oscillationCount = 0;
        _logger.info("Oscillation detection timer expired, count was: " + String(_oscillationCount));
    }
}

void MotorSensorController::slowPrint(const String& message, int messageID) {
    static unsigned long lastPrintTimes[10] = {0};
    const unsigned long printDelay = 1000;
    
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTimes[messageID] >= printDelay) {
        _logger.error(message);
        lastPrintTimes[messageID] = currentTime;
    }
}

void MotorSensorController::updateI2CErrorCounter(int i2c_addr) {
    if (i2c_addr == _az_hall_i2c_addr) {
        if (_az.consecutiveI2cErrors < 255) _az.consecutiveI2cErrors++;
        if (_az.consecutiveI2cErrors >= MAX_CONSECUTIVE_ERRORS) {
            i2cErrorFlag_az = true;
        }
    } else if (i2c_addr == _el_hall_i2c_addr) {
        if (_el.consecutiveI2cErrors < 255) _el.consecutiveI2cErrors++;
        if (_el.consecutiveI2cErrors >= MAX_CONSECUTIVE_ERRORS) {
            i2cErrorFlag_el = true;
        }
    }
}

void MotorSensorController::resetI2CErrorCounter(int i2c_addr) {
    if (i2c_addr == _az_hall_i2c_addr) {
        _az.consecutiveI2cErrors = 0;
    } else if (i2c_addr == _el_hall_i2c_addr) {
        _el.consecutiveI2cErrors = 0;
    }
}

void MotorSensorController::forceStopMs(unsigned long ms) {
    _forceStopUntil = millis() + ms;
    analogWrite(_pwm_pin_az, 255);
    analogWrite(_pwm_pin_el, 255);
}

int MotorSensorController::convertPercentageToSpeed(float percentage, int minSpeed) {
    return (int)((1 - (percentage / 100.0)) * minSpeed);
}

int MotorSensorController::convertSpeedToPercentage(float speed, int minSpeed) {
    if (minSpeed == 0) return 0;
    return (int)(100 * (1 - (speed / minSpeed)));
}

void MotorSensorController::playOdeToJoy() {
    // Stop motors and enter calibration mode temporarily
    setPWM(_pwm_pin_az, 255);
    setPWM(_pwm_pin_el, 255);
    
    bool previousCalMode = calMode;
    activateCalMode(true);
    
    // Musical note frequencies
    const int NOTE_D3 = 147, NOTE_CS4 = 277, NOTE_D4 = 294, NOTE_E4 = 330;
    const int NOTE_FS4 = 370, NOTE_G4 = 392, NOTE_A4 = 440, NOTE_B4 = 494;
    
    // Ode to Joy melody
    const int melody[] = {
        NOTE_E4, NOTE_E4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D4,
        NOTE_CS4, NOTE_CS4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4,
        NOTE_E4, NOTE_E4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D4,
        NOTE_CS4, NOTE_CS4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_CS4, NOTE_CS4,
        NOTE_D4, NOTE_D4, NOTE_E4, NOTE_CS4, NOTE_D4, NOTE_E4, NOTE_FS4, NOTE_E4,
        NOTE_CS4, NOTE_D4, NOTE_E4, NOTE_FS4, NOTE_E4, NOTE_D4, NOTE_CS4, NOTE_D4, NOTE_D3
    };

    const int noteDurations[] = {
        250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 375, 125, 500,
        250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 375, 125, 500,
        250, 250, 250, 250, 250, 125, 125, 250, 250, 250, 125, 125, 250, 250, 250, 250, 250
    };
    
    int musicPin = _pwm_pin_az;
    const int soundPWM = 128;
    
    // Play melody
    digitalWrite(_ccw_pin_az, 1);
    Serial.println("Playing Ode to Joy on motors...");
    
    for (int noteIndex = 0; noteIndex < sizeof(melody)/sizeof(melody[0]); noteIndex++) {
        analogWriteFrequency(musicPin, melody[noteIndex]);   
        analogWrite(musicPin, soundPWM);
        delay(noteDurations[noteIndex]);
        analogWrite(musicPin, 255);
        delay(30);
    }
    
    // Reset and restore state
    analogWriteFrequency(musicPin, FREQ);
    setPWM(musicPin, 255);
    activateCalMode(previousCalMode);
    
    _logger.info("Ode to Joy finished");
}

// =============================================================================
// SMOOTH TRACKING METHODS (Kalman filter)
// =============================================================================

void MotorSensorController::setSmoothTrackingEnabled(bool enabled) {
    bool wasEnabled = _smoothTrackingEnabled.load();
    _smoothTrackingEnabled = enabled;
    _preferences.putBool("smoothTrack", enabled);

    if (wasEnabled != enabled) {
        resetSmoothTracking();
        _az.errorIntegral = 0.0;
        _az.prevRawError = 0.0;
        _az.filteredDTerm = 0.0;
        _az.prevPidOutput = 0.0;
        _el.errorIntegral = 0.0;
        _el.prevRawError = 0.0;
        _el.filteredDTerm = 0.0;
        _el.prevPidOutput = 0.0;
        _isAzMotorLatched = false;
        _isElMotorLatched = false;
    }
    _logger.info("Smooth tracking " + String(enabled ? "enabled" : "disabled"));
}

void MotorSensorController::resetSmoothTracking() {
    _kalmanAz = KalmanState();
    _kalmanEl = KalmanState();
}

void MotorSensorController::kalmanPredict(KalmanState& kf, float dt, bool isAzimuth) {
    if (!kf.initialized) return;

    // Decay velocity if no measurement updates received recently
    // Prevents drift when target stops (setpoint unchanged = no updates)
    unsigned long now = millis();
    if (kf.lastUpdateMs > 0) {
        unsigned long elapsed = now - kf.lastUpdateMs;
        if (elapsed > KALMAN_VEL_DECAY_MS) {
            float decayAlpha = 1.0f - expf(-dt / KALMAN_VEL_DECAY_TAU);
            kf.vel *= (1.0 - decayAlpha);
        }
    }

    // State prediction: constant velocity model
    kf.pos += kf.vel * dt;

    kalmanClampBounds(kf, isAzimuth);

    // Covariance prediction: P = F*P*F' + Q
    // F = [[1, dt], [0, 1]], Q_discrete = q * [[dt³/3, dt²/2], [dt²/2, dt]]
    double q = _kalmanQ;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double new_p00 = kf.p00 + dt * (kf.p10 + kf.p01 + dt * kf.p11) + q * dt3 / 3.0;
    double new_p01 = kf.p01 + dt * kf.p11 + q * dt2 / 2.0;
    double new_p10 = kf.p10 + dt * kf.p11 + q * dt2 / 2.0;
    double new_p11 = kf.p11 + q * dt;

    kf.p00 = new_p00;
    kf.p01 = new_p01;
    kf.p10 = new_p10;
    kf.p11 = new_p11;
}

void MotorSensorController::kalmanUpdate(KalmanState& kf, float measurement, bool isAzimuth, float rOverride) {
    if (!kf.initialized) {
        kf.pos = measurement;
        kf.vel = 0.0;
        kf.p00 = 1.0;
        kf.p01 = 0.0;
        kf.p10 = 0.0;
        kf.p11 = 1.0;
        kf.lastUpdateMs = millis();
        kf.initialized = true;
        return;
    }

    // Innovation with wraparound handling for azimuth
    double innovation = (double)measurement - kf.pos;
    if (isAzimuth) {
        while (innovation > 180.0) innovation -= 360.0;
        while (innovation < -180.0) innovation += 360.0;
    }

    // Large jump detection — reinitialize filter (new target selected)
    if (fabs(innovation) > KALMAN_JUMP_THRESHOLD) {
        kf.pos = measurement;
        kf.vel = 0.0;
        kf.p00 = 1.0;
        kf.p01 = 0.0;
        kf.p10 = 0.0;
        kf.p11 = 1.0;
        kf.lastUpdateMs = millis();
        return;
    }

    // Innovation covariance: S = H*P*H' + R = p00 + R
    double r = (rOverride > 0.0f) ? (double)rOverride : (double)_kalmanR;
    double s = kf.p00 + r;

    // Kalman gain: K = P*H'/S
    double k0 = kf.p00 / s;  // position gain
    double k1 = kf.p10 / s;  // velocity gain

    // State update
    kf.pos += k0 * innovation;
    kf.vel += k1 * innovation;

    kalmanClampBounds(kf, isAzimuth);

    // Covariance update: P = (I - K*H) * P
    double new_p00 = kf.p00 - k0 * kf.p00;
    double new_p01 = kf.p01 - k0 * kf.p01;
    double new_p10 = kf.p10 - k1 * kf.p00;
    double new_p11 = kf.p11 - k1 * kf.p01;

    kf.p00 = new_p00;
    kf.p01 = new_p01;
    kf.p10 = new_p10;
    kf.p11 = new_p11;

    // Only update timestamp for real measurements (not synthetic convergence)
    if (rOverride < 0.0f) {
        kf.lastUpdateMs = millis();
    }
}

void MotorSensorController::kalmanClampBounds(KalmanState& kf, bool isAzimuth) {
    if (isAzimuth) {
        // AZ: wrap to [0, 360), but clamp to [-360, 720] to respect needs_unwind range
        // The error calculation in angle_shortest_error_az handles the actual ±360 logic
        while (kf.pos >= 360.0) kf.pos -= 360.0;
        while (kf.pos < 0.0) kf.pos += 360.0;
    } else {
        // EL: hard clamp to valid elevation range, zero velocity at bounds
        float minEl = _extendedElEnabled.load() ? -90.0f : 0.0f;
        float maxEl = 90.0f;
        if (kf.pos > maxEl) {
            kf.pos = maxEl;
            if (kf.vel > 0.0) kf.vel = 0.0;
        } else if (kf.pos < minEl) {
            kf.pos = minEl;
            if (kf.vel < 0.0) kf.vel = 0.0;
        }
    }
}

// Smooth tracking parameter setters
void MotorSensorController::setKalmanQ(float v) {
    if (v >= 0.01f && v <= 100.0f) { _kalmanQ = v; _preferences.putFloat("smKalQ", v); _logger.info("Kalman Q set to: " + String(v)); }
}
void MotorSensorController::setKalmanR(float v) {
    if (v >= 0.01f && v <= 100.0f) { _kalmanR = v; _preferences.putFloat("smKalR", v); _logger.info("Kalman R set to: " + String(v)); }
}
void MotorSensorController::setMinSmoothAzSpeed(int v) {
    if (v >= 0 && v <= 255) { MIN_SMOOTH_AZ_SPEED = v; _preferences.putInt("smMinAzSpd", v); _logger.info("smMinAzSpd set to: " + String(v)); }
}
void MotorSensorController::setMinSmoothElSpeed(int v) {
    if (v >= 0 && v <= 255) { MIN_SMOOTH_EL_SPEED = v; _preferences.putInt("smMinElSpd", v); _logger.info("smMinElSpd set to: " + String(v)); }
}

