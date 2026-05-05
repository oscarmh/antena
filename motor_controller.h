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

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

// System includes
#include <Arduino.h>
#include <atomic>
#include <Wire.h>
#include <Preferences.h>

// Custom includes
#include "INA219_manager.h"
#include "logger.h"

// Forward declaration to avoid circular dependency
class WeatherPoller;

struct WindStowState {
    bool active = false;
    char reason[64] = "None";
    float direction = 0.0f;
};

struct WindTrackingState {
    bool active = false;
    char status[64] = "Inactive";
    float lastDirection = 0.0f;
};

class MotorSensorController {
public:
    // Constructor
    MotorSensorController(Preferences& prefs, INA219Manager& ina219Manager, Logger& logger);

    // Core control methods
    void begin();
    void runControlLoop();
    void runSafetyLoop();

    // Weather integration
    void setWeatherPoller(WeatherPoller* weatherPoller);

    // I2C bus mutex (shared with INA219Manager to prevent bus contention)
    void setI2CMutex(SemaphoreHandle_t mutex);

    // Setpoint and angle access methods
    float getSetPointAz();
    float getSetPointEl();
    void setSetPointAz(float setpoint_az);
    void setSetPointEl(float setpoint_el);
    void setErrorAz(float value);
    void setErrorEl(float value);

    double getErrorAz();
    double getErrorEl();
    float getCorrectedAngleAz();
    float getCorrectedAngleEl();
    void setCorrectedAngleAz(float value);
    void setCorrectedAngleEl(float value);

    float getElStartAngle();
    void setElStartAngle(float value);
    int getMaxPowerFaultAz();
    void setMaxPowerFaultAz(int value);
    int getMaxPowerFaultEl();
    void setMaxPowerFaultEl(int value);
    int getMaxPowerFaultTotal();
    void setMaxPowerFaultTotal(int value);
    int getActivePowerThreshold();
    int getMinVoltageThreshold();
    void setMinVoltageThreshold(int value);

    // Angle offset methods (NEW)
    float getAzOffset();
    void setAzOffset(float offset);
    float getElOffset();
    void setElOffset(float offset);

    // Motor speed percentage getters (0% = stopped, 100% = max speed)
    int getMotorSpeedPctAz() const;
    int getMotorSpeedPctEl() const;

    // Configuration parameter getters
    int getPEl() const { return P_el; }
    int getPAz() const { return P_az; }
    float getIEl() const { return I_el; }
    float getIAz() const { return I_az; }
    float getDEl() const { return D_el; }
    float getDAz() const { return D_az; }
    int getMinElSpeed() const { return MIN_EL_SPEED; }
    int getMinAzSpeed() const { return MIN_AZ_SPEED; }
    float getMinAzTolerance() const { return _MIN_AZ_TOLERANCE; }
    float getMinElTolerance() const { return _MIN_EL_TOLERANCE; }

    // Configuration parameter setters
    void setPEl(int value);
    void setPAz(int value);
    void setIEl(float value);
    void setIAz(float value);
    void setDEl(float value);
    void setDAz(float value);
    void setMinElSpeed(int value);
    void setMinAzSpeed(int value);
    void setMinAzTolerance(float value);
    void setMinElTolerance(float value);

    // Direction lock methods
    bool isDirectionLockEnabled() const { return _directionLockEnabled.load(); }
    void setDirectionLockEnabled(bool enabled);

    // Calibration and special functions
    void activateCalMode(bool on);
    void calMoveMotor(const char* runTimeStr, const char* axis);
    void calibrate_elevation();
    void playOdeToJoy();

    // Wind safety methods
    WindStowState getWindStowState();
    bool isWindStowActive();
    void performWindStow();
    bool isMovementBlocked();

    // Wind tracking methods
    WindTrackingState getWindTrackingState();
    bool isWindTrackingActive();
    const char* getWindTrackingStatus();

    // Motor stop
    void forceStopMs(unsigned long ms);

    // Utility methods
    int convertPercentageToSpeed(float percentage, int minSpeed);
    int convertSpeedToPercentage(float speed, int minSpeed);
    void handleCalibrationMode();
    void handleOscillationDetection();
    void updateI2CErrorCounter(int i2c_addr);
    void resetI2CErrorCounter(int i2c_addr);

    // Motor control state
    std::atomic<bool> setPointState_az = false;
    std::atomic<bool> setPointState_el = false;
    std::atomic<bool> _isAzMotorLatched = false;
    std::atomic<bool> _isElMotorLatched = false;

    // Operating modes
    std::atomic<bool> calMode = false;
    std::atomic<bool> singleMotorMode = false;
    std::atomic<int> needs_unwind = 0;

    // Safe mode — blocks external sources (rotctl, serial, stellarium) but not web UI
    std::atomic<bool> safeMode{false};
    bool isSafeMode() const { return safeMode.load(); }
    void setSafeMode(bool enabled);

    // Wind stow state
    std::atomic<bool> _windStowActive = false;

    // Wind tracking state
    std::atomic<bool> _windTrackingActive = false;

    // Direction lock state
    std::atomic<bool> _directionLockEnabled = true;

    // Extended elevation state
    std::atomic<bool> _extendedElEnabled = false;
    bool isExtendedElEnabled() const { return _extendedElEnabled.load(); }
    void setExtendedElEnabled(bool enabled);

    // Flip mode: re-expresses the extended-elevation [-90, +90] range as user-facing
    // [0, 180] where 0=zenith (internal +90), 90=horizon (internal 0), 180=flipped over
    // (internal -90). Lets a satellite pass over zenith be tracked without an AZ flip
    // by sweeping the elevation axis through 180° instead. Requires extended elevation.
    std::atomic<bool> _flipModeEnabled = false;
    bool isFlipModeEnabled() const { return _flipModeEnabled.load(); }
    void setFlipModeEnabled(bool enabled);
    static float flipToInternal(float flipDeg) { return 90.0f - flipDeg; }
    static float internalToFlip(float internalDeg) { return 90.0f - internalDeg; }
    // Rest/home elevation in internal coords. Flip mode mounts the antenna boom such
    // that the user-facing 0° (forward horizon) lives at internal +90°, so home/park
    // /auto-home/wind-tracking all need to track the mode rather than hardcoding 0.
    float getHomeElInternal() const { return _flipModeEnabled.load() ? 90.0f : 0.0f; }

    // Auto-home state
    std::atomic<bool> _autoHomeEnabled{false};
    bool isAutoHomeEnabled() const { return _autoHomeEnabled.load(); }
    void setAutoHomeEnabled(bool enabled);
    void setAutoHomeTimeout(int minutes);
    int getAutoHomeTimeout() const;

    // Smooth tracking
    bool isSmoothTrackingEnabled() const { return _smoothTrackingEnabled.load(); }
    void setSmoothTrackingEnabled(bool enabled);
    float getKalmanQ() const { return _kalmanQ; }
    float getKalmanR() const { return _kalmanR; }
    float getKalmanAzPos() const { return _kalmanAz.initialized ? (float)_kalmanAz.pos : -1.0f; }
    float getKalmanElPos() const { return _kalmanEl.initialized ? (float)_kalmanEl.pos : -1.0f; }
    float getKalmanAzVel() const { return _kalmanAz.initialized ? (float)_kalmanAz.vel : 0.0f; }
    float getKalmanElVel() const { return _kalmanEl.initialized ? (float)_kalmanEl.vel : 0.0f; }
    int getMinSmoothAzSpeed() const { return MIN_SMOOTH_AZ_SPEED.load(); }
    int getMinSmoothElSpeed() const { return MIN_SMOOTH_EL_SPEED.load(); }
    void setKalmanQ(float v);
    void setKalmanR(float v);
    void setMinSmoothAzSpeed(int v);
    void setMinSmoothElSpeed(int v);

    // Fault and error flags
    std::atomic<bool> global_fault = false;
    std::atomic<bool> outOfBoundsFault = false;
    std::atomic<bool> overSpinFault = false;
    std::atomic<bool> magnetFault = false;
    std::atomic<bool> badAngleFlag = false;
    std::atomic<bool> overPowerFault = false;
    std::atomic<bool> lowVoltageFault = false;
    std::atomic<bool> i2cErrorFlag_az = false;
    std::atomic<bool> i2cErrorFlag_el = false;


    // Motor speed configuration
    std::atomic<int> MIN_EL_SPEED = 50;
    std::atomic<int> MIN_AZ_SPEED = 100;
    std::atomic<int> max_dual_motor_az_speed = MAX_AZ_SPEED;
    std::atomic<int> max_dual_motor_el_speed = MAX_EL_SPEED;
    std::atomic<int> max_single_motor_az_speed = 0;
    std::atomic<int> max_single_motor_el_speed = 0;

private:
    // Dependencies
    Preferences& _preferences;
    INA219Manager& ina219Manager;
    Logger& _logger;
    WeatherPoller* _weatherPoller = nullptr;

    // Hardware configuration constants
    static constexpr int _el_hall_i2c_addr = 0x36;  // AS5600
    static constexpr int _az_hall_i2c_addr = 0x40;  // AS5600L

    static constexpr int _pwm_pin_az = 35;          // PWM speed control (BLUE)
    static constexpr int _ccw_pin_az = 36;          // Direction control (WHITE)
    static constexpr int _pwm_pin_el = 40;
    static constexpr int _ccw_pin_el = 41;

    static constexpr int FREQ = 20000;              // 20 kHz PWM frequency
    static constexpr int MAX_AZ_SPEED = 0;
    static constexpr int MAX_EL_SPEED = 0;

    static constexpr int _numAvg = 10;              // Sensor averaging samples
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 5;


    // Wind tracking constants
    static constexpr unsigned long MANUAL_SETPOINT_TIMEOUT = 60000;      // 1 minute timeout for manual commands
    static constexpr unsigned long WIND_TRACKING_UPDATE_INTERVAL = 10000; // 10 seconds between wind tracking updates

    // Emergency stow motor control constants
    static constexpr int EMERGENCY_STOW_P_AZ = 500;    // High P gain for max torque
    static constexpr int EMERGENCY_STOW_P_EL = 1000;   // High P gain for max torque

    // PID control constants
    static constexpr float PID_DT_DEFAULT = 0.010f;      // Expected 10ms control loop period
    static constexpr float PID_DT_MAX = 0.1f;            // Clamp dt to 100ms max (safety)
    static constexpr float PID_INTEGRAL_LIMIT = 1000.0f;  // Anti-windup clamp
    static constexpr float PID_D_FILTER_ALPHA = 0.05f;   // EMA smoothing for D term (0-1, lower = more smoothing)

    // PID output ramp — exponential acceleration limit to protect gears/motors
    static constexpr float PID_RAMP_TAU = 0.1f;           // Time constant (seconds) — 63% in 100ms
    static constexpr float PID_RAMP_TAU_STOW = 0.04f;     // Faster ramp during emergency stow

    // Kalman filter constants
    static constexpr float KALMAN_JUMP_THRESHOLD = 5.0f;       // ° — large innovation resets filter
    static constexpr unsigned long KALMAN_VEL_DECAY_MS = 2000;  // ms — decay velocity if no updates
    static constexpr float KALMAN_VEL_DECAY_TAU = 0.5f;        // seconds — velocity decay time constant
    static constexpr float KALMAN_SYNTHETIC_R_MULT = 50.0f;    // synthetic measurement R = R * this

    // EMA angle filter constants
    static constexpr float ANGLE_EMA_ALPHA = 0.3f;            // EMA weight for new sample
    static constexpr float ANGLE_JUMP_THRESHOLD = 5.0f;       // degrees — reject readings beyond this

    // Direction lock constants
    static constexpr float DIR_LOCK_JUMP_THRESHOLD = 10.0f;   // deg — clear direction on jumps
    static constexpr unsigned long DIR_LOCK_STALE_MS = 5000;   // clear after 5s no updates
    static constexpr float DIR_LOCK_MIN_DELTA = 0.001f;        // min delta to register direction
    static constexpr int DIR_LOCK_CHANGE_CONFIRMS = 3;        // consecutive opposing updates to flip direction
    static constexpr float DIR_LOCK_CONFIDENT_DELTA = 0.5f;   // deg — immediate direction flip threshold

    // Wind stow constants
    static constexpr unsigned long WIND_STOW_UPDATE_INTERVAL = 5000; // 5 seconds

    // =========================================================================
    // Per-axis structs
    // =========================================================================

    // Immutable hardware + tuning references for one axis
    struct AxisConfig {
        int pwmPin;
        int dirPin;
        int hallI2cAddr;
        int* P;                      // &P_az or &P_el
        float* I;                    // &I_az or &I_el
        float* D;                    // &D_az or &D_el
        std::atomic<int>* minSpeed;  // &MIN_AZ_SPEED or &MIN_EL_SPEED
        float* tolerance;            // &_MIN_AZ_TOLERANCE or &_MIN_EL_TOLERANCE
        int emergencyP;
        bool isAzimuth;              // controls wraparound + acceleration prediction
        bool invertDir;              // true = reverse motor direction (for reversed gearboxes)
    };

    // All mutable per-axis state
    struct AxisState {
        // Motor output
        int currentSpeed = 0;
        double prevError = 0;
        int maxAdjustedSpeed = 0;

        // PID state
        double errorIntegral = 0.0;
        double prevRawError = 0.0;
        double filteredDTerm = 0.0;
        double prevPidOutput = 0.0;

        // Shared with other tasks (pointers to existing class-level atomics)
        std::atomic<bool>* setPointState = nullptr;
        std::atomic<bool>* isMotorLatched = nullptr;
        std::atomic<bool>* i2cErrorFlag = nullptr;
        volatile float* setpoint = nullptr;
        volatile double* error = nullptr;
        volatile float* correctedAngle = nullptr;

        // Direction lock
        int dirLockDirection = 0;
        bool dirLockHasTracked = false;
        float dirLockPrevSetpoint = 0.0f;
        unsigned long dirLockLastUpdate = 0;
        int dirLockChangeCount = 0;

        // I2C
        uint8_t consecutiveI2cErrors = 0;

        // EMA angle filter state
        float filteredAngleX = 0.0f;  // cos component of EMA
        float filteredAngleY = 0.0f;  // sin component of EMA
        bool angleFilterInitialized = false;

    };

    // Kalman filter state for smooth tracking (constant-velocity model)
    struct KalmanState {
        double pos = 0.0;           // Estimated position (degrees)
        double vel = 0.0;           // Estimated velocity (degrees/sec)
        double p00 = 1.0;           // Covariance: position variance
        double p01 = 0.0;           // Covariance: position-velocity
        double p10 = 0.0;           // Covariance: velocity-position
        double p11 = 1.0;           // Covariance: velocity variance
        unsigned long lastUpdateMs = 0;
        bool initialized = false;
    };

    // Pipeline stage results
    struct DirectionResult { bool shouldStop; };
    struct SpeedResult { int targetSpeed; bool earlyReturn; };

    // Per-axis config and state
    AxisConfig _azCfg, _elCfg;
    AxisState  _az, _el;
    bool _dirLock_initialized = false;           // shared (first-call flag)

    // Smooth tracking state
    std::atomic<bool> _smoothTrackingEnabled{false};
    KalmanState _kalmanAz, _kalmanEl;
    float _kalmanQ = 1.0f;          // Process noise (higher = tracks faster changes)
    float _kalmanR = 1.0f;          // Measurement noise (higher = smoother but more lag)
    std::atomic<int> MIN_SMOOTH_AZ_SPEED{220};
    std::atomic<int> MIN_SMOOTH_EL_SPEED{220};

    // Control parameters (configurable)
    int P_el = 100;
    int P_az = 10;
    float I_el = 0.0f;
    float I_az = 0.0f;
    float D_el = 0.0f;
    float D_az = 25.0f;
    float _MIN_AZ_TOLERANCE = 0.1;
    float _MIN_EL_TOLERANCE = 0.1;
    std::atomic<int> _maxPowerFaultAz = 8;
    std::atomic<int> _maxPowerFaultEl = 7;
    std::atomic<int> _maxPowerFaultTotal = 10;
    std::atomic<int> _minVoltageThreshold = 6;

    // Angle offset parameters
    float _az_offset = 0.0;
    float _el_offset = 0.0;

    // Setpoints and errors (thread-safe)
    volatile float _setpoint_az = 0;
    volatile float _setpoint_el = 0;
    volatile double _error_az = 0;
    volatile double _error_el = 0;
    volatile float _correctedAngle_az = 0;
    volatile float _correctedAngle_el = 0;

    // Update flags
    std::atomic<bool> _setPointAzUpdated = false;
    std::atomic<bool> _setPointElUpdated = false;
    std::atomic<bool> _az_priority = true;

    // Angle and positioning state
    float _az_startAngle = 0;
    float _el_startAngle = 0;
    int _prev_needs_unwind = 0;
    int _quadrantNumber_az = 0;
    int _previousquadrantNumber_az = 0;

    // Wind stow state
    char _windStowReason[64] = "";
    float _windStowDirection = 0.0;
    unsigned long _lastWindStowUpdate = 0;

    // Auto-home state
    std::atomic<unsigned long> _autoHomeTimeoutMs{180000};  // 3 minutes default
    std::atomic<bool> _autoHomeActive{false};

    // Wind tracking state
    std::atomic<unsigned long> _lastManualSetpointTime{0};
    std::atomic<unsigned long> _lastWindTrackingUpdate{0};
    std::atomic<float> _lastWindTrackingDirection{0.0f};

    // PID loop timing
    unsigned long _lastControlLoopTime = 0;

    // Over-power fault timing
    static constexpr unsigned long OVER_POWER_FAULT_MS = 1000;  // must exceed threshold for 1s
    unsigned long _overPowerStartTime = 0;
    int _overPowerFaultValue = 0;            // threshold value at time of detection
    const char* _overPowerFaultLabel = "";    // "AZ"/"EL"/"TOTAL" at time of detection

    // Over-power backoff: reverse each faulting axis by 20° before faulting
    static constexpr float OVER_POWER_BACKOFF_DEG = 20.0f;
    static constexpr unsigned long OVER_POWER_BACKOFF_MS = 10000;  // 10s timeout
    std::atomic<bool> _overPowerBackoff = false;
    unsigned long _overPowerBackoffStart = 0;
    bool _overPowerBackoffAz = false;  // AZ is part of this backoff
    bool _overPowerBackoffEl = false;  // EL is part of this backoff

    // Oscillation detection
    unsigned long _oscillationTimerStart = 0;
    int _oscillationCount = 0;
    bool _oscillationTimerActive = false;

    // Force-stop (brief motor halt before setpoint change)
    std::atomic<unsigned long> _forceStopUntil{0};

    // Calibration state
    std::atomic<int> _calRunTime{0};
    char _calAxis[4] = "";
    int _calState = 0;
    unsigned long _calMoveStartTime = 0;

    // Thread synchronization
    SemaphoreHandle_t _setPointMutex = NULL;
    SemaphoreHandle_t _getAngleMutex = NULL;
    SemaphoreHandle_t _i2cMutex = NULL;  // Shared I2C bus mutex (set via setI2CMutex)
    SemaphoreHandle_t _correctedAngleMutex = NULL;
    SemaphoreHandle_t _errorMutex = NULL;
    SemaphoreHandle_t _el_startAngleMutex = NULL;
    SemaphoreHandle_t _windStowMutex = NULL;
    SemaphoreHandle_t _offsetMutex = NULL;
    SemaphoreHandle_t _calMutex = NULL;

    // =========================================================================
    // Unified motor control pipeline
    // =========================================================================
    void actuateMotor(const AxisConfig& cfg, AxisState& axis, int minSpeed, float dt);
    DirectionResult resolveDirection(const AxisConfig& cfg, AxisState& axis, double error);
    SpeedResult calculateTargetSpeed(const AxisConfig& cfg, AxisState& axis, double error, int minSpeed);
    void applyMotorOutput(const AxisConfig& cfg, AxisState& axis, int targetSpeed,
                          double error, int minSpeed);

    // Unified latching logic
    void updateLatch(const AxisConfig& cfg, AxisState& axis,
                     bool setPointUpdated);

    // Per-axis helpers
    double getAxisError(const AxisState& axis);
    void updateDirectionLockAxis(AxisState& axis, float setpoint,
                                  bool setPointUpdated, bool hasWraparound);

    // Motor control methods
    void setPWM(int pin, int pwm_value);
    void updateMotorControl(float current_setpoint_az, float current_setpoint_el,
                           bool setPointAzUpdated, bool setPointElUpdated);
    void updateMotorPriority(bool setPointAzUpdated, bool setPointElUpdated);

    // Wind safety methods
    void updateWindStowStatus();
    void setWindStowActive(bool active, const char* reason, float direction);
    bool shouldBlockMovement();

    // Auto-home methods
    void checkAutoHome();

    // Wind tracking methods
    void updateWindTrackingStatus();
    void performWindTracking();
    bool shouldActivateWindTracking();
    void setWindTrackingActive(bool active);

    // Internal setpoint methods (bypass manual command tracking)
    void setSetPointAzInternal(float setpoint_az);
    void setSetPointElInternal(float setpoint_el);

    // Direction lock internal method
    void updateDirectionLock(float setpoint_az, float setpoint_el,
                             bool setPointAzUpdated, bool setPointElUpdated);

    // Angle calculation methods
    void angle_shortest_error_az(float target_angle, float current_angle);
    void angle_error_el(float target_angle, float current_angle);
    float correctAngle(float startAngle, float inputAngle);
    void calcIfNeedsUnwind(float correctedAngle_az);

    // Sensor interface methods
    float readFilteredAngle(int i2c_addr, AxisState& axis);
    float ReadRawAngle(int i2c_addr);
    int checkMagnetPresence(int i2c_addr);


    // Utility methods
    void slowPrint(const String& message, int messageID);

    // Smooth tracking methods (Kalman filter)
    void kalmanPredict(KalmanState& kf, float dt, bool isAzimuth);
    void kalmanUpdate(KalmanState& kf, float measurement, bool isAzimuth, float rOverride = -1.0f);
    void kalmanClampBounds(KalmanState& kf, bool isAzimuth);
    void resetSmoothTracking();

    // Helper methods for offset-adjusted start angles
    float getAdjustedAzStartAngle();
    float getAdjustedElStartAngle();

};

#endif
