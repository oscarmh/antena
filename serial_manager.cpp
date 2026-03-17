/*
 * Firmware for the discovery-drive satellite dish rotator.
 * Serial Manager - Manage serial rotctl connections.
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

// Implements the EasyComm II Serial Protocol

#include "serial_manager.h"
#include "wifi_manager.h"

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

SerialManager::SerialManager(Preferences& prefs, MotorSensorController& motorSensorCtrl, Logger& logger)
    : _preferences(prefs), _motorSensorCtrl(motorSensorCtrl), _logger(logger) {
}

void SerialManager::begin() {
    _inputString.reserve(64);  // Reserve space for input string
    _stringComplete = false;
    _logger.info("SerialManager initialized");
}

// =============================================================================
// CORE FUNCTIONALITY
// =============================================================================

void SerialManager::runSerialLoop() {
    readSerialInput();
    
    if (_stringComplete) {
        processCommand();
        resetInputBuffer();
    }
    
    updateSerialActivityStatus();
}

void SerialManager::readSerialInput() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (_inputString.length() < 63) {  // Prevent overflow
            _inputString += inChar;
        }
        
        if (inChar == '\n' || inChar == '\r') {
            _stringComplete = true;
            break;
        }
    }
}

void SerialManager::processCommand() {
    _inputString.trim();
    
    if (_inputString.length() == 0) {
        return;
    }

    // Process different command types
    if (processPositionQueries()) return;
    if (processPositionCommands()) return;
    if (processSetPositionCommands()) return;
    if (processCalibrationCommands()) return;
    if (processWiFiCommands(_inputString)) return;
    if (processSystemCommands()) return;
    
    // If no command matched, log unknown command
    _logger.warn("Unknown serial command: " + _inputString);
}

// =============================================================================
// COMMAND PROCESSING METHODS
// =============================================================================

bool SerialManager::processPositionQueries() {
    if (_inputString == "AZ EL") {
        Serial.print("AZ");
        Serial.print(_motorSensorCtrl.getCorrectedAngleAz());
        Serial.print(" EL");
        Serial.println(_motorSensorCtrl.getCorrectedAngleEl());
        updateSerialActivity();
        return true;
    }
    
    if (_inputString == "AZ") {
        Serial.print("AZ");
        Serial.println(_motorSensorCtrl.getCorrectedAngleAz());
        updateSerialActivity();
        return true;
    }
    
    if (_inputString == "EL") {
        Serial.print("EL");
        Serial.println(_motorSensorCtrl.getCorrectedAngleEl());
        updateSerialActivity();
        return true;
    }
    
    if (_inputString.startsWith("STATUS")) {
        printStatusInfo();
        updateSerialActivity();
        return true;
    }
    
    return false;
}

bool SerialManager::processPositionCommands() {
    if ((_inputString.startsWith("AZ") && _inputString.length() > 2) || (_inputString.startsWith("EL") && _inputString.length() > 2)) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        parseAndSetPosition();
        return true;
    }

    if (_inputString.startsWith("HOME")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.setSetPointAz(0);
        _motorSensorCtrl.setSetPointEl(0);
        return true;
    }

    return false;
}

bool SerialManager::processSetPositionCommands() {
    if (_inputString.startsWith("SA SE")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.setSetPointAz(_motorSensorCtrl.getCorrectedAngleAz());
        _motorSensorCtrl.setSetPointEl(_motorSensorCtrl.getCorrectedAngleEl());
        return true;
    }

    if (_inputString.startsWith("SA")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.setSetPointAz(_motorSensorCtrl.getCorrectedAngleAz());
        return true;
    }

    if (_inputString.startsWith("SE")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.setSetPointEl(_motorSensorCtrl.getCorrectedAngleEl());
        return true;
    }

    return false;
}

bool SerialManager::processCalibrationCommands() {
    if (_inputString.startsWith("MV_EL")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.calMoveMotor(_inputString.substring(5), "EL");
        return true;
    }

    if (_inputString.startsWith("MV_AZ")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.calMoveMotor(_inputString.substring(5), "AZ");
        return true;
    }

    if (_inputString.startsWith("CAL_ON")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _logger.info("CAL MODE ON");
        _motorSensorCtrl.calMode = true;
        return true;
    }

    if (_inputString.startsWith("CAL_OFF")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _logger.info("CAL MODE OFF");
        _motorSensorCtrl.calMode = false;
        return true;
    }

    if (_inputString.startsWith("CAL_EL")) {
        updateSerialActivity();
        if (_motorSensorCtrl.isSafeMode()) {
            Serial.println("SAFE MODE - command blocked");
            return true;
        }
        _motorSensorCtrl.calibrate_elevation();
        return true;
    }

    return false;
}

void SerialManager::setWiFiManager(WiFiManager* wm) {
    _wifiManager = wm;
}

bool SerialManager::processWiFiCommands(const String& input) {
    if (!_wifiManager) return false;

    if (input.startsWith("WIFI_OFF_P")) {
        _wifiManager->disableWiFi(true);
        Serial.println("WiFi permanently disabled. Use WIFI_ON_P to re-enable.");
        updateSerialActivity();
        return true;
    }
    if (input.startsWith("WIFI_ON_P")) {
        _wifiManager->enableWiFi(true);
        Serial.println("WiFi permanently enabled.");
        updateSerialActivity();
        return true;
    }
    if (input.startsWith("WIFI_OFF")) {
        _wifiManager->disableWiFi(false);
        Serial.println("WiFi disabled until reboot. Use WIFI_ON to re-enable.");
        updateSerialActivity();
        return true;
    }
    if (input.startsWith("WIFI_ON")) {
        _wifiManager->enableWiFi(false);
        Serial.println("WiFi enabled.");
        updateSerialActivity();
        return true;
    }
    return false;
}

bool SerialManager::processSystemCommands() {
    if (_inputString.startsWith("RESET_WEB_PW")) {
        _preferences.putString("loginUser", "");
        _preferences.putString("loginPassword", "");
        _logger.info("Web Interface Password Reset!");
        updateSerialActivity();
        return true;
    }
    
    if (_inputString.startsWith("PLAY_ODE")) {
        _motorSensorCtrl.playOdeToJoy();
        updateSerialActivity();
        return true;
    }
    
    return false;
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

void SerialManager::parseAndSetPosition() {
    int elIndex = _inputString.indexOf("EL");
    int azIndex = _inputString.indexOf("AZ");
    
    if (azIndex != -1) {
        // Extract AZ value — ends at "EL" if present, otherwise end of string
        String az_str = (elIndex != -1) ? _inputString.substring(azIndex + 2, elIndex) 
                                        : _inputString.substring(azIndex + 2);
        az_str.trim();
        float az = validateAndCleanAzimuth(az_str.toFloat());
        _motorSensorCtrl.setSetPointAz(az);
        _logger.info("Serial AZ command - Az: " + String(az, 2) + "°");
    }
    
    if (elIndex != -1) {
        String el_str = _inputString.substring(elIndex + 2);
        el_str.trim();
        float el = validateAndCleanElevation(el_str.toFloat());
        _motorSensorCtrl.setSetPointEl(el);
        _logger.info("Serial EL command - El: " + String(el, 2) + "°");
    }
}

float SerialManager::validateAndCleanAzimuth(float az) {
    if (isnan(az)) az = 0;
    
    // Normalize azimuth to 0-360 range
    az = fmod(az, 360.0);
    if (az < 0) {
        az += 360.0;
    }
    
    return az;
}

float SerialManager::validateAndCleanElevation(float el) {
    if (isnan(el)) el = 0;

    // Clamp elevation respecting extended elevation mode
    float minEl = _motorSensorCtrl.isExtendedElEnabled() ? -90.0f : 0.0f;
    if (el < minEl) el = minEl;
    if (el > 90) el = 90;

    return el;
}

void SerialManager::printStatusInfo() {
    Serial.println("===============================================");
    Serial.println("=== DISCOVERY DISH ROTATOR STATUS ===");
    Serial.println("===============================================");
    
    // === CURRENT POSITION & SETPOINTS ===
    Serial.println("--- Current Position & Setpoints ---");
    Serial.println("Corrected Angle Elevation: " + String(_motorSensorCtrl.getCorrectedAngleEl(), 2) + "°");
    Serial.println("Corrected Angle Azimuth: " + String(_motorSensorCtrl.getCorrectedAngleAz(), 2) + "°");
    Serial.println("Azimuth Setpoint: " + String(_motorSensorCtrl.getSetPointAz(), 2) + "°");
    Serial.println("Elevation Setpoint: " + String(_motorSensorCtrl.getSetPointEl(), 2) + "°");
    Serial.println("Azimuth State: " + String(_motorSensorCtrl.setPointState_az ? "ACTIVE" : "STOPPED"));
    Serial.println("Elevation State: " + String(_motorSensorCtrl.setPointState_el ? "ACTIVE" : "STOPPED"));
    Serial.println("Azimuth Error: " + String(_motorSensorCtrl.getErrorAz(), 3) + "°");
    Serial.println("Elevation Error: " + String(_motorSensorCtrl.getErrorEl(), 3) + "°");
    Serial.println("Elevation Tare Angle: " + String(_motorSensorCtrl.getElStartAngle(), 2) + "°");
    Serial.println("Needs Unwind: " + String(_motorSensorCtrl.needs_unwind));
    Serial.println("Azimuth Angle Offset: " + String(_motorSensorCtrl.getAzOffset(), 3) + "°");
    Serial.println("Elevation Angle Offset: " + String(_motorSensorCtrl.getElOffset(), 3) + "°");
    
    // === SYSTEM STATUS & ERRORS ===
    Serial.println("--- System Status & Errors ---");
    Serial.println("Cal Mode: " + String(_motorSensorCtrl.calMode ? "ON" : "OFF"));
    Serial.println("Hall AZ Sensor I2C Error: " + String(_motorSensorCtrl.i2cErrorFlag_az ? "TRUE" : "FALSE"));
    Serial.println("Hall EL Sensor I2C Error: " + String(_motorSensorCtrl.i2cErrorFlag_el ? "TRUE" : "FALSE"));
    Serial.println("Bad Angle Error: " + String(_motorSensorCtrl.badAngleFlag ? "TRUE" : "FALSE"));
    Serial.println("Magnet Error: " + String(_motorSensorCtrl.magnetFault ? "TRUE" : "FALSE"));
    Serial.println("Fault Tripped: " + String(_motorSensorCtrl.global_fault ? "TRUE" : "FALSE"));
    Serial.println("AZ Motor Latched: " + String(_motorSensorCtrl._isAzMotorLatched ? "TRUE" : "FALSE"));
    Serial.println("EL Motor Latched: " + String(_motorSensorCtrl._isElMotorLatched ? "TRUE" : "FALSE"));
    Serial.println("Serial Active: " + String(serialActive ? "TRUE" : "FALSE"));
    
    // === MOTOR CONFIGURATION ===
    Serial.println("--- Motor Configuration ---");
    Serial.println("Tolerance AZ: " + String(_motorSensorCtrl.getMinAzTolerance(), 3) + "°");
    Serial.println("Tolerance EL: " + String(_motorSensorCtrl.getMinElTolerance(), 3) + "°");
    Serial.println("Single Motor Mode: " + String(_motorSensorCtrl.singleMotorMode ? "ON" : "OFF"));
    Serial.println("Max Dual Motor AZ Speed: " + String(_motorSensorCtrl.convertSpeedToPercentage((float)_motorSensorCtrl.max_dual_motor_az_speed, _motorSensorCtrl.MIN_AZ_SPEED)) + "%");
    Serial.println("Max Dual Motor EL Speed: " + String(_motorSensorCtrl.convertSpeedToPercentage((float)_motorSensorCtrl.max_dual_motor_el_speed, _motorSensorCtrl.MIN_EL_SPEED)) + "%");
    Serial.println("Max Single Motor AZ Speed: " + String(_motorSensorCtrl.convertSpeedToPercentage((float)_motorSensorCtrl.max_single_motor_az_speed, _motorSensorCtrl.MIN_AZ_SPEED)) + "%");
    Serial.println("Max Single Motor EL Speed: " + String(_motorSensorCtrl.convertSpeedToPercentage((float)_motorSensorCtrl.max_single_motor_el_speed, _motorSensorCtrl.MIN_EL_SPEED)) + "%");
    
    // === ADVANCED PARAMETERS ===
    Serial.println("--- Advanced Parameters ---");
    Serial.println("P_el (Elevation P-Gain): " + String(_motorSensorCtrl.getPEl()));
    Serial.println("P_az (Azimuth P-Gain): " + String(_motorSensorCtrl.getPAz()));
    Serial.println("MIN_EL_SPEED: " + String(_motorSensorCtrl.getMinElSpeed()));
    Serial.println("MIN_AZ_SPEED: " + String(_motorSensorCtrl.getMinAzSpeed()));
    Serial.println("MIN_AZ_TOLERANCE: " + String(_motorSensorCtrl.getMinAzTolerance(), 3) + "°");
    Serial.println("MIN_EL_TOLERANCE: " + String(_motorSensorCtrl.getMinElTolerance(), 3) + "°");
    Serial.println("MAX_FAULT_POWER_AZ: " + String(_motorSensorCtrl.getMaxPowerFaultAz()) + "W");
    Serial.println("MAX_FAULT_POWER_EL: " + String(_motorSensorCtrl.getMaxPowerFaultEl()) + "W");
    Serial.println("MAX_FAULT_POWER_TOTAL: " + String(_motorSensorCtrl.getMaxPowerFaultTotal()) + "W");
    
    // === NETWORK CONFIGURATION ===
    Serial.println("--- Network Configuration ---");
    if (_wifiManager && _wifiManager->wifiDisabled.load()) {
        Serial.println("WiFi:           DISABLED");
    }
    Serial.println("HTTP Port: " + String(_preferences.getInt("http_port", 80)));
    Serial.println("Rotctl Port: " + String(_preferences.getInt("rotctl_port", 4533)));
    Serial.println("WiFi SSID: " + _preferences.getString("wifi_ssid", "discoverydish_HOTSPOT"));
    
    // === STELLARIUM SETTINGS ===
    Serial.println("--- Stellarium Settings ---");
    Serial.println("Stellarium Polling: " + String(_preferences.getBool("stellariumOn", false) ? "ON" : "OFF"));
    Serial.println("Stellarium Server IP: " + _preferences.getString("stelServIP", "NO IP SET"));
    Serial.println("Stellarium Server Port: " + _preferences.getString("stelServPort", "8090"));
    
    // === AUTHENTICATION ===
    Serial.println("--- Authentication ---");
    Serial.println("Login User: " + _preferences.getString("loginUser", "(none)"));
    String passwordStatus = (_preferences.getString("loginUser", "").length() != 0 && 
                           _preferences.getString("loginPassword", "").length() != 0) ? "SET" : "NOT SET";
    Serial.println("Password Status: " + passwordStatus);
    
    // === LOGGING ===
    Serial.println("--- Logging ---");
    Serial.println("Current Debug Level: " + String(_logger.getDebugLevel()));
    String levelNames[] = {"NONE", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE"};
    int currentLevel = _logger.getDebugLevel();
    if (currentLevel >= 0 && currentLevel <= 5) {
        Serial.println("Debug Level Name: " + String(levelNames[currentLevel]));
    }
    
    Serial.println("===============================================");
}

void SerialManager::updateSerialActivity() {
    _lastSerialActivity = millis();
}

void SerialManager::updateSerialActivityStatus() {
    if (millis() - _lastSerialActivity > _serialActiveTimeout) {
        serialActive = false;
        delay(1000);  // Throttle when inactive
    } else {
        serialActive = true;
    }
}

void SerialManager::resetInputBuffer() {
    _inputString = "";
    _stringComplete = false;
}