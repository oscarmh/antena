/*
 * Firmware for the discovery-drive satellite dish rotator.
 * Logger - Enable logging levels and logging to the web UI and over serial.
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

#include "logger.h"

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

Logger::Logger(Preferences& prefs) : _preferences(prefs) {
}

void Logger::begin() {
    // Create mutex for thread-safe log access
    _logMessagesMutex = xSemaphoreCreateMutex();
    
    // Load saved debug level from preferences
    int savedDebugLevel = _preferences.getInt("debugLevel", 1);
    _currentDebugLevel = savedDebugLevel;
    
    // Load saved serial output setting from preferences (set directly, don't use setter during init)
    bool savedSerialOutputDisabled = _preferences.getBool("serialDisabled", false);
    _serialOutputDisabled = savedSerialOutputDisabled;
    
    info("Logger initialized with debug level: " + String(savedDebugLevel) + 
         ", serial output " + String(savedSerialOutputDisabled ? "disabled" : "enabled"));
}

// =============================================================================
// CORE FUNCTIONALITY
// =============================================================================

void Logger::logMessage(LogLevel level, const String& message) {
    // Check if message should be logged based on current debug level
    if (level > _currentDebugLevel) {
        return;
    }
    
    String levelStr = getLevelString(level);
    String fullMessage = levelStr + message;
    
    // Output to Serial console only if not disabled
    if (!_serialOutputDisabled) {
        Serial.println(fullMessage);
    }
    
    // Add to web log buffer for web interface
    addToWebLog(fullMessage);
}

// =============================================================================
// DEBUG LEVEL MANAGEMENT
// =============================================================================

void Logger::setDebugLevel(int level) {
    if (level >= LOG_NONE && level <= LOG_VERBOSE) {
        _currentDebugLevel = level;
        _preferences.putInt("debugLevel", level);
        info("Debug level changed to: " + String(level));
    }
}

int Logger::getDebugLevel() {
    return _currentDebugLevel;
}

// =============================================================================
// SERIAL OUTPUT CONTROL
// =============================================================================

void Logger::setSerialOutputDisabled(bool disabled) {
    _serialOutputDisabled = disabled;
    _preferences.putBool("serialDisabled", disabled);
    info("Serial output " + String(disabled ? "disabled" : "enabled"));
}

bool Logger::getSerialOutputDisabled() {
    return _serialOutputDisabled;
}

// =============================================================================
// WEB INTERFACE METHODS
// =============================================================================

String Logger::getNewLogMessages() {
    String result;
    if (_logMessagesMutex != NULL && xSemaphoreTake(_logMessagesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (_logWritePos > 0 || _logWrapped) {
            if (_logWrapped) {
                // Buffer has wrapped: old data from _logWritePos..end, then 0.._logWritePos
                size_t tailLen = LOG_BUFFER_SIZE - _logWritePos;
                result.reserve(LOG_BUFFER_SIZE);
                result.concat(_logBuffer + _logWritePos, tailLen);
                result.concat(_logBuffer, _logWritePos);
            } else {
                // Buffer hasn't wrapped: data from 0.._logWritePos
                result.concat(_logBuffer, _logWritePos);
            }
            _logWritePos = 0;
            _logWrapped = false;
        }
        xSemaphoreGive(_logMessagesMutex);
    }
    return result;
}

void Logger::addToWebLog(const String& message) {
    if (_logMessagesMutex == NULL) {
        return;
    }

    if (xSemaphoreTake(_logMessagesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Format: "[millis] message\n"
        char prefix[16];
        int prefixLen = snprintf(prefix, sizeof(prefix), "[%lu] ", millis());

        // Write prefix + message + newline into circular buffer
        const char* parts[] = { prefix, message.c_str(), "\n" };
        size_t lengths[] = { (size_t)prefixLen, message.length(), 1 };

        for (int p = 0; p < 3; p++) {
            const char* src = parts[p];
            size_t len = lengths[p];
            for (size_t i = 0; i < len; i++) {
                _logBuffer[_logWritePos] = src[i];
                _logWritePos++;
                if (_logWritePos >= LOG_BUFFER_SIZE) {
                    _logWritePos = 0;
                    _logWrapped = true;
                }
            }
        }

        xSemaphoreGive(_logMessagesMutex);
    }
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

String Logger::getLevelString(LogLevel level) {
    switch (level) {
        case LOG_ERROR:   return "[ERROR] ";
        case LOG_WARN:    return "[WARN]  ";
        case LOG_INFO:    return "[INFO]  ";
        case LOG_DEBUG:   return "[DEBUG] ";
        case LOG_VERBOSE: return "[VERB]  ";
        default:          return "[LOG]   ";
    }
}