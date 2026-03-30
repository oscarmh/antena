/*
 * Firmware for the discovery-drive satellite dish rotator.
 * Weather Poller - Poll the WeatherAPI.com API for wind data and forecasts.
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

#include "weather_poller.h"

// Print adapter that writes to a pre-allocated char buffer (zero heap cost).
// Used with http.writeToStream() which handles chunked encoding, etc.
class BufferPrint : public Stream {
    char* _buf;
    size_t _size;
    size_t _pos;
public:
    BufferPrint(char* buf, size_t size) : _buf(buf), _size(size), _pos(0) {}
    size_t write(uint8_t c) override {
        if (_pos < _size - 1) { _buf[_pos++] = c; return 1; }
        return 0;
    }
    size_t write(const uint8_t* buf, size_t len) override {
        size_t room = _size - 1 - _pos;
        size_t n = (len < room) ? len : room;
        memcpy(_buf + _pos, buf, n);
        _pos += n;
        return n;
    }
    size_t length() const { return _pos; }
    void terminate() { _buf[_pos] = '\0'; }
    // Stream interface stubs (write-only, never read from)
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
};

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

WeatherPoller::WeatherPoller(Preferences& prefs, Logger& logger)
    : _preferences(prefs), _logger(logger) {
}

void WeatherPoller::begin() {
    // Create mutexes for thread-safe access
    _weatherDataMutex = xSemaphoreCreateMutex();
    _apiKeyMutex = xSemaphoreCreateMutex();
    _windSafetyMutex = xSemaphoreCreateMutex();

    // Load saved configuration
    _latitude = _preferences.getFloat("weather_lat", 0.0);
    _longitude = _preferences.getFloat("weather_lon", 0.0);
    _pollingEnabled = _preferences.getBool("weather_enabled", false);
    String savedKey = _preferences.getString("weather_api_key", "");
    safeCopy(_apiKey, savedKey.c_str(), sizeof(_apiKey));
    
    // Load sun/moon display preference
    _showSunMoon = _preferences.getBool("showSunMoon", true);

    // Load unit display preference
    _useImperial = _preferences.getBool("useImperial", false);

    // Load wind safety configuration
    _windSafetyEnabled = _preferences.getBool("wind_safety_en", false);
    _windSpeedThreshold = _preferences.getFloat("wind_speed_thr", 50.0);
    _windGustThreshold = _preferences.getFloat("wind_gust_thr", 60.0);
    _windBasedHomeEnabled = _preferences.getBool("wind_based_home", false);
    
    // Initialize weather data
    clearWeatherData();
    
    String configStatus = "Weather poller initialized - ";
    if (isFullyConfigured()) {
        configStatus += "Fully configured (Location: " + String(_latitude.load(), 6) + ", " + 
                       String(_longitude.load(), 6) + ", API key: SET)";
        
        // Force immediate update on first boot if fully configured
        if (_pollingEnabled) {
            _forceUpdate = true;
            _logger.info("Weather system configured - will fetch data immediately");
        }
    } else if (isLocationConfigured() && !isApiKeyConfigured()) {
        configStatus += "Location set but API key missing";
    } else if (!isLocationConfigured() && isApiKeyConfigured()) {
        configStatus += "API key set but location missing";
    } else {
        configStatus += "Not configured (missing location and API key)";
    }
    
    _logger.info(configStatus);
    
    // Log wind safety configuration
    if (_windSafetyEnabled) {
        _logger.info("Wind safety enabled - Speed threshold: " + String(_windSpeedThreshold.load(), 1) + 
                    " km/h, Gust threshold: " + String(_windGustThreshold.load(), 1) + " km/h");
    }
}

// =============================================================================
// CORE FUNCTIONALITY
// =============================================================================

void WeatherPoller::runWeatherLoop(bool wifiConnected) {
    // Check if we should poll weather data
    if (!shouldPollWeather()) {
        return;
    }
    
    if (!wifiConnected) {
        setErrorState("WiFi disconnected");
        return;
    }
    
    if (!isFullyConfigured()) {
        if (!isLocationConfigured()) {
            setErrorState("Location not configured");
        } else if (!isApiKeyConfigured()) {
            setErrorState("API key not configured");
        } else {
            setErrorState("Configuration incomplete");
        }
        return;
    }
    
    // Attempt to poll weather data
    _lastPollTime = millis();
    
    if (pollWeatherData()) {
        _lastSuccessTime = millis();
        updateWindSafetyStatus(); // Update wind safety after successful poll
        _logger.info("Weather data updated successfully (free heap: " + String(ESP.getFreeHeap()) + ")");
    } else {
        _logger.warn("Failed to update weather data (free heap: " + String(ESP.getFreeHeap()) + ")");
    }
    
    _forceUpdate = false;
}

bool WeatherPoller::shouldPollWeather() {
    if (!_pollingEnabled) {
        return false;
    }
    
    // Force update requested
    if (_forceUpdate) {
        return true;
    }
    
    unsigned long currentTime = millis();
    unsigned long timeSinceLastPoll = currentTime - _lastPollTime;
    unsigned long timeSinceLastSuccess = currentTime - _lastSuccessTime;
    
    // First boot scenario - poll immediately if we've never successfully updated
    if (_lastSuccessTime == 0 && timeSinceLastPoll > 5000) { // Wait 5 seconds after boot for system to settle
        _logger.debug("First weather poll attempt after boot");
        return true;
    }
    
    // Regular interval polling
    if (timeSinceLastPoll >= POLL_INTERVAL_MS) {
        return true;
    }
    
    // Retry on error after shorter interval (even shorter after truncation)
    unsigned long retryInterval = _lastPollWasTruncation.load() ? TRUNCATION_RETRY_MS : RETRY_INTERVAL_MS;
    if (timeSinceLastSuccess >= POLL_INTERVAL_MS && timeSinceLastPoll >= retryInterval) {
        return true;
    }
    
    return false;
}

bool WeatherPoller::pollWeatherData() {
    const char* apiUrl = buildApiUrl();
    if (apiUrl == nullptr) {
        setErrorState("Failed to build API URL");
        return false;
    }

    _logger.debug("Polling weather API: " + String(apiUrl));

    bool success = false;

    // Use a fresh WiFiClient each request to avoid stale-socket errors (-1).
    // Stack-local client is cleaned up automatically when this function returns.
    WiFiClient client;
    client.setTimeout(HTTP_READ_TIMEOUT_MS / 1000);

    HTTPClient http;
    http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(HTTP_READ_TIMEOUT_MS);

    if (!http.begin(client, apiUrl)) {
        setErrorState("HTTP begin failed");
        return false;
    }

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
        // Read response into pre-allocated char buffer (BSS, zero heap cost).
        // writeToStream handles chunked encoding; BufferPrint writes to our buffer.
        BufferPrint bp(_responseBuf, RESPONSE_BUF_SIZE);
        http.writeToStream(&bp);
        bp.terminate();
        size_t totalRead = bp.length();

        // Done reading — free HTTP resources before JSON parsing
        http.end();
        client.stop();

        _logger.debug("Weather API response: " + String(totalRead) + " bytes (free heap: " + String(ESP.getFreeHeap()) + ")");

        if (totalRead == 0) {
            _lastPollWasTruncation = true;
            setErrorState("Empty response from API");
            return false;
        }

        DynamicJsonDocument doc(8192);
        DeserializationError error = deserializeJson(doc, _responseBuf);

        if (error) {
            char errBuf[128];
            if (error == DeserializationError::IncompleteInput ||
                error == DeserializationError::EmptyInput) {
                _lastPollWasTruncation = true;
                snprintf(errBuf, sizeof(errBuf), "JSON parse error: %s (truncated response)", error.c_str());
            } else {
                _lastPollWasTruncation = false;
                snprintf(errBuf, sizeof(errBuf), "JSON parse error: %s", error.c_str());
            }
            setErrorState(errBuf);
        } else {
            _lastPollWasTruncation = false;

            if (doc.containsKey("error")) {
                char errBuf[128];
                const char* apiError = doc["error"]["message"].as<const char*>();
                snprintf(errBuf, sizeof(errBuf), "API error: %s", apiError ? apiError : "unknown");
                setErrorState(errBuf);
            } else {
                bool currentOk = extractCurrentWeather(doc);
                bool forecastOk = extractForecastWeather(doc);

                if (currentOk || forecastOk) {
                    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        _weatherData.dataValid = true;
                        _weatherData.errorMessage[0] = '\0';
                        xSemaphoreGive(_weatherDataMutex);
                    }
                    success = true;
                } else {
                    setErrorState("Failed to extract weather data");
                }
            }
        }
    } else {
        http.end();
        client.stop();

        if (httpResponseCode == 401) {
            setErrorState("Invalid API key");
            _logger.error("WeatherAPI authentication failed - check API key");
        } else if (httpResponseCode == 403) {
            setErrorState("API key quota exceeded");
            _logger.error("WeatherAPI quota exceeded");
        } else if (httpResponseCode > 0) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "HTTP error: %d", httpResponseCode);
            setErrorState(errBuf);
            _logger.error("WeatherAPI HTTP error: " + String(httpResponseCode));
        } else {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "Network error: %d", httpResponseCode);
            setErrorState(errBuf);
            _logger.error("WeatherAPI network error: " + String(httpResponseCode));
        }
    }

    return success;
}


// =============================================================================
// WIND SAFETY METHODS
// =============================================================================

void WeatherPoller::updateWindSafetyStatus() {
    if (!_windSafetyEnabled) {
        setEmergencyStowState(false, "");
        return;
    }
    
    if (!isDataValid()) {
        _logger.warn("Cannot update wind safety - no valid weather data");
        return;
    }
    
    bool currentConditionsTriggered = checkCurrentWindConditions();
    bool forecastConditionsTriggered = checkForecastWindConditions();
    
    if (currentConditionsTriggered || forecastConditionsTriggered) {
        const char* reason;
        if (currentConditionsTriggered && forecastConditionsTriggered) {
            reason = "Current and forecast wind conditions exceed thresholds";
        } else if (currentConditionsTriggered) {
            reason = "Current wind conditions exceed thresholds";
        } else {
            reason = "Forecast wind conditions exceed thresholds";
        }

        setEmergencyStowState(true, reason);
    } else {
        // SIMPLIFIED: No hysteresis - deactivate immediately when conditions improve
        setEmergencyStowState(false, "");
    }
}


bool WeatherPoller::checkCurrentWindConditions() {
    WeatherData data = getWeatherData();
    
    float speedThreshold = _windSpeedThreshold.load();
    float gustThreshold = _windGustThreshold.load();
    
    bool speedExceeded = data.currentWindSpeed > speedThreshold;
    bool gustExceeded = data.currentWindGust > gustThreshold;
    
    if (speedExceeded || gustExceeded) {
        _logger.info("Current wind conditions exceed thresholds - Speed: " + 
                    String(data.currentWindSpeed, 1) + " km/h (limit: " + String(speedThreshold, 1) + 
                    "), Gust: " + String(data.currentWindGust, 1) + " km/h (limit: " + String(gustThreshold, 1) + ")");
        return true;
    }
    
    return false;
}

bool WeatherPoller::checkForecastWindConditions() {
    WeatherData data = getWeatherData();
    
    float speedThreshold = _windSpeedThreshold.load();
    float gustThreshold = _windGustThreshold.load();
    
    // Check next hour forecast (index 0)
    if (data.forecastWindSpeed[0] > speedThreshold || data.forecastWindGust[0] > gustThreshold) {
        _logger.info("Next hour forecast exceeds thresholds - Speed: " + 
                    String(data.forecastWindSpeed[0], 1) + " km/h (limit: " + String(speedThreshold, 1) + 
                    "), Gust: " + String(data.forecastWindGust[0], 1) + " km/h (limit: " + String(gustThreshold, 1) + ")");
        return true;
    }
    
    return false;
}

void WeatherPoller::setEmergencyStowState(bool active, const char* reason) {
    // FIXED: Get weather data BEFORE taking _windSafetyMutex to avoid nested locking.
    // This eliminates potential deadlock if another thread acquires these mutexes
    // in reverse order (_weatherDataMutex -> _windSafetyMutex).
    float stowDirection = 0.0;
    if (active) {
        WeatherData data = getWeatherData();  // Takes and releases _weatherDataMutex
        stowDirection = calculateOptimalStowDirection(data.currentWindDirection);
    }
    
    if (_windSafetyMutex != NULL && xSemaphoreTake(_windSafetyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool wasActive = _windSafetyData.emergencyStowActive;
        
        _windSafetyData.emergencyStowActive = active;
        safeCopy(_windSafetyData.stowReason, reason, sizeof(_windSafetyData.stowReason));
        
        if (active) {
            _windSafetyData.currentStowDirection = stowDirection;
            
            if (!wasActive) {
                _logger.warn("EMERGENCY WIND STOW ACTIVATED: " + String(reason) +
                           " - Stow direction: " + String(stowDirection, 1) + "°");
            }
        } else {
            if (wasActive) {
                _logger.info("Emergency wind stow deactivated - conditions have improved");
            }
            _windSafetyData.currentStowDirection = 0.0;
        }
        
        xSemaphoreGive(_windSafetyMutex);
    }
}


float WeatherPoller::calculateOptimalStowDirection(float windDirection) {
    // Position dish edge-on to wind for minimum wind load
    // Choose the +90° or -90° option that requires less movement from current position
    
    float option1 = normalizeAngle(windDirection + 90.0);
    float option2 = normalizeAngle(windDirection - 90.0);
    
    // For now, default to +90° (can be improved to consider current dish position)
    return option1;
}

float WeatherPoller::getWindBasedHomePosition() {
    if (!_windBasedHomeEnabled || !isDataValid()) {
        return 0.0; // Default home position
    }
    
    WeatherData data = getWeatherData();
    return calculateOptimalStowDirection(data.currentWindDirection);
}

WindSafetyData WeatherPoller::getWindSafetyData() {
    WindSafetyData data;
    
    if (_windSafetyMutex != NULL && xSemaphoreTake(_windSafetyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = _windSafetyData;
        xSemaphoreGive(_windSafetyMutex);
    }
    
    return data;
}

bool WeatherPoller::shouldActivateEmergencyStow() {
    WindSafetyData safetyData = getWindSafetyData();
    return safetyData.emergencyStowActive;
}

// =============================================================================
// WIND SAFETY CONFIGURATION METHODS
// =============================================================================

void WeatherPoller::setWindSafetyEnabled(bool enabled) {
    _windSafetyEnabled = enabled;
    _preferences.putBool("wind_safety_en", enabled);
    _logger.info("Wind safety " + String(enabled ? "enabled" : "disabled"));
    
    if (!enabled) {
        setEmergencyStowState(false, "");
    }
}

bool WeatherPoller::isWindSafetyEnabled() {
    return _windSafetyEnabled.load();
}

void WeatherPoller::setWindSpeedThreshold(float threshold) {
    if (threshold > 0 && threshold <= 200) {
        _windSpeedThreshold = threshold;
        _preferences.putFloat("wind_speed_thr", threshold);
        _logger.info("Wind speed threshold set to: " + String(threshold, 1) + " km/h");
    }
}

float WeatherPoller::getWindSpeedThreshold() {
    return _windSpeedThreshold.load();
}

void WeatherPoller::setWindGustThreshold(float threshold) {
    if (threshold > 0 && threshold <= 200) {
        _windGustThreshold = threshold;
        _preferences.putFloat("wind_gust_thr", threshold);
        _logger.info("Wind gust threshold set to: " + String(threshold, 1) + " km/h");
    }
}

float WeatherPoller::getWindGustThreshold() {
    return _windGustThreshold.load();
}

void WeatherPoller::setWindBasedHomeEnabled(bool enabled) {
    _windBasedHomeEnabled = enabled;
    _preferences.putBool("wind_based_home", enabled);
    _logger.info("Wind-based home positioning " + String(enabled ? "enabled" : "disabled"));
}

bool WeatherPoller::isWindBasedHomeEnabled() {
    return _windBasedHomeEnabled.load();
}

// =============================================================================
// DATA PROCESSING HELPERS
// =============================================================================

bool WeatherPoller::extractCurrentWeather(JsonDocument& doc) {
    if (!doc.containsKey("current")) {
        _logger.warn("No current weather data in WeatherAPI response");
        return false;
    }
    
    JsonObject current = doc["current"];
    
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Extract current weather values with validation
        _weatherData.currentWindSpeed = validateWindSpeed(current["wind_kph"]);
        _weatherData.currentWindDirection = validateWindDirection(current["wind_degree"]);
        _weatherData.currentWindGust = validateWindSpeed(current["gust_kph"]);

        // Extract non-wind current conditions
        // Use .as<T>() instead of | default — the | operator checks is<T>() first,
        // which fails when JSON stores an integer but we expect float (e.g. temp_c: 22 vs 22.0)
        _weatherData.currentTempC = current["temp_c"].as<float>();
        _weatherData.currentPrecipMm = current["precip_mm"].as<float>();
        _weatherData.currentHumidity = current["humidity"].as<int>();
        int condCode = current["condition"]["code"].as<int>();
        _weatherData.currentConditionCode = condCode;
        const char* condText = current["condition"]["text"];
        safeCopy(_weatherData.currentConditionText, condText, sizeof(_weatherData.currentConditionText));
        _weatherData.currentIsThunderstorm = isThunderstormCode(condCode);

        // Handle time strings
        const char* timeStr = current["last_updated"];
        if (timeStr != nullptr) {
            safeCopy(_weatherData.currentTime, timeStr, sizeof(_weatherData.currentTime));
            formatWeatherApiTime(_weatherData.currentTime, _weatherData.lastUpdateTime, sizeof(_weatherData.lastUpdateTime));
        } else {
            safeCopy(_weatherData.currentTime, "Unknown", sizeof(_weatherData.currentTime));
            safeCopy(_weatherData.lastUpdateTime, "Unknown", sizeof(_weatherData.lastUpdateTime));
        }

        xSemaphoreGive(_weatherDataMutex);
        
        _logger.debug("Current wind: " + String(_weatherData.currentWindSpeed, 1) + " km/h, " +
                     "Direction: " + String(_weatherData.currentWindDirection, 0) + "°, " +
                     "Gusts: " + String(_weatherData.currentWindGust, 1) + " km/h");
        return true;
    }
    
    return false;
}

bool WeatherPoller::extractForecastWeather(JsonDocument& doc) {
    if (!doc.containsKey("forecast") || !doc["forecast"].containsKey("forecastday")) {
        _logger.warn("No forecast data in WeatherAPI response");
        return false;
    }
    
    JsonArray forecastDays = doc["forecast"]["forecastday"];
    if (forecastDays.size() == 0) {
        _logger.warn("Empty forecast array");
        return false;
    }
    
    // Get current time from the API response to find our position
    const char* currentTimeStr = "";
    if (doc.containsKey("current") && doc["current"].containsKey("last_updated")) {
        const char* timeStr = doc["current"]["last_updated"];
        if (timeStr != nullptr) {
            currentTimeStr = timeStr;
        }
    }

    // Local structure for parsing outside mutex
    struct ForecastEntry {
        char time[24];
        float windSpeed;
        float windDirection;
        float windGust;
        float tempC;
        float precipMm;
        float snowCm;
        int humidity;
        int conditionCode;
        char conditionText[64];
        bool isThunderstorm;
    };
    ForecastEntry localForecast[3];
    memset(localForecast, 0, sizeof(localForecast));
    int forecastCount = 0;

    int currentHour = getCurrentHourFromTime(currentTimeStr);
    _logger.debug("Current hour: " + String(currentHour));

    // Scan all forecast days (today + tomorrow) for the next 3 future hours
    for (size_t d = 0; d < forecastDays.size() && forecastCount < 3; d++) {
        JsonObject day = forecastDays[d];
        if (!day.containsKey("hour")) continue;
        JsonArray dayHours = day["hour"];

        for (size_t i = 0; i < dayHours.size() && forecastCount < 3; i++) {
            JsonObject hour = dayHours[i];

            const char* hourTimeStr = hour["time"];
            if (hourTimeStr == nullptr) continue;

            // For today (d==0), skip past/current hours; for tomorrow, include all
            if (d == 0) {
                int hourValue = getHourFromTimeString(hourTimeStr);
                if (hourValue <= currentHour) continue;
            }

            safeCopy(localForecast[forecastCount].time, hourTimeStr, sizeof(localForecast[forecastCount].time));
            localForecast[forecastCount].windSpeed = validateWindSpeed(hour["wind_kph"]);
            localForecast[forecastCount].windDirection = validateWindDirection(hour["wind_degree"]);
            localForecast[forecastCount].windGust = validateWindSpeed(hour["gust_kph"]);
            localForecast[forecastCount].tempC = hour["temp_c"].as<float>();
            localForecast[forecastCount].precipMm = hour["precip_mm"].as<float>();
            localForecast[forecastCount].snowCm = hour["snow_cm"].as<float>();
            localForecast[forecastCount].humidity = hour["humidity"].as<int>();
            int fc = hour["condition"]["code"].as<int>();
            localForecast[forecastCount].conditionCode = fc;
            const char* ft = hour["condition"]["text"];
            safeCopy(localForecast[forecastCount].conditionText, ft, sizeof(localForecast[forecastCount].conditionText));
            localForecast[forecastCount].isThunderstorm = isThunderstormCode(fc);

            _logger.debug("Forecast " + String(forecastCount) + ": " +
                         String(localForecast[forecastCount].time) + " - Wind: " +
                         String(localForecast[forecastCount].windSpeed, 1) + " km/h");

            forecastCount++;
        }
    }
    
    // Warn if we couldn't get enough forecast entries
    if (forecastCount < 3) {
        _logger.warn("Only got " + String(forecastCount) + " forecast entries (expected 3)");
    }
    
    // Quick atomic update of shared data
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < 3; i++) {
            if (i < forecastCount) {
                safeCopy(_weatherData.forecastTimes[i], localForecast[i].time, sizeof(_weatherData.forecastTimes[i]));
                _weatherData.forecastWindSpeed[i] = localForecast[i].windSpeed;
                _weatherData.forecastWindDirection[i] = localForecast[i].windDirection;
                _weatherData.forecastWindGust[i] = localForecast[i].windGust;
                _weatherData.forecastTempC[i] = localForecast[i].tempC;
                _weatherData.forecastPrecipMm[i] = localForecast[i].precipMm;
                _weatherData.forecastSnowCm[i] = localForecast[i].snowCm;
                _weatherData.forecastHumidity[i] = localForecast[i].humidity;
                _weatherData.forecastConditionCode[i] = localForecast[i].conditionCode;
                safeCopy(_weatherData.forecastConditionText[i], localForecast[i].conditionText, sizeof(_weatherData.forecastConditionText[i]));
                _weatherData.forecastIsThunderstorm[i] = localForecast[i].isThunderstorm;
            } else {
                _weatherData.forecastTimes[i][0] = '\0';
                _weatherData.forecastWindSpeed[i] = 0.0f;
                _weatherData.forecastWindDirection[i] = 0.0f;
                _weatherData.forecastWindGust[i] = 0.0f;
                _weatherData.forecastTempC[i] = 0.0f;
                _weatherData.forecastPrecipMm[i] = 0.0f;
                _weatherData.forecastSnowCm[i] = 0.0f;
                _weatherData.forecastHumidity[i] = 0;
                _weatherData.forecastConditionCode[i] = 0;
                _weatherData.forecastConditionText[i][0] = '\0';
                _weatherData.forecastIsThunderstorm[i] = false;
            }
        }
        xSemaphoreGive(_weatherDataMutex);
    } else {
        _logger.error("Failed to acquire weather data mutex");
        return false;
    }
    
    _logger.debug("Forecast extracted for next " + String(forecastCount) + " hours");
    return (forecastCount > 0);
}

void WeatherPoller::clearWeatherData() {
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _weatherData.currentWindSpeed = 0.0;
        _weatherData.currentWindGust = 0.0;
        _weatherData.currentWindDirection = 0.0;
        _weatherData.currentTime[0] = '\0';
        safeCopy(_weatherData.lastUpdateTime, "Never", sizeof(_weatherData.lastUpdateTime));
        _weatherData.errorMessage[0] = '\0';
        _weatherData.dataValid = false;

        // Clear current non-wind fields
        _weatherData.currentTempC = 0.0;
        _weatherData.currentPrecipMm = 0.0;
        _weatherData.currentHumidity = 0;
        _weatherData.currentConditionCode = 0;
        _weatherData.currentConditionText[0] = '\0';
        _weatherData.currentIsThunderstorm = false;

        // Clear forecast arrays
        for (int i = 0; i < 3; i++) {
            _weatherData.forecastWindSpeed[i] = 0.0;
            _weatherData.forecastWindGust[i] = 0.0;
            _weatherData.forecastWindDirection[i] = 0.0;
            _weatherData.forecastTimes[i][0] = '\0';
            _weatherData.forecastTempC[i] = 0.0;
            _weatherData.forecastPrecipMm[i] = 0.0;
            _weatherData.forecastSnowCm[i] = 0.0;
            _weatherData.forecastHumidity[i] = 0;
            _weatherData.forecastConditionCode[i] = 0;
            _weatherData.forecastConditionText[i][0] = '\0';
            _weatherData.forecastIsThunderstorm[i] = false;
        }

        xSemaphoreGive(_weatherDataMutex);
    }
}

void WeatherPoller::setErrorState(const char* error) {
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safeCopy(_weatherData.errorMessage, error, sizeof(_weatherData.errorMessage));
        // Keep previous data valid on transient failures so wind safety
        // continues working with stale-but-usable data.
        // Only invalidate if we never had a successful update.
        if (_lastSuccessTime == 0) {
            _weatherData.dataValid = false;
        }
        xSemaphoreGive(_weatherDataMutex);
    }
    _logger.error("Weather polling error: " + String(error));
}

// =============================================================================
// CONFIGURATION METHODS
// =============================================================================

bool WeatherPoller::setLocation(float latitude, float longitude) {
    if (!isValidCoordinate(latitude, longitude)) {
        _logger.error("Invalid coordinates: " + String(latitude, 6) + ", " + String(longitude, 6));
        return false;
    }
    
    _latitude = latitude;
    _longitude = longitude;
    
    // Save to preferences
    _preferences.putFloat("weather_lat", latitude);
    _preferences.putFloat("weather_lon", longitude);
    
    _logger.info("Weather location set to: " + String(latitude, 6) + ", " + String(longitude, 6));
    
    // Clear old data and force update if now fully configured
    if (isFullyConfigured() && _pollingEnabled) {
        clearWeatherData();
        forceUpdate();
    }
    
    return true;
}

bool WeatherPoller::setApiKey(const String& apiKey) {
    String trimmedKey = apiKey;
    trimmedKey.trim();

    if (!isValidApiKey(trimmedKey.c_str())) {
        _logger.error("Invalid API key format");
        return false;
    }

    if (_apiKeyMutex != NULL && xSemaphoreTake(_apiKeyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safeCopy(_apiKey, trimmedKey.c_str(), sizeof(_apiKey));
        xSemaphoreGive(_apiKeyMutex);
    }

    // Save to preferences
    _preferences.putString("weather_api_key", trimmedKey);

    _logger.info("WeatherAPI key configured");

    // Clear old data and force update if now fully configured
    if (isFullyConfigured() && _pollingEnabled) {
        clearWeatherData();
        forceUpdate();
    }

    return true;
}

float WeatherPoller::getLatitude() {
    return _latitude.load();
}

float WeatherPoller::getLongitude() {
    return _longitude.load();
}

const char* WeatherPoller::getApiKey() {
    // Returns pointer to mutex-protected buffer — caller must use immediately or copy
    static char key[65];
    key[0] = '\0';
    if (_apiKeyMutex != NULL && xSemaphoreTake(_apiKeyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safeCopy(key, _apiKey, sizeof(key));
        xSemaphoreGive(_apiKeyMutex);
    }
    return key;
}

bool WeatherPoller::isLocationConfigured() {
    float lat = _latitude.load();
    float lon = _longitude.load();
    return isValidCoordinate(lat, lon);
}

bool WeatherPoller::isApiKeyConfigured() {
    return isValidApiKey(getApiKey());
}

bool WeatherPoller::isFullyConfigured() {
    return isLocationConfigured() && isApiKeyConfigured();
}

// =============================================================================
// DATA ACCESS METHODS
// =============================================================================

WeatherData WeatherPoller::getWeatherData() {
    WeatherData data;
    
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = _weatherData;
        xSemaphoreGive(_weatherDataMutex);
    }
    
    return data;
}

bool WeatherPoller::isDataValid() {
    bool valid = false;
    
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        valid = _weatherData.dataValid;
        xSemaphoreGive(_weatherDataMutex);
    }
    
    return valid;
}

const char* WeatherPoller::getLastError() {
    static char error[128];
    error[0] = '\0';

    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safeCopy(error, _weatherData.errorMessage, sizeof(error));
        xSemaphoreGive(_weatherDataMutex);
    }

    return error;
}

unsigned long WeatherPoller::getLastUpdateTime() {
    return _lastSuccessTime.load();
}

// =============================================================================
// CONTROL METHODS
// =============================================================================

void WeatherPoller::forceUpdate() {
    _forceUpdate = true;
    _logger.debug("Weather update forced");
}

bool WeatherPoller::isPollingEnabled() {
    return _pollingEnabled.load();
}

void WeatherPoller::setPollingEnabled(bool enabled) {
    _pollingEnabled = enabled;
    _preferences.putBool("weather_enabled", enabled);
    _logger.info("Weather polling " + String(enabled ? "enabled" : "disabled"));
    
    if (!enabled) {
        clearWeatherData();
        setEmergencyStowState(false, "");
    }
}

// =============================================================================
// SUN/MOON DISPLAY METHODS
// =============================================================================

void WeatherPoller::setShowSunMoon(bool enabled) {
    _showSunMoon = enabled;
    _preferences.putBool("showSunMoon", enabled);
    _logger.info("Sun/Moon compass display " + String(enabled ? "enabled" : "disabled"));
}

bool WeatherPoller::isShowSunMoon() {
    return _showSunMoon.load();
}

void WeatherPoller::setUseImperial(bool enabled) {
    _useImperial = enabled;
    _preferences.putBool("useImperial", enabled);
    _logger.info("Weather units set to " + String(enabled ? "Imperial" : "Metric"));
}

bool WeatherPoller::isUseImperial() {
    return _useImperial.load();
}

// =============================================================================
// WEATHER CONDITION HELPERS
// =============================================================================

bool WeatherPoller::isThunderstormCode(int code) {
    // WeatherAPI.com condition codes for thunderstorm-related weather
    return (code == 1087 || code == 1273 || code == 1276 || code == 1279 || code == 1282);
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

const char* WeatherPoller::buildApiUrl() {
    if (!isFullyConfigured()) {
        return nullptr;
    }

    const char* apiKey = getApiKey();
    if (apiKey[0] == '\0') {
        return nullptr;
    }

    static char url[256];
    snprintf(url, sizeof(url),
             "http://api.weatherapi.com/v1/forecast.json?key=%s&q=%.6f,%.6f&days=2&aqi=no&alerts=no",
             apiKey, _latitude.load(), _longitude.load());

    return url;
}

float WeatherPoller::validateWindSpeed(float speed) {
    if (isnan(speed) || speed < 0) {
        return 0.0;
    }
    // Cap at reasonable maximum (500 km/h)
    return min(speed, 500.0f);
}

float WeatherPoller::validateWindDirection(float direction) {
    if (isnan(direction)) {
        return 0.0;
    }
    // Normalize to 0-360 range
    while (direction < 0) direction += 360;
    while (direction >= 360) direction -= 360;
    return direction;
}

float WeatherPoller::normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

int WeatherPoller::getCurrentHourFromTime(const char* timeStr) {
    // Parse time string like "2024-01-15 17:30" to extract hour (17)
    if (timeStr == nullptr || timeStr[0] == '\0') {
        return -1;
    }

    const char* space = strchr(timeStr, ' ');
    if (space == nullptr) {
        return -1;
    }

    const char* timePart = space + 1;  // "17:30"
    return atoi(timePart);  // atoi stops at ':'
}

int WeatherPoller::getHourFromTimeString(const char* timeStr) {
    return getCurrentHourFromTime(timeStr);
}

void WeatherPoller::formatWeatherApiTime(const char* apiTime, char* out, size_t outSize) {
    // WeatherAPI returns time like "2024-01-15 14:30" (local time at location)
    if (apiTime == nullptr || apiTime[0] == '\0') {
        safeCopy(out, "Unknown", outSize);
        return;
    }

    const char* space = strchr(apiTime, ' ');
    if (space == nullptr) {
        safeCopy(out, apiTime, outSize);
        return;
    }

    const char* timePart = space + 1;  // "14:30"
    snprintf(out, outSize, "%s (local)", timePart);
}

const char* WeatherPoller::getRelativeUpdateTime() {
    static char buf[24];
    WeatherData data = getWeatherData();
    safeCopy(buf, data.lastUpdateTime, sizeof(buf));
    return buf;
}

bool WeatherPoller::isValidCoordinate(float lat, float lon) {
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 && 
            (lat != 0.0 || lon != 0.0)); // Exclude 0,0 as it's likely unset
}

bool WeatherPoller::isValidApiKey(const char* key) {
    if (key == nullptr) return false;
    size_t len = strlen(key);
    if (len < 16 || len > 64) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        char c = key[i];
        if (!isAlphaNumeric(c) && c != '_' && c != '-') {
            return false;
        }
    }

    return true;
}