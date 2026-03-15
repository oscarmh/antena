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
    _apiKey = _preferences.getString("weather_api_key", "");
    
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
    
    // Retry on error after shorter interval
    if (timeSinceLastSuccess >= POLL_INTERVAL_MS && timeSinceLastPoll >= RETRY_INTERVAL_MS) {
        return true;
    }
    
    return false;
}

bool WeatherPoller::pollWeatherData() {
    String apiUrl = buildApiUrl();
    if (apiUrl.isEmpty()) {
        setErrorState("Failed to build API URL");
        return false;
    }
    
    _logger.debug("Polling weather API: " + apiUrl);
    
    // Fetch response as string, then free TLS before parsing.
    // This sequences memory: peak = max(TLS+string, doc) ≈ 20KB
    // instead of TLS+doc concurrent ≈ 32KB with streaming.
    bool success = false;
    String response;
    int httpResponseCode;
    {
        HTTPClient http;
        if (!http.begin(apiUrl)) {
            setErrorState("HTTP begin failed");
            return false;
        } 

        http.setConnectTimeout(HTTP_TIMEOUT_MS);
        http.setTimeout(HTTP_TIMEOUT_MS);

        httpResponseCode = http.GET();

        if (httpResponseCode == 200) {
            response = http.getString();
        }

        http.end();  // Frees TLS buffers before JSON parsing
    }

    if (httpResponseCode == 200) {
        if (response.length() == 0) {
            setErrorState("Empty response from API");
            return false;
        }

        // Filter: only deserialize the fields we actually use
        StaticJsonDocument<384> filter;
        filter["error"]["message"] = true;
        filter["current"] = true;
        JsonObject forecastHourFilter = filter["forecast"]["forecastday"][0]["hour"][0].to<JsonObject>();
        forecastHourFilter["time"] = true;
        forecastHourFilter["wind_kph"] = true;
        forecastHourFilter["wind_degree"] = true;
        forecastHourFilter["gust_kph"] = true;
        forecastHourFilter["temp_c"] = true;
        forecastHourFilter["precip_mm"] = true;
        forecastHourFilter["snow_cm"] = true;
        forecastHourFilter["humidity"] = true;
        forecastHourFilter["condition"]["text"] = true;
        forecastHourFilter["condition"]["code"] = true;

        DynamicJsonDocument doc(12288);
        DeserializationError error = deserializeJson(doc, response,
                                                     DeserializationOption::Filter(filter));
        response = "";  // Free response string before processing

        if (error) {
            setErrorState("JSON parse error: " + String(error.c_str()));
        } else if (doc.containsKey("error")) {
            String apiError = doc["error"]["message"].as<String>();
            setErrorState("API error: " + apiError);
        } else {
            bool currentOk = extractCurrentWeather(doc);
            bool forecastOk = extractForecastWeather(doc);

            if (currentOk || forecastOk) {
                if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    _weatherData.dataValid = true;
                    _weatherData.errorMessage = "";
                    xSemaphoreGive(_weatherDataMutex);
                }
                success = true;
            } else {
                setErrorState("Failed to extract weather data");
            }
        }
    } else if (httpResponseCode == 401) {
        setErrorState("Invalid API key");
        _logger.error("WeatherAPI authentication failed - check API key");
    } else if (httpResponseCode == 403) {
        setErrorState("API key quota exceeded");
        _logger.error("WeatherAPI quota exceeded");
    } else if (httpResponseCode > 0) {
        setErrorState("HTTP error: " + String(httpResponseCode));
        _logger.error("WeatherAPI HTTP error: " + String(httpResponseCode));
    } else {
        setErrorState("Network error: " + String(httpResponseCode));
        _logger.error("WeatherAPI network error: " + String(httpResponseCode));
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
        String reason = "";
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

void WeatherPoller::setEmergencyStowState(bool active, const String& reason) {
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
        _windSafetyData.stowReason = reason;
        
        if (active) {
            _windSafetyData.currentStowDirection = stowDirection;
            
            if (!wasActive) {
                _logger.warn("EMERGENCY WIND STOW ACTIVATED: " + reason + 
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
        _weatherData.currentConditionText = (condText != nullptr) ? String(condText) : "";
        _weatherData.currentIsThunderstorm = isThunderstormCode(condCode);

        // Handle time strings carefully to avoid memory leaks
        const char* timeStr = current["last_updated"];
        if (timeStr != nullptr) {
            _weatherData.currentTime = String(timeStr);
            _weatherData.lastUpdateTime = formatWeatherApiTime(_weatherData.currentTime);
        } else {
            _weatherData.currentTime = "Unknown";
            _weatherData.lastUpdateTime = "Unknown";
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
    String currentTimeStr = "";
    if (doc.containsKey("current") && doc["current"].containsKey("last_updated")) {
        const char* timeStr = doc["current"]["last_updated"];
        if (timeStr != nullptr) {
            currentTimeStr = String(timeStr);
        }
    }

    // Local structure for parsing outside mutex
    struct ForecastEntry {
        String time;
        float windSpeed;
        float windDirection;
        float windGust;
        float tempC;
        float precipMm;
        float snowCm;
        int humidity;
        int conditionCode;
        String conditionText;
        bool isThunderstorm;
    };
    ForecastEntry localForecast[3];
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
                int hourValue = getHourFromTimeString(String(hourTimeStr));
                if (hourValue <= currentHour) continue;
            }

            localForecast[forecastCount].time = String(hourTimeStr);
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
            localForecast[forecastCount].conditionText = (ft != nullptr) ? String(ft) : "";
            localForecast[forecastCount].isThunderstorm = isThunderstormCode(fc);

            _logger.debug("Forecast " + String(forecastCount) + ": " +
                         localForecast[forecastCount].time + " - Wind: " +
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
                _weatherData.forecastTimes[i] = localForecast[i].time;
                _weatherData.forecastWindSpeed[i] = localForecast[i].windSpeed;
                _weatherData.forecastWindDirection[i] = localForecast[i].windDirection;
                _weatherData.forecastWindGust[i] = localForecast[i].windGust;
                _weatherData.forecastTempC[i] = localForecast[i].tempC;
                _weatherData.forecastPrecipMm[i] = localForecast[i].precipMm;
                _weatherData.forecastSnowCm[i] = localForecast[i].snowCm;
                _weatherData.forecastHumidity[i] = localForecast[i].humidity;
                _weatherData.forecastConditionCode[i] = localForecast[i].conditionCode;
                _weatherData.forecastConditionText[i] = localForecast[i].conditionText;
                _weatherData.forecastIsThunderstorm[i] = localForecast[i].isThunderstorm;
            } else {
                _weatherData.forecastTimes[i] = "";
                _weatherData.forecastWindSpeed[i] = 0.0f;
                _weatherData.forecastWindDirection[i] = 0.0f;
                _weatherData.forecastWindGust[i] = 0.0f;
                _weatherData.forecastTempC[i] = 0.0f;
                _weatherData.forecastPrecipMm[i] = 0.0f;
                _weatherData.forecastSnowCm[i] = 0.0f;
                _weatherData.forecastHumidity[i] = 0;
                _weatherData.forecastConditionCode[i] = 0;
                _weatherData.forecastConditionText[i] = "";
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
        // Clear all data and explicitly reset strings to free memory
        _weatherData.currentWindSpeed = 0.0;
        _weatherData.currentWindGust = 0.0;
        _weatherData.currentWindDirection = 0.0;
        _weatherData.currentTime = "";
        _weatherData.lastUpdateTime = "Never";
        _weatherData.errorMessage = "";
        _weatherData.dataValid = false;

        // Clear current non-wind fields
        _weatherData.currentTempC = 0.0;
        _weatherData.currentPrecipMm = 0.0;
        _weatherData.currentHumidity = 0;
        _weatherData.currentConditionCode = 0;
        _weatherData.currentConditionText = "";
        _weatherData.currentIsThunderstorm = false;

        // Clear forecast arrays
        for (int i = 0; i < 3; i++) {
            _weatherData.forecastWindSpeed[i] = 0.0;
            _weatherData.forecastWindGust[i] = 0.0;
            _weatherData.forecastWindDirection[i] = 0.0;
            _weatherData.forecastTimes[i] = "";
            _weatherData.forecastTempC[i] = 0.0;
            _weatherData.forecastPrecipMm[i] = 0.0;
            _weatherData.forecastSnowCm[i] = 0.0;
            _weatherData.forecastHumidity[i] = 0;
            _weatherData.forecastConditionCode[i] = 0;
            _weatherData.forecastConditionText[i] = "";
            _weatherData.forecastIsThunderstorm[i] = false;
        }

        xSemaphoreGive(_weatherDataMutex);
    }
}

void WeatherPoller::setErrorState(const String& error) {
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _weatherData.errorMessage = error;
        // Keep previous data valid on transient failures so wind safety
        // continues working with stale-but-usable data.
        // Only invalidate if we never had a successful update.
        if (_lastSuccessTime == 0) {
            _weatherData.dataValid = false;
        }
        xSemaphoreGive(_weatherDataMutex);
    }
    _logger.error("Weather polling error: " + error);
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
    
    if (!isValidApiKey(trimmedKey)) {
        _logger.error("Invalid API key format");
        return false;
    }
    
    if (_apiKeyMutex != NULL && xSemaphoreTake(_apiKeyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _apiKey = trimmedKey;
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

String WeatherPoller::getApiKey() {
    String key = "";
    if (_apiKeyMutex != NULL && xSemaphoreTake(_apiKeyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        key = _apiKey;
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
    String key = getApiKey();
    return isValidApiKey(key);
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

String WeatherPoller::getLastError() {
    String error = "";
    
    if (_weatherDataMutex != NULL && xSemaphoreTake(_weatherDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        error = _weatherData.errorMessage;
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

String WeatherPoller::buildApiUrl() {
    if (!isFullyConfigured()) {
        return "";
    }
    
    String apiKey = getApiKey();
    if (apiKey.isEmpty()) {
        return "";
    }
    
    String url = "https://api.weatherapi.com/v1/forecast.json";
    url += "?key=" + apiKey;
    url += "&q=" + String(_latitude.load(), 6) + "," + String(_longitude.load(), 6);
    url += "&days=2";
    url += "&aqi=no";
    url += "&alerts=no";
    
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

int WeatherPoller::getCurrentHourFromTime(const String& timeStr) {
    // Parse time string like "2024-01-15 17:30" to extract hour (17)
    if (timeStr.length() == 0) {
        return -1; // Invalid time
    }
    
    int spaceIndex = timeStr.indexOf(' ');
    if (spaceIndex == -1) {
        return -1; // No space found
    }
    
    String timePart = timeStr.substring(spaceIndex + 1); // "17:30"
    int colonIndex = timePart.indexOf(':');
    if (colonIndex == -1) {
        return -1; // No colon found
    }
    
    String hourStr = timePart.substring(0, colonIndex); // "17"
    return hourStr.toInt();
}

int WeatherPoller::getHourFromTimeString(const String& timeStr) {
    // Parse time string like "2024-01-15 18:00" to extract hour (18)
    return getCurrentHourFromTime(timeStr);
}

String WeatherPoller::formatWeatherApiTime(const String& apiTime) {
    // WeatherAPI returns time like "2024-01-15 14:30" (local time at location)
    // We'll just clean it up for display
    if (apiTime.length() == 0) {
        return "Unknown";
    }
    
    // Find the space between date and time
    int spaceIndex = apiTime.indexOf(' ');
    if (spaceIndex == -1) {
        return apiTime; // Return as-is if format is unexpected
    }
    
    String datePart = apiTime.substring(0, spaceIndex);
    String timePart = apiTime.substring(spaceIndex + 1);
    
    // Return in format "Jan 15, 14:30" for better readability
    // For now, just return time part since that's most relevant
    return timePart + " (local)";
}

String WeatherPoller::getRelativeUpdateTime() {
    // This method is kept for backward compatibility but uses weather data timestamp
    WeatherData data = getWeatherData();
    return data.lastUpdateTime;
}

bool WeatherPoller::isValidCoordinate(float lat, float lon) {
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 && 
            (lat != 0.0 || lon != 0.0)); // Exclude 0,0 as it's likely unset
}

bool WeatherPoller::isValidApiKey(const String& key) {
    // WeatherAPI.com keys are typically 32 characters long and alphanumeric
    if (key.length() < 16 || key.length() > 64) {
        return false;
    }
    
    // Check for basic alphanumeric pattern (letters, numbers, possibly some symbols)
    for (int i = 0; i < key.length(); i++) {
        char c = key.charAt(i);
        if (!isAlphaNumeric(c) && c != '_' && c != '-') {
            return false;
        }
    }
    
    return true;
}