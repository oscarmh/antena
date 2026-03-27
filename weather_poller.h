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

#ifndef WEATHER_POLLER_H
#define WEATHER_POLLER_H

// System includes
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <atomic>

// Custom includes
#include "logger.h"

struct WeatherData {
    float currentWindSpeed = 0.0;       // km/h
    float currentWindGust = 0.0;        // km/h
    float currentWindDirection = 0.0;   // degrees
    char currentTime[24] = "";

    // Current conditions (non-wind)
    float currentTempC = 0.0;
    float currentPrecipMm = 0.0;
    int currentHumidity = 0;
    int currentConditionCode = 0;
    char currentConditionText[64] = "";
    bool currentIsThunderstorm = false;

    // 3-hour forecast arrays
    float forecastWindSpeed[3] = {0.0, 0.0, 0.0};
    float forecastWindGust[3] = {0.0, 0.0, 0.0};
    float forecastWindDirection[3] = {0.0, 0.0, 0.0};
    char forecastTimes[3][24] = {};

    // Forecast conditions (non-wind)
    float forecastTempC[3] = {0.0, 0.0, 0.0};
    float forecastPrecipMm[3] = {0.0, 0.0, 0.0};
    float forecastSnowCm[3] = {0.0, 0.0, 0.0};
    int forecastHumidity[3] = {0, 0, 0};
    int forecastConditionCode[3] = {0, 0, 0};
    char forecastConditionText[3][64] = {};
    bool forecastIsThunderstorm[3] = {false, false, false};

    bool dataValid = false;
    char lastUpdateTime[24] = "";
    char errorMessage[128] = "";
};

struct WindSafetyData {
    bool emergencyStowActive = false;
    bool forecastStowActive = false;  // Keep this if you use it elsewhere
    float currentStowDirection = 0.0;
    char stowReason[64] = "";
};

class WeatherPoller {
public:
    // Constructor
    WeatherPoller(Preferences& prefs, Logger& logger);

    // Core functionality
    void begin();
    void runWeatherLoop(bool wifiConnected);

    // Configuration methods
    bool setLocation(float latitude, float longitude);
    bool setApiKey(const String& apiKey);
    float getLatitude();
    float getLongitude();
    const char* getApiKey();
    bool isLocationConfigured();
    bool isApiKeyConfigured();
    bool isFullyConfigured();

    // Data access methods (thread-safe)
    WeatherData getWeatherData();
    bool isDataValid();
    const char* getLastError();
    unsigned long getLastUpdateTime();

    // Control methods
    void forceUpdate();
    bool isPollingEnabled();
    void setPollingEnabled(bool enabled);

    // Sun/Moon display toggle
    void setShowSunMoon(bool enabled);
    bool isShowSunMoon();

    // Unit display toggle
    void setUseImperial(bool enabled);
    bool isUseImperial();

    // Wind safety methods
    void setWindSafetyEnabled(bool enabled);
    bool isWindSafetyEnabled();
    void setWindSpeedThreshold(float threshold);
    float getWindSpeedThreshold();
    void setWindGustThreshold(float threshold);
    float getWindGustThreshold();
    void setWindBasedHomeEnabled(bool enabled);
    bool isWindBasedHomeEnabled();
    
    WindSafetyData getWindSafetyData();
    bool shouldActivateEmergencyStow();
    float calculateOptimalStowDirection(float windDirection);
    float getWindBasedHomePosition();

private:
    // Dependencies
    Preferences& _preferences;
    Logger& _logger;

    // Configuration
    std::atomic<float> _latitude{0.0};
    std::atomic<float> _longitude{0.0};
    std::atomic<bool> _pollingEnabled{false};
    char _apiKey[65] = "";
    
    // Sun/Moon display
    std::atomic<bool> _showSunMoon{true};

    // Unit display preference
    std::atomic<bool> _useImperial{false};

    // Wind safety configuration
    std::atomic<bool> _windSafetyEnabled{false};
    std::atomic<float> _windSpeedThreshold{50.0};    // km/h
    std::atomic<float> _windGustThreshold{60.0};     // km/h
    std::atomic<bool> _windBasedHomeEnabled{false};
    
    // Timing constants
    static constexpr unsigned long POLL_INTERVAL_MS = 300000;       // 5 minutes
    static constexpr unsigned long RETRY_INTERVAL_MS = 60000;       // 60 seconds on error
    static constexpr unsigned long HTTP_CONNECT_TIMEOUT_MS = 15000; // 15s for TCP+TLS handshake
    static constexpr unsigned long HTTP_READ_TIMEOUT_MS = 30000;    // 30s for data transfer
    static constexpr unsigned long TRUNCATION_RETRY_MS = 10000;     // 10s retry after truncation

    // State variables
    std::atomic<unsigned long> _lastPollTime{0};
    std::atomic<unsigned long> _lastSuccessTime{0};
    std::atomic<bool> _forceUpdate{false};
    std::atomic<bool> _lastPollWasTruncation{false};
    
    // Thread synchronization
    SemaphoreHandle_t _weatherDataMutex = NULL;
    SemaphoreHandle_t _apiKeyMutex = NULL;
    SemaphoreHandle_t _windSafetyMutex = NULL;
    
    // Weather data storage
    WeatherData _weatherData;
    WindSafetyData _windSafetyData;

    // Persistent HTTP client — plain HTTP, no TLS overhead (~40KB saved).
    WiFiClient _httpClient;

    // Pre-allocated response buffer — lives in BSS, zero heap cost.
    // TLS reads into this, then TLS is freed before JSON parsing.
    static constexpr size_t RESPONSE_BUF_SIZE = 12288;
    char _responseBuf[RESPONSE_BUF_SIZE];

    // Core functionality helpers
    bool shouldPollWeather();
    bool pollWeatherData();
    const char* buildApiUrl();
    
    // Data processing helpers
    bool extractCurrentWeather(JsonDocument& doc);
    bool extractForecastWeather(JsonDocument& doc);
    void clearWeatherData();
    void setErrorState(const char* error);
    
    // Wind safety helpers
    void updateWindSafetyStatus();
    bool checkCurrentWindConditions();
    bool checkForecastWindConditions();
    void setEmergencyStowState(bool active, const char* reason);
    
    // Weather condition helpers
    bool isThunderstormCode(int code);

    // Utility methods
    float validateWindSpeed(float speed);
    float validateWindDirection(float direction);
    void formatWeatherApiTime(const char* apiTime, char* out, size_t outSize);
    const char* getRelativeUpdateTime();
    int getCurrentHourFromTime(const char* timeStr);
    int getHourFromTimeString(const char* timeStr);
    bool isValidCoordinate(float lat, float lon);
    bool isValidApiKey(const char* key);
    float normalizeAngle(float angle);
};

#endif // WEATHER_POLLER_H