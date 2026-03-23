var logScrollPaused = false;
var logLines = [];
var MAX_LOG_LINES = 100000;
var useImperial = false;
var cachedWindDir = null;
var cachedWeatherValid = false;
var cachedEmergencyStow = "NO";
var cachedWindStowReason = "";

function convertTemp(c) {
  if (c === null || c === undefined || c === "N/A") return c;
  if (!useImperial) return c;
  return ((parseFloat(c) * 9 / 5) + 32).toFixed(1);
}
function convertSpeed(kph) {
  if (kph === null || kph === undefined || kph === "N/A") return kph;
  if (!useImperial) return kph;
  return (parseFloat(kph) * 0.621371).toFixed(1);
}
function convertPrecip(mm) {
  if (mm === null || mm === undefined || mm === "N/A") return mm;
  if (!useImperial) return mm;
  return (parseFloat(mm) / 25.4).toFixed(2);
}
function convertSnow(cm) {
  if (cm === null || cm === undefined || cm === "N/A") return cm;
  if (!useImperial) return cm;
  return (parseFloat(cm) / 2.54).toFixed(2);
}
function updateUnitLabels() {
  var e;
  e = document.getElementById("unitCurrentTemp");
  if (e) e.innerHTML = useImperial ? "&deg;F" : "&deg;C";
  e = document.getElementById("unitCurrentPrecip");
  if (e) e.innerHTML = useImperial ? "in" : "mm";
  e = document.getElementById("unitCurrentWindSpeed");
  if (e) e.innerHTML = useImperial ? "mph" : "km/h";
  e = document.getElementById("unitCurrentWindGust");
  if (e) e.innerHTML = useImperial ? "mph" : "km/h";
  e = document.getElementById("unitForecastTemp");
  if (e) e.innerHTML = useImperial ? "Temp (&deg;F)" : "Temp (&deg;C)";
  e = document.getElementById("unitForecastWind");
  if (e) e.innerHTML = useImperial ? "Wind (mph)" : "Wind (km/h)";
  e = document.getElementById("unitForecastGust");
  if (e) e.innerHTML = useImperial ? "Gust (mph)" : "Gust (km/h)";
  e = document.getElementById("unitForecastPrecip");
  if (e) e.innerHTML = useImperial ? "Precip (in)" : "Precip (mm)";
  e = document.getElementById("unitForecastSnow");
  if (e) e.innerHTML = useImperial ? "Snow (in)" : "Snow (cm)";
  e = document.getElementById("unitWindSpeedThreshold");
  if (e) e.innerHTML = useImperial ? "mph" : "km/h";
  e = document.getElementById("unitWindGustThreshold");
  if (e) e.innerHTML = useImperial ? "mph" : "km/h";
  // Update validation ranges for speed-converted fields
  var speedFields = document.querySelectorAll('[data-convert="speed"]');
  for (var i = 0; i < speedFields.length; i++) {
    if (useImperial) {
      speedFields[i].setAttribute('data-min', '6');
      speedFields[i].setAttribute('data-max', '125');
    } else {
      speedFields[i].setAttribute('data-min', '10');
      speedFields[i].setAttribute('data-max', '200');
    }
  }
}

function xget(url,cb){var x=new XMLHttpRequest();x.open("GET",url,true);if(cb)x.onreadystatechange=function(){if(x.readyState==4)cb(x)};x.send()}
// s() — set innerHTML, but skip if an inline edit is active in the element
function s(id,v){var e=document.getElementById(id);if(e && !e.querySelector('input,select'))e.innerHTML=v}
function confirmPost(msg,action){if(confirm(msg)){var f=document.createElement('form');f.method='POST';f.action=action;document.body.appendChild(f);f.submit()}}

// Intercept form submissions to config endpoints — submit via XHR and refresh config
var configEndpoints = ['/setWeatherApiKey', '/setPassword'];
document.addEventListener('submit', function(e) {
  var form = e.target;
  var action = form.getAttribute('action') || '';
  var isConfig = false;
  for (var i = 0; i < configEndpoints.length; i++) {
    if (action === configEndpoints[i]) { isConfig = true; break; }
  }
  if (!isConfig) return;
  e.preventDefault();
  var params = [];
  for (var i = 0; i < form.elements.length; i++) {
    var el = form.elements[i];
    if (!el.name || el.type === 'submit' || el.type === 'button') continue;
    if ((el.type === 'checkbox' || el.type === 'radio') && !el.checked) continue;
    if (el.disabled) continue;
    params.push(encodeURIComponent(el.name) + '=' + encodeURIComponent(el.value));
  }
  var xhr = new XMLHttpRequest();
  xhr.open(form.method || 'POST', action, true);
  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4) {
      if (xhr.status >= 200 && xhr.status < 300) {
        fetchConfig();
      } else if (xhr.status >= 400) {
        alert('Error: ' + (xhr.responseText || 'Request failed'));
      }
    }
  };
  xhr.send(params.join('&'));
});

// Fetch configuration data on page load (and after settings changes)
// Retries on failure since the ESP32 may be busy serving other page assets
function fetchConfig() {
  xget("/config", function(x) {
    if (x.status != 200) { setTimeout(fetchConfig, 2000); return; }
    var data;
    try { data = JSON.parse(x.responseText); } catch(e) { setTimeout(fetchConfig, 2000); return; }

    s("http_port",data.http_port);
    s("rotctl_port",data.rotctl_port);
    s("hostname",data.hostname);
    s("wifissid",data.wifissid);
    s("loginUserDisplay",data.loginUser || "Not Set");
    s("passwordStatus",data.passwordStatus);
    s("maxDualMotorAzSpeed",data.maxDualMotorAzSpeed);
    s("maxDualMotorElSpeed",data.maxDualMotorElSpeed);
    s("maxSingleMotorAzSpeed",data.maxSingleMotorAzSpeed);
    s("maxSingleMotorElSpeed",data.maxSingleMotorElSpeed);
    s("stellariumPollingOn",data.stellariumPollingOn);
    s("stellariumServerIPText",data.stellariumServerIPText);
    s("stellariumServerPortText",data.stellariumServerPortText);
    s("toleranceAz",data.toleranceAz);
    s("toleranceEl",data.toleranceEl);
    s("P_el",data.P_el);
    s("P_az",data.P_az);
    s("MIN_EL_SPEED",data.MIN_EL_SPEED);
    s("MIN_AZ_SPEED",data.MIN_AZ_SPEED);
    s("MIN_AZ_TOLERANCE",data.MIN_AZ_TOLERANCE);
    s("MIN_EL_TOLERANCE",data.MIN_EL_TOLERANCE);
    s("MAX_FAULT_POWER_AZ",data.MAX_FAULT_POWER_AZ);
    s("MAX_FAULT_POWER_EL",data.MAX_FAULT_POWER_EL);
    s("MAX_FAULT_POWER_TOTAL",data.MAX_FAULT_POWER_TOTAL);
    s("MIN_VOLTAGE_THRESHOLD",data.MIN_VOLTAGE_THRESHOLD);
    s("I_el",data.I_el);
    s("I_az",data.I_az);
    s("D_el",data.D_el);
    s("D_az",data.D_az);
    s("azOffset",data.azOffset);
    s("elOffset",data.elOffset);
    s("weatherEnabled",data.weatherEnabled);
    s("weatherApiKeyConfigured",data.weatherApiKeyConfigured);
    s("weatherLocationConfigured",data.weatherLocationConfigured);
    s("weatherLatitude",data.weatherLatitude);
    s("weatherLongitude",data.weatherLongitude);
    s("showSunMoon",data.showSunMoon);
    s("weatherUnits",data.weatherUnits);
    useImperial = (data.weatherUnits === "Imperial");
    updateUnitLabels();
    s("windSafetyEnabled",data.windSafetyEnabled);
    s("windBasedHomeEnabled",data.windBasedHomeEnabled);
    s("windSpeedThreshold",convertSpeed(data.windSpeedThreshold));
    s("windGustThreshold",convertSpeed(data.windGustThreshold));
    s("autoHomeEnabled",data.autoHomeEnabled);
    s("autoHomeMins",data.autoHomeMins);
    s("singleMotorModeText",data.singleMotorModeText);
    s("directionLockStatus",data.directionLockEnabled);
    s("directionLockSetting",data.directionLockEnabled);
    s("safeModeStatus",data.safeMode);
    var safeCb = document.getElementById('safeModeToggle');
    if (safeCb) safeCb.checked = (data.safeMode === "ON");
    var safeLbl = document.getElementById('safeModeLabel');
    if (safeLbl) safeLbl.textContent = (data.safeMode === "ON") ? "SAFE MODE ON" : "SAFE MODE";
    s("smoothTrackingActive",data.smoothTrackingActive);
    s("smoothTrackingEnabled",data.smoothTrackingEnabled);
    s("smKalQ",data.smKalQ);
    s("smKalR",data.smKalR);
    s("smMinAzSpd",data.smMinAzSpd);
    s("smMinElSpd",data.smMinElSpd);
  });
}
fetchConfig();

// Real-time status polling
var pollInFlight = false;
setInterval(function() {
  if (pollInFlight) return;
  pollInFlight = true;
  var xhr = new XMLHttpRequest();
  xhr.timeout = 5000;
  xhr.open("GET", "/status", true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4) {
      pollInFlight = false;
      if (xhr.status != 200) return;
      var data;
      try {
        data = JSON.parse(xhr.responseText);
      } catch(e) {
        return;
      }
      s("correctedAngle_el",data.correctedAngle_el);
      s("correctedAngle_az",data.correctedAngle_az);
      s("setpoint_az",data.setpoint_az);
      s("setpoint_el",data.setpoint_el);
      s("setPointState_az",data.setPointState_az);
      s("setPointState_el",data.setPointState_el);
      s("error_az",data.error_az);
      s("error_el",data.error_el);
      s("el_startAngle",data.el_startAngle);
      s("needs_unwind",data.needs_unwind);
      s("i2cErrorFlag_az",data.i2cErrorFlag_az);
      s("i2cErrorFlag_el",data.i2cErrorFlag_el);
      s("badAngleFlag",data.badAngleFlag);
      s("magnetFault",data.magnetFault);
      s("faultTripped",data.faultTripped);
      s("isAzMotorLatched",data.isAzMotorLatched);
      s("isElMotorLatched",data.isElMotorLatched);
      s("motorSpeedPctAz",data.motorSpeedPctAz);
      s("motorSpeedPctEl",data.motorSpeedPctEl);
      s("inputVoltage",data.inputVoltage);
      s("currentDraw",data.currentDraw);
      s("rotatorPowerDraw",data.rotatorPowerDraw);

      var windStowAlert = document.getElementById("windStowAlert");
      var windStowMessage = document.getElementById("windStowMessage");
      if (cachedEmergencyStow === "YES") {
        windStowMessage.innerHTML = "EMERGENCY WIND STOW ACTIVE: " + cachedWindStowReason;
        windStowAlert.style.display = "block";
      } else {
        windStowAlert.style.display = "none";
      }

      if (data.newLogMessages && data.newLogMessages.trim() !== "") {
        var newLines = data.newLogMessages.split('\n');
        for (var i = 0; i < newLines.length; i++) {
          if (newLines[i].trim() !== "") {
            logLines.push(newLines[i]);
            if (logLines.length > MAX_LOG_LINES) {
              logLines.shift();
            }
          }
        }
      }

      updateLogDisplay();

      updateDebugLevelDisplay(data.currentDebugLevel);

      var azimuth = data.correctedAngle_az;
      var elevation = data.correctedAngle_el;
      var setpoint_az = data.setpoint_az;
      var setpoint_el = data.setpoint_el;
      var kalmanAz = data.kalmanAzPos != null ? parseFloat(data.kalmanAzPos) : null;
      var kalmanEl = data.kalmanElPos != null ? parseFloat(data.kalmanElPos) : null;
      if (data.kalmanAzVel) s("kalmanAzVel", data.kalmanAzVel + " deg/s");
      if (data.kalmanElVel) s("kalmanElVel", data.kalmanElVel + " deg/s");

      // Calculate sun/moon positions if enabled
      var showSunMoonEl = document.getElementById("showSunMoon");
      var showSunMoonOn = showSunMoonEl && showSunMoonEl.textContent.trim() === "ON";
      var wLat = parseFloat(document.getElementById("weatherLatitude") ? document.getElementById("weatherLatitude").textContent.trim() : "0");
      var wLon = parseFloat(document.getElementById("weatherLongitude") ? document.getElementById("weatherLongitude").textContent.trim() : "0");
      var sunPos = null, moonPos = null;
      if (showSunMoonOn && !isNaN(wLat) && !isNaN(wLon) && (wLat !== 0 || wLon !== 0)) {
        var now = new Date();
        sunPos = SunCalc.getPosition(now, wLat, wLon);
        moonPos = SunCalc.getMoonPosition(now, wLat, wLon);
        // Convert from suncalc (radians, az from south CW) to compass degrees (az from north CW)
        var sunAzDeg = (sunPos.azimuth * 180 / Math.PI + 180) % 360;
        var sunElDeg = sunPos.altitude * 180 / Math.PI;
        var moonAzDeg = (moonPos.azimuth * 180 / Math.PI + 180) % 360;
        var moonElDeg = moonPos.altitude * 180 / Math.PI;
        s("sunAzimuth", sunAzDeg.toFixed(1) + "\u00B0");
        s("sunElevation", sunElDeg.toFixed(1) + "\u00B0");
        s("sunStatus", sunElDeg > 0 ? "Above Horizon" : "Below Horizon");
        s("moonAzimuth", moonAzDeg.toFixed(1) + "\u00B0");
        s("moonElevation", moonElDeg.toFixed(1) + "\u00B0");
        s("moonStatus", moonElDeg > 0 ? "Above Horizon" : "Below Horizon");
        sunPos = { az: sunAzDeg, el: sunElDeg };
        moonPos = { az: moonAzDeg, el: moonElDeg };
      } else {
        s("sunAzimuth","N/A"); s("sunElevation","N/A"); s("sunStatus","N/A");
        s("moonAzimuth","N/A"); s("moonElevation","N/A"); s("moonStatus","N/A");
      }

      drawSkyplane();
      drawSunMoon(sunPos, moonPos);
      drawPositions(azimuth, elevation, setpoint_az, setpoint_el, kalmanAz, kalmanEl);
      drawWindDirection(cachedWindDir, cachedWeatherValid);
    }
  };
  xhr.ontimeout = function() { pollInFlight = false; };
  xhr.onerror = function() { pollInFlight = false; };
  xhr.send();
}, 250);

// Slow-changing info polling (network, weather, wind safety)
var infoPollInFlight = false;
setInterval(function() {
  if (infoPollInFlight) return;
  infoPollInFlight = true;
  var xhr = new XMLHttpRequest();
  xhr.timeout = 5000;
  xhr.open("GET", "/info", true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4) {
      infoPollInFlight = false;
      if (xhr.status != 200) return;
      var data;
      try { data = JSON.parse(xhr.responseText); } catch(e) { return; }

      // Network
      s("ip_addr", data.ip_addr);
      s("rotctl_client_ip", data.rotctl_client_ip);
      s("bssid", data.bssid);
      s("wifi_channel", data.wifi_channel);
      s("rssi", data.rssi);
      var level = data.level;
      for(var i=1;i<5;i++) document.getElementById('bar'+i).classList.toggle('active',level>=i);

      // Slow-changing status flags
      s("extendedElEnabled", data.extendedElEnabled);
      var elSetpointLabel = document.getElementById("elSetpointLabel");
      if (elSetpointLabel) {
        elSetpointLabel.innerHTML = (data.extendedElEnabled === "ON") ? "Set Elevation (-90 - 90):" : "Set Elevation (0 - 90):";
      }
      s("stellariumConnActive", data.stellariumConnActive);
      s("serialOutputDisabled", data.serialOutputDisabled ? "True" : "False");
      s("calMode", data.calMode);
      var calSec = document.getElementById("calMoveSection");
      if (calSec) calSec.style.display = (data.calMode === "ON") ? "block" : "none";
      s("serialActive", data.serialActive);
      s("singleMotorModeText", data.singleMotorModeText);
      s("directionLockStatus", data.directionLockEnabled);
      s("directionLockSetting", data.directionLockEnabled);
      s("smoothTrackingActive", data.smoothTrackingActive);
      s("safeModeStatus", data.safeMode);
      var safeCb = document.getElementById('safeModeToggle');
      if (safeCb) safeCb.checked = (data.safeMode === "ON");
      var safeLbl = document.getElementById('safeModeLabel');
      if (safeLbl) safeLbl.textContent = (data.safeMode === "ON") ? "SAFE MODE ON" : "SAFE MODE";

      // Weather data
      s("weatherDataValid", data.weatherDataValid);
      s("weatherLastUpdate", data.weatherLastUpdate);
      s("currentWindSpeed", convertSpeed(data.currentWindSpeed));
      s("currentWindDirection", formatWindDirection(data.currentWindDirection));
      s("currentWindGust", convertSpeed(data.currentWindGust));
      s("currentWeatherTime", formatWeatherTime(data.currentWeatherTime));
      s("currentTempC", data.currentTempC != null ? convertTemp(data.currentTempC) : "N/A");
      s("currentPrecipMm", data.currentPrecipMm != null ? convertPrecip(data.currentPrecipMm) : "N/A");
      s("currentHumidity", data.currentHumidity != null ? data.currentHumidity : "N/A");
      s("currentConditionText", data.currentConditionText || "N/A");
      var stormEl = document.getElementById("currentIsThunderstorm");
      if (stormEl && !stormEl.querySelector('input,select')) {
        stormEl.innerHTML = (data.currentIsThunderstorm === "YES") ? "YES" : "No";
        stormEl.className = (data.currentIsThunderstorm === "YES") ? "storm-yes" : "";
      }

      // Forecast data
      var hasForecastData = data.forecastWindSpeed &&
                            data.forecastWindDirection &&
                            data.forecastWindGust &&
                            data.forecastTimes;
      if (hasForecastData) {
        for (var i = 0; i < 3; i++) {
          var time = (data.forecastTimes.length > i) ? data.forecastTimes[i] : null;
          var speed = (data.forecastWindSpeed.length > i) ? data.forecastWindSpeed[i] : null;
          var direction = (data.forecastWindDirection.length > i) ? data.forecastWindDirection[i] : null;
          var gust = (data.forecastWindGust.length > i) ? data.forecastWindGust[i] : null;
          s("forecastTime" + i, formatWeatherTime(time));
          s("forecastWindSpeed" + i, (speed !== null) ? convertSpeed(speed) : "N/A");
          s("forecastWindDirection" + i, formatWindDirection(direction));
          s("forecastWindGust" + i, (gust !== null) ? convertSpeed(gust) : "N/A");
          s("forecastCondition" + i, data.forecastConditionText && data.forecastConditionText[i] ? data.forecastConditionText[i] : "N/A");
          s("forecastTemp" + i, data.forecastTempC && data.forecastTempC[i] != null ? convertTemp(data.forecastTempC[i]) : "N/A");
          s("forecastPrecip" + i, data.forecastPrecipMm && data.forecastPrecipMm[i] != null ? convertPrecip(data.forecastPrecipMm[i]) : "N/A");
          s("forecastSnow" + i, data.forecastSnowCm && data.forecastSnowCm[i] != null ? convertSnow(data.forecastSnowCm[i]) : "N/A");
          s("forecastHumidity" + i, data.forecastHumidity && data.forecastHumidity[i] != null ? data.forecastHumidity[i] : "N/A");
          var fStormEl = document.getElementById("forecastStorm" + i);
          if (fStormEl && !fStormEl.querySelector('input,select')) {
            var isStorm = data.forecastIsThunderstorm && data.forecastIsThunderstorm[i] === "YES";
            fStormEl.innerHTML = isStorm ? "YES" : "No";
            fStormEl.className = isStorm ? "storm-yes" : "";
          }
        }
      } else {
        for (var i = 0; i < 3; i++) {
          s("forecastTime" + i, "N/A");
          s("forecastWindSpeed" + i, "N/A");
          s("forecastWindDirection" + i, "N/A");
          s("forecastWindGust" + i, "N/A");
          s("forecastCondition" + i, "N/A");
          s("forecastTemp" + i, "N/A");
          s("forecastPrecip" + i, "N/A");
          s("forecastSnow" + i, "N/A");
          s("forecastHumidity" + i, "N/A");
          s("forecastStorm" + i, "N/A");
        }
      }

      // Weather error
      var weatherErrorDiv = document.getElementById("weatherErrorDiv");
      var weatherError = document.getElementById("weatherError");
      if (data.weatherError && data.weatherError !== "") {
        weatherError.innerHTML = "Error: " + data.weatherError;
        weatherErrorDiv.style.display = "block";
      } else {
        weatherErrorDiv.style.display = "none";
      }

      // Wind safety
      s("windStowActive", data.windStowActive);
      s("windStowReason", data.windStowReason);
      s("windTrackingActive", data.windTrackingActive);
      s("emergencyStowActive", data.emergencyStowActive);
      s("stowDirection", data.stowDirection);

      // Update cached values for fast poll canvas drawing
      cachedWindDir = data.currentWindDirection;
      cachedWeatherValid = (data.weatherDataValid === "YES");
      cachedEmergencyStow = data.emergencyStowActive;
      cachedWindStowReason = data.windStowReason;
    }
  };
  xhr.ontimeout = function() { infoPollInFlight = false; };
  xhr.onerror = function() { infoPollInFlight = false; };
  xhr.send();
}, 2000);

function nudgeSetpoint(axis, delta) {
  var currentSpan = document.getElementById("setpoint_" + axis);
  var current = parseFloat(currentSpan.innerHTML);
  if (isNaN(current)) return;
  var newValue = current + delta;

  if (axis === "az") {
    newValue = ((newValue % 360) + 360) % 360;
  } else {
    var extendedEl = document.getElementById("extendedElEnabled");
    var minEl = (extendedEl && extendedEl.textContent.trim() === "ON") ? -90 : 0;
    if (newValue < minEl) newValue = minEl;
    if (newValue > 90) newValue = 90;
  }

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/update_variable", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("new_setpoint_" + axis + "=" + newValue.toFixed(2));
}

function updateLogDisplay() {
  var errorTextArea = document.getElementById("errorMessages");
  if (logScrollPaused) return;
  if (logLines.length > 0) {
    errorTextArea.value = logLines.join('\n');
    errorTextArea.className = "has-errors";
    errorTextArea.scrollTop = errorTextArea.scrollHeight;
  } else {
    errorTextArea.value = "";
    errorTextArea.className = "";
  }
  updatePauseButtonText();
}

function toggleLogScrollPause() {
  logScrollPaused = !logScrollPaused;
  if (!logScrollPaused) updateLogDisplay();
  updatePauseButtonText();
}

function updatePauseButtonText() {
  var pauseBtn = document.getElementById('pauseScrollBtn');
  if (pauseBtn) {
    pauseBtn.innerHTML = logScrollPaused ? 'Resume' : 'Pause';
    pauseBtn.style.backgroundColor = logScrollPaused ? '#28a745' : '#ffc107';
  }
}

function clearErrorMessages() {
  logLines = [];
  updateLogDisplay();
}

function updateDebugLevelDisplay(currentLevel) {
  var levelNames = ["NONE", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE"];
  var el = document.getElementById("currentDebugLevel");
  if (el && !el.querySelector('input,select') && currentLevel >= 0 && currentLevel <= 5) {
    el.innerHTML = currentLevel + " (" + levelNames[currentLevel] + ")";
  }
}

var canvas = document.getElementById("skyplaneCanvas");
var ctx = canvas.getContext("2d");

var centerX = canvas.width / 2;
var centerY = canvas.height / 2;
var radius = Math.min(centerX, centerY) - 20;

function drawSkyplane() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.textAlign = "center";
  ctx.font = "16px Arial";

  for (let i = 4; i >= 0; i--) {
    const currentRadius = radius * (i / 4);
    ctx.beginPath();
    ctx.arc(centerX, centerY, currentRadius, 0, 2 * Math.PI);
    ctx.strokeStyle = i === 4 ? 'black' : 'gray';
    ctx.stroke();
  }

  ctx.beginPath();
  for (let az = 0; az < 360; az += 45) {
    const rad = az * (Math.PI / 180);
    const x = centerX + radius * Math.cos(rad);
    const y = centerY + radius * Math.sin(rad);
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(x, y);
  }
  ctx.strokeStyle = 'gray';
  ctx.stroke();

  ctx.fillStyle = "black";
  const directions = [
    { text: "N", x: centerX, y: centerY - radius - 10 },
    { text: "E", x: centerX + radius + 10, y: centerY + 5 },
    { text: "S", x: centerX, y: centerY + radius + 20 },
    { text: "W", x: centerX - radius - 10, y: centerY + 5 }
  ];
  directions.forEach(dir => {
    ctx.fillText(dir.text, dir.x, dir.y);
  });
}

function drawPositions(azimuth, elevation, setpoint_az, setpoint_el, kalmanAz, kalmanEl) {
  if (elevation >= 350) elevation = 0;

  const toRadians = Math.PI / 180;
  const adjustedAzimuth = -90 * toRadians;

  const positions = [
    { az: azimuth * toRadians + adjustedAzimuth, el: 1 - (elevation / 90), radius: 8, color: 'blue', fill: false },
    { az: setpoint_az * toRadians + adjustedAzimuth, el: 1 - (setpoint_el / 90), radius: 5, color: 'red', fill: true }
  ];

  // Add Kalman estimate as green marker when smooth tracking is active
  if (kalmanAz !== null && kalmanEl !== null && kalmanAz >= 0 && (kalmanEl >= 0 || kalmanEl >= -5)) {
    var kEl = kalmanEl >= 350 ? 0 : Math.max(kalmanEl, 0);
    positions.push({ az: kalmanAz * toRadians + adjustedAzimuth, el: 1 - (kEl / 90), radius: 4, color: '#00cc00', fill: true });
  }

  positions.forEach(pos => {
    let x = centerX + (radius * pos.el) * Math.cos(pos.az);
    let y = centerY + (radius * pos.el) * Math.sin(pos.az);

    const distanceFromCenter = Math.hypot(x - centerX, y - centerY);
    if (distanceFromCenter > radius) {
      const scale = radius / distanceFromCenter;
      x = centerX + (x - centerX) * scale;
      y = centerY + (y - centerY) * scale;
    }

    ctx.beginPath();
    ctx.arc(x, y, pos.radius, 0, 2 * Math.PI);
    if (pos.fill) {
      ctx.fillStyle = pos.color;
      ctx.fill();
    } else {
      ctx.strokeStyle = pos.color;
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  });
}

function drawWindDirection(windDirection, weatherDataValid) {
  if (!weatherDataValid || windDirection === null || windDirection === undefined ||
      windDirection === "N/A" || windDirection === "" || isNaN(parseFloat(windDirection))) {
    return;
  }

  const windDir = parseFloat(windDirection);
  const windRad = (windDir - 90) * (Math.PI / 180);
  const triangleRadius = radius * 1.08;
  const triangleSize = 8;

  const centerTriangleX = centerX + triangleRadius * Math.cos(windRad);
  const centerTriangleY = centerY + triangleRadius * Math.sin(windRad);

  const point1X = centerTriangleX + triangleSize * Math.cos(windRad);
  const point1Y = centerTriangleY + triangleSize * Math.sin(windRad);
  const point2X = centerTriangleX - triangleSize * 0.5 * Math.cos(windRad) + triangleSize * 0.866 * Math.cos(windRad + Math.PI/2);
  const point2Y = centerTriangleY - triangleSize * 0.5 * Math.sin(windRad) + triangleSize * 0.866 * Math.sin(windRad + Math.PI/2);
  const point3X = centerTriangleX - triangleSize * 0.5 * Math.cos(windRad) - triangleSize * 0.866 * Math.cos(windRad + Math.PI/2);
  const point3Y = centerTriangleY - triangleSize * 0.5 * Math.sin(windRad) - triangleSize * 0.866 * Math.sin(windRad + Math.PI/2);

  ctx.beginPath();
  ctx.moveTo(point1X, point1Y);
  ctx.lineTo(point2X, point2Y);
  ctx.lineTo(point3X, point3Y);
  ctx.closePath();
  ctx.fillStyle = '#FF6B35';
  ctx.fill();
  ctx.strokeStyle = '#E55A2B';
  ctx.lineWidth = 1;
  ctx.stroke();
}

function drawSunMoon(sunPos, moonPos) {
  if (!sunPos && !moonPos) return;
  var toRadians = Math.PI / 180;
  var adjustedAzimuth = -90 * toRadians;

  function drawBody(pos, isSun) {
    var el = pos.el;
    var aboveHorizon = (el > 0);
    // Clamp elevation: show objects down to -5 degrees at edge
    if (el < -5) return;
    var elClamped = Math.max(el, 0);
    var elFrac = 1 - (elClamped / 90);
    var azRad = pos.az * toRadians + adjustedAzimuth;
    var x = centerX + (radius * elFrac) * Math.cos(azRad);
    var y = centerY + (radius * elFrac) * Math.sin(azRad);
    // Clamp to circle
    var dist = Math.hypot(x - centerX, y - centerY);
    if (dist > radius) {
      var scale = radius / dist;
      x = centerX + (x - centerX) * scale;
      y = centerY + (y - centerY) * scale;
    }
    ctx.beginPath();
    if (isSun) {
      var r = 6;
      ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.fillStyle = aboveHorizon ? '#FFD700' : '#B8860B';
      ctx.fill();
      ctx.strokeStyle = aboveHorizon ? '#FFA500' : '#8B6914';
      ctx.lineWidth = 1;
      ctx.stroke();
      if (aboveHorizon) {
        for (var i = 0; i < 8; i++) {
          var angle = i * Math.PI / 4;
          ctx.beginPath();
          ctx.moveTo(x + (r + 2) * Math.cos(angle), y + (r + 2) * Math.sin(angle));
          ctx.lineTo(x + (r + 5) * Math.cos(angle), y + (r + 5) * Math.sin(angle));
          ctx.strokeStyle = '#FFD700';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        }
      }
    } else {
      var r = 5;
      ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.fillStyle = aboveHorizon ? '#C0C0C0' : '#808080';
      ctx.fill();
      ctx.strokeStyle = aboveHorizon ? '#A0A0A0' : '#606060';
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  if (sunPos) drawBody(sunPos, true);
  if (moonPos) drawBody(moonPos, false);
}

drawSkyplane();

// ============================================================================
// SunCalc - sun/moon position calculator (MIT License, mourner/suncalc)
// ============================================================================
var SunCalc = (function() {
  var PI = Math.PI, sin = Math.sin, cos = Math.cos, tan = Math.tan,
      asin = Math.asin, atan2 = Math.atan2,
      rad = PI / 180, dayMs = 86400000, J1970 = 2440588, J2000 = 2451545,
      e = rad * 23.4397; // obliquity of Earth

  function toJulian(date) { return date.valueOf() / dayMs - 0.5 + J1970; }
  function toDays(date) { return toJulian(date) - J2000; }

  function rightAscension(l, b) { return atan2(sin(l) * cos(e) - tan(b) * sin(e), cos(l)); }
  function declination(l, b) { return asin(sin(b) * cos(e) + cos(b) * sin(e) * sin(l)); }
  function azimuthCalc(H, phi, dec) { return atan2(sin(H), cos(H) * sin(phi) - tan(dec) * cos(phi)); }
  function altitudeCalc(H, phi, dec) { return asin(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H)); }
  function siderealTime(d, lw) { return rad * (280.16 + 360.9856235 * d) - lw; }

  function solarMeanAnomaly(d) { return rad * (357.5291 + 0.98560028 * d); }
  function eclipticLongitude(M) {
    var C = rad * (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M)),
        P = rad * 102.9372;
    return M + C + P + PI;
  }
  function sunCoords(d) {
    var M = solarMeanAnomaly(d), L = eclipticLongitude(M);
    return { dec: declination(L, 0), ra: rightAscension(L, 0) };
  }

  function getPosition(date, lat, lng) {
    var lw = rad * -lng, phi = rad * lat, d = toDays(date),
        c = sunCoords(d), H = siderealTime(d, lw) - c.ra;
    return { azimuth: azimuthCalc(H, phi, c.dec), altitude: altitudeCalc(H, phi, c.dec) };
  }

  function moonCoords(d) {
    var L = rad * (218.316 + 13.176396 * d),
        M = rad * (134.963 + 13.064993 * d),
        F = rad * (93.272 + 13.229350 * d),
        l = L + rad * 6.289 * sin(M),
        b = rad * 5.128 * sin(F),
        dt = 385001 - 20905 * cos(M);
    return { ra: rightAscension(l, b), dec: declination(l, b), dist: dt };
  }

  function getMoonPosition(date, lat, lng) {
    var lw = rad * -lng, phi = rad * lat, d = toDays(date),
        c = moonCoords(d), H = siderealTime(d, lw) - c.ra,
        h = altitudeCalc(H, phi, c.dec);
    // Parallax correction
    h = h + asin(6371 / c.dist) * cos(h);
    return { azimuth: azimuthCalc(H, phi, c.dec), altitude: h };
  }

  return { getPosition: getPosition, getMoonPosition: getMoonPosition };
})();

function calEl(){xget("/calEl")}
function moveEl(v){xget("/moveEl?value="+v)}
function moveAz(v){xget("/moveAz?value="+v)}

function updateVariable() {
  var azValue = parseFloat(document.getElementById("new_setpoint_az").value);
  var elValue = parseFloat(document.getElementById("new_setpoint_el").value);
  var errorMessageAz = document.getElementById("error_message_az");
  var errorMessageEl = document.getElementById("error_message_el");
  var valid = true;

  if (isNaN(azValue) || azValue < -360 || azValue > 360) {
    errorMessageAz.textContent = "Please enter a valid Azimuth between -360 and 360.";
    valid = false;
  } else {
    errorMessageAz.textContent = "";
  }

  var extendedEl = document.getElementById("extendedElEnabled");
  var minEl = (extendedEl && extendedEl.textContent.trim() === "ON") ? -90 : 0;
  if (isNaN(elValue) || elValue < minEl || elValue > 90) {
    errorMessageEl.textContent = "Please enter a valid Elevation between " + minEl + " and 90.";
    valid = false;
  } else {
    errorMessageEl.textContent = "";
  }

  if (!valid) return false;

  document.getElementById("new_setpoint_az").value = azValue.toFixed(2);
  document.getElementById("new_setpoint_el").value = elValue.toFixed(2);
  return true;
}

function copyToClipboard(id, el) {
  var text = document.getElementById(id).textContent.trim();
  if (navigator.clipboard) {
    navigator.clipboard.writeText(text);
  } else {
    var ta = document.createElement('textarea');
    ta.value = text;
    document.body.appendChild(ta);
    ta.select();
    document.execCommand('copy');
    document.body.removeChild(ta);
  }
  if (el) {
    var orig = el.textContent;
    el.textContent = '\u2705';
    el.classList.add('copy-flash');
    setTimeout(function() { el.textContent = orig; el.classList.remove('copy-flash'); }, 800);
  }
}

function submitHome() {
  fetch('/submitHome', { method: 'POST' });
}

function stopMotors() {
  fetch('/stopMotors', { method: 'POST' });
}

function toggleSafeMode() {
  var cb = document.getElementById('safeModeToggle');
  xget(cb.checked ? '/safeModeOn' : '/safeModeOff');
}


function toggleHotspotMode() {
  var hotspotSelect = document.getElementById("hotspotSelect");
  var hotspotHidden = document.getElementById("hotspotHidden");
  var ssidField = document.getElementById("ssid");
  var passwordField = document.getElementById("password");

  if (hotspotSelect.value === "on") {
    hotspotHidden.name = "hotspot";
    hotspotHidden.value = "on";
    ssidField.value = "discoverydish_HOTSPOT";
    passwordField.value = "discoverydish";
    passwordField.type = "text";
    ssidField.disabled = true;
    passwordField.disabled = true;
  } else {
    hotspotHidden.name = "";
    hotspotHidden.value = "";
    ssidField.value = "";
    passwordField.value = "";
    passwordField.type = "password";
    ssidField.disabled = false;
    passwordField.disabled = false;
  }
}

function confirmRestartESP32(){confirmPost("Are you sure you want to restart the ESP32?\n\nThis will temporarily disconnect the web interface and interrupt any ongoing operations.","/restart")}
function confirmResetNeedsUnwind(){confirmPost("Are you sure you want to reset Needs Unwind flag?\n\nThis could cause the rotator to over rotate and tangle cables.","/resetNeedsUnwind")}
function confirmResetEEPROM(){confirmPost("WARNING: Are you sure you want to reset the EEPROM?\n\nThis will erase all saved settings and return the device to factory defaults. This action cannot be undone.","/resetEEPROM")}

function toggleAdvancedParams() {
  var checkbox = document.getElementById("showAdvanced");
  var section = document.getElementById("advancedParamsSection");
  section.style.display = checkbox.checked ? "block" : "none";
}

function forceWeatherUpdate() {
  xget("/forceWeatherUpdate", function(x) {
    if (x.status == 200) {
      alert("Weather update requested. Data will refresh shortly.");
    } else {
      var errorMsg = "Failed to force weather update";
      if (x.responseText) {
        errorMsg += ": " + x.responseText;
      }
      if (x.responseText.includes("Location not configured")) {
        errorMsg += "\nPlease set your location first.";
      } else if (x.responseText.includes("API key not configured")) {
        errorMsg += "\nPlease set your WeatherAPI.com API key first.";
      }
      alert(errorMsg);
    }
  });
}

function formatWeatherTime(timeStr) {
  if (!timeStr || timeStr === "N/A" || timeStr === "") return "N/A";
  try {
    var spaceIndex = timeStr.indexOf(' ');
    if (spaceIndex === -1) return timeStr;
    var timePart = timeStr.substring(spaceIndex + 1);
    if (timePart.length >= 5 && timePart.indexOf(':') !== -1) return timePart;
    return timeStr;
  } catch (e) {
    return timeStr;
  }
}

function formatWindDirection(degrees) {
  if (!degrees || degrees === "N/A" || isNaN(degrees)) return "N/A";
  var dir = parseFloat(degrees);
  var directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                   "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"];
  var index = Math.round(dir / 22.5) % 16;
  return dir.toFixed(0) + "\u00B0 (" + directions[index] + ")";
}

// Inline editing for table values — click any .editable span to edit
document.addEventListener('click', function(e) {
  var span = e.target.closest('.editable');
  if (!span || span.querySelector('input') || span.querySelector('select')) return;

  var currentValue = span.textContent.trim();
  if (currentValue === 'Loading...') return;

  var type = span.getAttribute('data-type');
  var committed = false;

  if (type === 'toggle') {
    // ON/OFF (or custom label) dropdown via GET URLs
    var onLabel = span.getAttribute('data-on-label') || 'ON';
    var offLabel = span.getAttribute('data-off-label') || 'OFF';

    var sel = document.createElement('select');
    sel.className = 'inline-edit';
    [onLabel, offLabel].forEach(function(label) {
      var opt = document.createElement('option');
      opt.value = label;
      opt.textContent = label;
      if (label === currentValue) opt.selected = true;
      sel.appendChild(opt);
    });

    span.textContent = '';
    span.appendChild(sel);
    sel.focus();

    function commitToggle() {
      if (committed) return;
      committed = true;
      var newValue = sel.value;
      span.textContent = newValue;
      if (newValue === currentValue) return;
      var url = (newValue === onLabel) ?
        span.getAttribute('data-on-url') : span.getAttribute('data-off-url');
      xget(url, function() { setTimeout(fetchConfig, 500); });
    }

    sel.addEventListener('change', commitToggle);
    sel.addEventListener('blur', function() {
      if (!committed) { committed = true; span.textContent = currentValue; }
    });

  } else if (type === 'select') {
    // Custom dropdown with POST endpoint
    var optionsStr = span.getAttribute('data-options') || '';
    var options = optionsStr.split(',').map(function(o) {
      var idx = o.indexOf(':');
      if (idx === -1) return { value: o, label: o };
      return { value: o.substring(0, idx), label: o.substring(idx + 1) };
    });

    var sel = document.createElement('select');
    sel.className = 'inline-edit';
    options.forEach(function(opt) {
      var el = document.createElement('option');
      el.value = opt.value;
      el.textContent = opt.label;
      // Match current text by checking if it starts with the value
      if (currentValue.indexOf(opt.value) === 0) el.selected = true;
      sel.appendChild(el);
    });

    span.textContent = '';
    span.appendChild(sel);
    sel.focus();

    function commitSelect() {
      if (committed) return;
      committed = true;
      var selectedValue = sel.value;
      var selectedLabel = sel.options[sel.selectedIndex].textContent;
      span.textContent = selectedLabel;

      // Check if value actually changed
      var changed = true;
      for (var i = 0; i < options.length; i++) {
        if (currentValue.indexOf(options[i].value) === 0 && options[i].value === selectedValue) {
          changed = false; break;
        }
      }
      if (!changed) return;

      var param = span.getAttribute('data-param');
      var endpoint = span.getAttribute('data-endpoint');

      var xhr = new XMLHttpRequest();
      xhr.open('POST', endpoint, true);
      xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
      xhr.onreadystatechange = function() {
        if (xhr.readyState == 4) {
          if (xhr.status >= 200 && xhr.status < 300) {
            fetchConfig();
          } else {
            span.textContent = currentValue;
            alert('Error: ' + (xhr.responseText || 'Failed to update'));
          }
        }
      };
      xhr.send(encodeURIComponent(param) + '=' + encodeURIComponent(selectedValue));
    }

    sel.addEventListener('change', commitSelect);
    sel.addEventListener('blur', function() {
      if (!committed) { committed = true; span.textContent = currentValue; }
    });

  } else {
    // Default: text input for param/endpoint based editing
    var input = document.createElement('input');
    input.type = 'text';
    input.value = currentValue;
    input.className = 'inline-edit';
    input.setAttribute('data-original', currentValue);

    span.textContent = '';
    span.appendChild(input);
    input.focus();
    input.select();

    function commitText() {
      if (committed) return;
      committed = true;

      var newValue = input.value.trim();
      var original = input.getAttribute('data-original');

      if (newValue === '' || newValue === original) {
        span.textContent = original;
        return;
      }

      var inputType = span.getAttribute('data-input-type');
      var dataMin = span.getAttribute('data-min');
      var dataMax = span.getAttribute('data-max');
      if (inputType === 'int') {
        if (!/^-?\d+$/.test(newValue)) { alert('Must be a whole number'); span.textContent = original; return; }
        var num = parseInt(newValue, 10);
        if (dataMin !== null && num < parseInt(dataMin, 10)) { alert('Value must be at least ' + dataMin); span.textContent = original; return; }
        if (dataMax !== null && num > parseInt(dataMax, 10)) { alert('Value must be at most ' + dataMax); span.textContent = original; return; }
      } else if (inputType === 'number') {
        var num = parseFloat(newValue);
        if (isNaN(num)) { alert('Must be a valid number'); span.textContent = original; return; }
        if (dataMin !== null && num < parseFloat(dataMin)) { alert('Value must be at least ' + dataMin); span.textContent = original; return; }
        if (dataMax !== null && num > parseFloat(dataMax)) { alert('Value must be at most ' + dataMax); span.textContent = original; return; }
      } else if (dataMin !== null || dataMax !== null) {
        var num = parseFloat(newValue);
        if (isNaN(num)) { alert('Must be a valid number'); span.textContent = original; return; }
        if (dataMin !== null && num < parseFloat(dataMin)) { alert('Value must be at least ' + dataMin); span.textContent = original; return; }
        if (dataMax !== null && num > parseFloat(dataMax)) { alert('Value must be at most ' + dataMax); span.textContent = original; return; }
      }

      var param = span.getAttribute('data-param');
      var endpoint = span.getAttribute('data-endpoint');
      var confirmMsg = span.getAttribute('data-confirm');

      if (confirmMsg && !confirm(confirmMsg)) {
        span.textContent = original;
        return;
      }

      span.textContent = newValue;

      // Convert displayed value back to metric for server submission
      var submitValue = newValue;
      var convertAttr = span.getAttribute('data-convert');
      if (convertAttr === 'speed' && useImperial) {
        submitValue = (parseFloat(newValue) / 0.621371).toFixed(1);
      }

      var body = encodeURIComponent(param) + '=' + encodeURIComponent(submitValue);

      // If this field has a paired field, include its current value too
      var pairAttr = span.getAttribute('data-pair');
      if (pairAttr) {
        var parts = pairAttr.split(':');
        var pairEl = document.getElementById(parts[0]);
        if (pairEl) body += '&' + encodeURIComponent(parts[1]) + '=' + encodeURIComponent(pairEl.textContent.trim());
      }

      var xhr = new XMLHttpRequest();
      xhr.open('POST', endpoint, true);
      xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
      xhr.onreadystatechange = function() {
        if (xhr.readyState == 4) {
          if (xhr.status >= 200 && xhr.status < 300) {
            if (confirmMsg) {
              alert('Device is restarting. Please wait and reload the page.');
            } else {
              fetchConfig();
            }
          } else {
            span.textContent = original;
            alert('Error: ' + (xhr.responseText || 'Failed to update'));
          }
        }
      };
      xhr.send(body);
    }

    input.addEventListener('blur', commitText);
    input.addEventListener('keydown', function(ev) {
      if (ev.key === 'Enter') {
        ev.preventDefault();
        input.blur();
      }
      if (ev.key === 'Escape') {
        committed = true;
        span.textContent = input.getAttribute('data-original');
      }
    });
  }
});

// Fetch and display firmware version
fetch('/version').then(function(r) { return r.text(); }).then(function(v) {
  var el = document.getElementById('firmwareVersion');
  if (el) el.textContent = 'Firmware v' + v;
});
