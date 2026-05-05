/*
 * Firmware for the discovery-drive satellite dish rotator.
 * rotctl WiFi - Allow for direct rotctl connections over WiFi.
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

 // Implements the Rotctl NET Protocol

#include "rotctl_wifi.h"

RotctlWifi::RotctlWifi(Preferences& prefs, MotorSensorController& motorSensorCtrl, Logger& logger)
    : _preferences(prefs), _motorSensorCtrl(motorSensorCtrl), _logger(logger) {
}

void RotctlWifi::begin() {
    _rotator_server = new WiFiServer(_preferences.getInt("rotctl_port", DEFAULT_ROTCTL_PORT));
    _rotator_server->begin();
    _logger.info("Rotator rotctl TCP server started");
}

void RotctlWifi::rotctlWifiLoop(bool serialActive, bool stellariumOn) {
    // If stellarium polling is on, or serial is active, disconnect rotctl client
    if (stellariumOn || serialActive) {
        if (_rotator_client && _rotator_client.connected()) {
            disconnectClient();
        }
        return;
    }
    
    // Handle rotator control for GPredict and Look4Sat direct control
    handleClientConnection();
    handleClientCommands();

    // Check for disconnected clients with grace period for mesh handoff.
    // During WiFi mesh handoff, connected() returns false momentarily.
    // Without the grace period, we'd destroy a connection that TCP would
    // have kept alive through retransmission.
    if (_rotator_client && !_rotator_client.connected()) {
        if (_disconnectDetectedMs == 0) {
            _disconnectDetectedMs = millis();
            _logger.debug("Client connection lost — starting " + String(DISCONNECT_GRACE_MS / 1000) + "s grace period");
        } else if (millis() - _disconnectDetectedMs > DISCONNECT_GRACE_MS) {
            _logger.info("Client disconnected after grace period");
            disconnectClient();
            _disconnectDetectedMs = 0;
        }
    } else if (_disconnectDetectedMs != 0) {
        // Connection recovered during grace period (mesh handoff survived)
        _logger.info("Client connection recovered after " + String(millis() - _disconnectDetectedMs) + "ms");
        _disconnectDetectedMs = 0;
    }
}

void RotctlWifi::handleClientConnection() {
    WiFiClient newClient = _rotator_server->available();

    if (newClient) {
        // A new connection is pending — accept it
        if (_rotator_client && _rotator_client.connected()) {
            // Replace existing client (likely stale after mesh handoff)
            _logger.warn("New client connecting while existing client present — replacing (old: " + _rotctl_client_ip + ")");
            _rotator_client.stop();
        }
        _rotator_client = newClient;
        _rotctl_client_ip = _rotator_client.remoteIP().toString();
        _rotator_client.setTimeout(60);
        enableTcpKeepalive();
        _lastClientActivity = millis();
        _logger.info("Client connected from " + _rotctl_client_ip);
    }
}

void RotctlWifi::handleClientCommands() {
    if (!_rotator_client || !_rotator_client.connected()) {
        // Don't reset IP here — let the grace period logic in rotctlWifiLoop handle it
        return;
    }
    
    if (_rotator_client.available() <= 0) {
        return;
    }
    
    String request = readCommandFromClient();
    if (request.length() == 0) {
        return;
    }
    
    _logger.debug("Received message: " + request);
    
    // Handle different rotctl commands
    if (request.startsWith("\\P") || request.startsWith("P") || request.startsWith("\\set_pos") || request.startsWith("set_pos")) {
        if (_motorSensorCtrl.isSafeMode()) {
            _logger.info("SAFE MODE - position command blocked (rotctl WiFi)");
            _rotator_client.print("RPRT -1\n");
        } else {
            handlePositionCommand(request);
        }
    } else if (request == "p" || request == "\\get_pos") {
        handleGetPositionCommand();
    } else if (request == "S" || request == "\\stop") {
        if (_motorSensorCtrl.isSafeMode()) {
            _logger.info("SAFE MODE - stop command blocked (rotctl WiFi)");
            _rotator_client.print("RPRT -1\n");
        } else {
            handleStopCommand();
            _rotator_client.print("RPRT 0\n");
        }
    } else if (request.startsWith("R") || request.startsWith("\\reset")) {
        if (_motorSensorCtrl.isSafeMode()) {
            _logger.info("SAFE MODE - reset command blocked (rotctl WiFi)");
            _rotator_client.print("RPRT -1\n");
        } else {
            handleResetCommand();
            _rotator_client.print("RPRT 0\n");
        }
    } else if (request == "K" || request == "\\park") {
        if (_motorSensorCtrl.isSafeMode()) {
            _logger.info("SAFE MODE - park command blocked (rotctl WiFi)");
            _rotator_client.print("RPRT -1\n");
        } else {
            handleParkCommand();
            _rotator_client.print("RPRT 0\n");
        }
    } else if (request == "_" || request == "\\get_info") {
        _rotator_client.print("Discovery Drive V1.0\n");
    } else if (request == "?" || request == "dump_state" || request == "\\dump_state") {
        handleDumpStateCommand();
    } else if (request == "1" || request == "dump_caps" || request == "\\dump_caps") {
        handleDumpCapsCommand();
    } else {
        _logger.error("Unexpected message format: " + request);
        _rotator_client.print("RPRT -1\n");         // better than silence
    }
    
    _lastClientActivity = millis();
}

String RotctlWifi::readCommandFromClient() {
    String request = "";
    unsigned long startMillis = millis();

    while (millis() - startMillis < READ_TIMEOUT) {
        while (_rotator_client.available()) {
            char c = _rotator_client.read();
            if (c == '\n') {
                request.trim();  // Strip \r and any whitespace
                return request;
            }
            if (request.length() < 256) {
                request += c;
            }
        }
        delay(1);  // Yield CPU to other tasks while waiting for more data
    }

    request.trim();
    return request;
}

void RotctlWifi::handleDumpStateCommand() {
    // NET rotctl expects:
    // 1) protocol version
    // 2) rotator model number behind the server
    // 3-6) min/max az/el
    //
    // Protocol "0" uses plain numeric min/max lines.
    // Protocol "1" tends to use key=value lines (min_az=..., etc.).

    const int protoVer = 0;

    // If you are the rotator directly (not a proxy to a serial rotator),
    // you can return "0" or "2" here. Returning "0" is commonly used to mean "unknown".
    const int backendModel = 2;

    const float minAz = -360.0f;
    const float maxAz =  360.0f;
    const float minEl =    0.0f;
    const float maxEl =   90.0f;

    String resp;
    resp.reserve(120);

    resp += String(protoVer);    resp += "\n";
    resp += String(backendModel);resp += "\n";

    resp += String(minAz, 6);    resp += "\n";
    resp += String(maxAz, 6);    resp += "\n";
    resp += String(minEl, 6);    resp += "\n";
    resp += String(maxEl, 6);    resp += "\n";

    _rotator_client.print(resp);
    _logger.info("Responded dump_state:\n" + resp);
}

void RotctlWifi::handleDumpCapsCommand() {
    String resp;
    resp.reserve(800);

    resp += "Caps dump for model: 2\n";
    resp += "Model name:\tDiscovery Drive\n";
    resp += "Mfg name:\tKrakenRF\n";
    resp += "Backend version:\t1.0\n";
    resp += "Backend copyright:\tGPL\n";
    resp += "Backend status:\tStable\n";
    resp += "Rot type:\tAz/El\n";
    resp += "Port type:\tNetwork link\n";
    resp += "Write delay:\t0mS, timeout 2000mS, 3 retries\n";
    resp += "Post Write delay:\t0mS\n";
    resp += "Status flags:\tNone\n";
    resp += "Get functions:\n";
    resp += "Set functions:\n";
    resp += "Extra functions:\n";
    resp += "Get level:\n";
    resp += "Set level:\n";
    resp += "Extra levels:\n";
    resp += "Get parameters:\n";
    resp += "Set parameters:\n";
    resp += "Extra parameters:\n";
    resp += "Min Azimuth:\t-360.00\n";
    resp += "Max Azimuth:\t360.00\n";
    resp += "Min Elevation:\t0.00\n";
    resp += "Max Elevation:\t90.00\n";
    resp += "Has priv data:\tN\n";
    resp += "Has Init:\tN\n";
    resp += "Has Cleanup:\tN\n";
    resp += "Has Open:\tY\n";
    resp += "Has Close:\tY\n";
    resp += "Can set Conf:\tN\n";
    resp += "Can get Conf:\tN\n";
    resp += "Can set Position:\tY\n";
    resp += "Can get Position:\tY\n";
    resp += "Can Stop:\tY\n";
    resp += "Can Park:\tY\n";
    resp += "Can Reset:\tY\n";
    resp += "Can Move:\tN\n";
    resp += "Can get Info:\tY\n";
    resp += "Can get Status:\tN\n";
    resp += "Can set Func:\tN\n";
    resp += "Can get Func:\tN\n";
    resp += "Can set Level:\tN\n";
    resp += "Can get Level:\tN\n";
    resp += "Can set Param:\tN\n";
    resp += "Can get Param:\tN\n";

    _rotator_client.print(resp);
    _logger.info("Responded dump_caps");
}

void RotctlWifi::handlePositionCommand(const String& request) {
    float az = 0.0f, el = 0.0f;
    
    // Find first space, parse numbers after it
    int spaceIdx = request.indexOf(' ');
    if (spaceIdx < 0 || sscanf(request.c_str() + spaceIdx, " %f %f", &az, &el) != 2) {
        _logger.error("Failed to parse position command: " + request);
        _rotator_client.print("RPRT -1\n");
        return;
    }
    
    az = cleanupAzimuth(az);
    el = cleanupElevation(el);
    
    _motorSensorCtrl.setSetPointAz(az);
    _motorSensorCtrl.setSetPointEl(el);
    
    _logger.debug("Parsed Azimuth: " + String(_motorSensorCtrl.getSetPointAz(), 2) +
                  ", Elevation: " + String(_motorSensorCtrl.getSetPointEl(), 2));
    
    _rotator_client.print("RPRT 0\n");
}

void RotctlWifi::handleGetPositionCommand() {
    float el = _motorSensorCtrl.getCorrectedAngleEl();

    // Hack to stop it breaking displays in satdump when el sits at ~359.99 instead of 0
    if (el > 359) {
        el = 0;
    }

    if (_motorSensorCtrl.isFlipModeEnabled()) {
        // Normalize internal angle to (-180, 180] before translating, otherwise
        // sensor readings near 360° (e.g. 359.5 = -0.5°) would emit absurd flip values.
        if (el > 180.0f) el -= 360.0f;
        el = MotorSensorController::internalToFlip(el);
    }

    String response = String(_motorSensorCtrl.getCorrectedAngleAz(), 2) + "\n" +
                     String(el, 2) + "\n";
    _rotator_client.print(response);
    _logger.debug("Responded with position: " + response);
}

void RotctlWifi::handleStopCommand() {
    _motorSensorCtrl.setSetPointAz(_motorSensorCtrl.getCorrectedAngleAz());
    _motorSensorCtrl.setSetPointEl(_motorSensorCtrl.getCorrectedAngleEl());
}

void RotctlWifi::handleResetCommand() {
    // Soft reset - reinitialize the controller
    _logger.info("Rotator reset requested");
    ESP.restart();
}

void RotctlWifi::handleParkCommand() {
    float homeEl = _motorSensorCtrl.getHomeElInternal();
    _motorSensorCtrl.setSetPointAz(0);
    _motorSensorCtrl.setSetPointEl(homeEl);
    _logger.info("Parking rotator to home position (0, " + String(homeEl, 1) + ")");
}

float RotctlWifi::cleanupAzimuth(float az) {
    if (isnan(az)) {
        az = 0;
    }
    
    az = fmod(az, 360.0);
    if (az < 0) {
        az += 360.0;
    }
    
    return az;
}

float RotctlWifi::cleanupElevation(float el) {
    if (isnan(el)) {
        el = 0;
    }

    if (_motorSensorCtrl.isFlipModeEnabled()) {
        // Client speaks flip [0, 180]; clamp on that scale, then translate to internal.
        if (el < 0.0f) el = 0.0f;
        if (el > 180.0f) el = 180.0f;
        return MotorSensorController::flipToInternal(el);
    }

    float minEl = _motorSensorCtrl.isExtendedElEnabled() ? -90.0f : 0.0f;
    if (el < minEl) el = minEl;
    if (el > 90) el = 90;

    return el;
}

void RotctlWifi::enableTcpKeepalive() {
    int fd = _rotator_client.fd();
    if (fd < 0) return;

    int keepAlive = 1;
    int keepIdle = 5;       // Start probing after 5s idle
    int keepInterval = 5;   // Probe every 5s
    int keepCount = 3;      // Drop after 3 failed probes (~20s total)

    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(keepAlive));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(keepIdle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(keepInterval));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(keepCount));

    _logger.debug("TCP keepalive enabled (idle=5s, interval=5s, count=3)");
}

void RotctlWifi::disconnectClient() {
    _rotctl_client_ip = "NO ROTCTL CONNECTION";
    _disconnectDetectedMs = 0;
    _rotator_client.stop();
    _rotator_client = WiFiClient();
}

String RotctlWifi::getRotctlClientIP() {
    return _rotctl_client_ip;
}

bool RotctlWifi::isRotctlConnected() {
    return _rotator_client.connected();
}