// WiFi station + fixed-size TCP frame server shared by the arm and claw firmware.
//
// Replaces the original WiFiConnection.hpp. Behaviour kept from it: station mode, optional
// static IP, power save off, one client per port, one 64-byte request -> one 64-byte response.
// Changes: never blocks for long (reconnects in the background instead of spinning in a
// while-loop), several ports can be served from one task, and a new connection replaces a stale
// one, so a restarted ROS2 driver can reconnect without power-cycling the ESP32.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <functional>

namespace dume {

struct NetConfig {
  const char* ssid;
  const char* password;
  const char* hostname;
  bool useStaticIp;
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
};

class WiFiStation {
 public:
  void begin(const NetConfig& cfg) {
    _cfg = cfg;
    WiFi.persistent(false);
    WiFi.setHostname(_cfg.hostname);  // must precede mode()/begin() on arduino-esp32 3.x
    WiFi.mode(WIFI_STA);
    if (_cfg.useStaticIp) WiFi.config(_cfg.ip, _cfg.gateway, _cfg.subnet, _cfg.gateway);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_cfg.ssid, _cfg.password);
    _lastAttemptMs = millis();
    Serial.printf("[wifi] connecting to '%s'\n", _cfg.ssid);
  }

  // Call regularly. Returns true while connected.
  bool poll() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!_connected) {
        _connected = true;
        esp_wifi_set_ps(WIFI_PS_NONE);  // modem sleep adds 100+ ms latency spikes
        Serial.printf("[wifi] connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(),
                      WiFi.RSSI());
      }
      return true;
    }
    if (_connected) {
      _connected = false;
      _lastAttemptMs = millis();
      Serial.println("[wifi] connection lost");
    }
    // Auto-reconnect usually handles it; kick it again if it has been stuck for a while.
    if (millis() - _lastAttemptMs > kRetryMs) {
      _lastAttemptMs = millis();
      Serial.println("[wifi] retrying");
      WiFi.disconnect();
      WiFi.begin(_cfg.ssid, _cfg.password);
    }
    return false;
  }

  bool connected() const { return _connected; }

 private:
  static constexpr uint32_t kRetryMs = 15000;
  NetConfig _cfg{};
  bool _connected = false;
  uint32_t _lastAttemptMs = 0;
};

// Serves fixed-size request/response frames on one TCP port.
// handler(request, response, newClient) fills the response and returns true to send it.
template <size_t FrameSize>
class FrameServer {
 public:
  using Handler = std::function<bool(const uint8_t* request, uint8_t* response, bool newClient)>;

  FrameServer(uint16_t port, Handler handler) : _server(port), _port(port), _handler(handler) {}

  void begin() {
    if (_started) return;
    _server.begin();
    _server.setNoDelay(true);
    _started = true;
  }

  void poll() {
    if (!_started) return;

    // A new connection always wins: the old socket is usually a dead peer that never sent FIN.
    if (_server.hasClient()) {
      WiFiClient incoming = _server.accept();
      if (_hasClient && _client.connected()) {
        Serial.printf("[net:%u] replacing existing client\n", _port);
        _client.stop();
      }
      _client = incoming;
      _client.setNoDelay(true);
      _hasClient = true;
      _newClient = true;
      _partialSinceMs = 0;
      Serial.printf("[net:%u] client %s connected\n", _port, _client.remoteIP().toString().c_str());
    }

    if (!_hasClient) return;
    if (!_client.connected()) {
      Serial.printf("[net:%u] client disconnected\n", _port);
      _client.stop();
      _hasClient = false;
      return;
    }

    int avail = _client.available();
    if (avail >= (int)FrameSize) {
      uint8_t request[FrameSize];
      uint8_t response[FrameSize];
      _client.read(request, FrameSize);
      memset(response, 0, FrameSize);
      _partialSinceMs = 0;
      _lastFrameMs = millis();
      _frames++;
      if (_handler(request, response, _newClient)) _client.write(response, FrameSize);
      _newClient = false;
    } else if (avail > 0) {
      // Clients send whole frames; a partial frame that never completes means we lost sync.
      if (_partialSinceMs == 0) {
        _partialSinceMs = millis();
      } else if (millis() - _partialSinceMs > kPartialTimeoutMs) {
        Serial.printf("[net:%u] dropping %d stray bytes\n", _port, avail);
        while (_client.available()) _client.read();
        _partialSinceMs = 0;
      }
    }
  }

  bool hasClient() { return _hasClient && _client.connected(); }
  uint32_t lastFrameMs() const { return _lastFrameMs; }
  uint32_t frameCount() const { return _frames; }

 private:
  static constexpr uint32_t kPartialTimeoutMs = 500;
  WiFiServer _server;
  WiFiClient _client;
  uint16_t _port;
  Handler _handler;
  bool _started = false;
  bool _hasClient = false;
  bool _newClient = false;
  uint32_t _partialSinceMs = 0;
  uint32_t _lastFrameMs = 0;
  uint32_t _frames = 0;
};

}  // namespace dume
