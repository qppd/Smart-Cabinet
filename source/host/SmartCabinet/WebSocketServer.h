
#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <ArduinoWebsockets.h>
#include <WiFi.h>

using namespace websockets;

class WebSocketServer {
private:
    WebsocketsServer server;
    WebsocketsClient client;
    uint16_t port;
    bool clientConnected;
    const char* ssid;
    const char* password;
public:
    WebSocketServer(uint16_t port = 80);
    void setWiFiCredentials(const char* ssid, const char* password);
    bool connectWiFi();
    void begin();
    void loop();
    bool hasConnectedClients();
    void sendUnlockCommand(int userId);
    void sendAuthenticationResult(bool success, int userId);
    void sendMessage(const String& message);
    String getIPAddress();
};

#endif
