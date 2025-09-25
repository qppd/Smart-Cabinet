#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <ArduinoWebsockets.h>
#include <WiFi.h>

using namespace websockets;

class WebSocketClient {
private:
    WebsocketsClient wsClient;
    String host;
    uint16_t port;
    String path;
    bool connected;
    void onMessageCallback(WebsocketsMessage message);
public:
    WebSocketClient();
    void begin(const char* host, uint16_t port = 81, const char* path = "/");
    void loop();
    void sendMessage(const String& message);
    bool isConnected() const;
};

#endif