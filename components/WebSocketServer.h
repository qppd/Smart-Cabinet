#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

class WebSocketServer {
private:
    AsyncWebServer server;
    AsyncWebSocket ws;

    static void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

public:
    WebSocketServer(uint16_t port, const char* wsPath);
    void begin();
    void sendMessageToAll(const String& message);
};

#endif
