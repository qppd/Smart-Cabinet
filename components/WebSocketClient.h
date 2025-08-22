#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <WebSocketsClient.h>

class WebSocketClient {
private:
    WebSocketsClient webSocket;

    static void onWebSocketEvent(WStype_t type, uint8_t *payload, size_t length);

public:
    WebSocketClient();
    void begin(const char* host, uint16_t port, const char* wsPath);
    void loop();
    void sendMessage(const String& message);
};

#endif
