#include "WebSocketServer.h"

WebSocketServer::WebSocketServer(uint16_t port, const char* wsPath) : server(port), ws(wsPath) {
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
}

void WebSocketServer::begin() {
    server.begin();
}

void WebSocketServer::sendMessageToAll(const String& message) {
    ws.textAll(message);
}

void WebSocketServer::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        String message = String((char*)data).substring(0, len);
        Serial.println("Received: " + message);
        // Handle the received message here
    }
}
