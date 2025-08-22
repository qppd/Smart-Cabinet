#include "WebSocketClient.h"

WebSocketClient::WebSocketClient() {}

void WebSocketClient::begin(const char* host, uint16_t port, const char* wsPath) {
    webSocket.begin(host, port, wsPath);
    webSocket.onEvent(onWebSocketEvent);
}

void WebSocketClient::loop() {
    webSocket.loop();
}

void WebSocketClient::sendMessage(const String& message) {
    webSocket.sendTXT(message);
}

void WebSocketClient::onWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        String message = String((char*)payload).substring(0, length);
        Serial.println("Received: " + message);
        // Handle the received message here
    }
}
