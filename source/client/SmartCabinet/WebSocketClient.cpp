#include "WebSocketClient.h"

WebSocketClient::WebSocketClient() : port(81), path("/"), connected(false) {}

void WebSocketClient::begin(const char* host_, uint16_t port_, const char* path_) {
    host = String(host_);
    port = port_;
    path = String(path_);
    connected = wsClient.connect(host, port, path);
    if (connected) {
        Serial.println("[WebSocketClient] Connected to server");
        wsClient.send("Hello Server");
    } else {
        Serial.println("[WebSocketClient] Connection failed");
    }
    wsClient.onMessage([this](WebsocketsMessage message){
        this->onMessageCallback(message);
    });
}

void WebSocketClient::loop() {
    if (connected) {
        wsClient.poll();
    }
}

void WebSocketClient::sendMessage(const String& message) {
    if (connected) {
        wsClient.send(message);
    }
}

bool WebSocketClient::isConnected() const {
    return connected;
}

void WebSocketClient::onMessageCallback(WebsocketsMessage message) {
    Serial.print("[WebSocketClient] Received: ");
    Serial.println(message.data());
    // Handle the received message here
}