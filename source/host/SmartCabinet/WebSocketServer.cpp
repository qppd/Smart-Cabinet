
#include "WebSocketServer.h"

WebSocketServer::WebSocketServer(uint16_t port_) 
    : port(port_), clientConnected(false), ssid(nullptr), password(nullptr) {}

void WebSocketServer::setWiFiCredentials(const char* ssid_, const char* password_) {
    ssid = ssid_;
    password = password_;
}

bool WebSocketServer::connectWiFi() {
    if (ssid == nullptr || password == nullptr) {
        Serial.println("[WebSocketServer] WiFi credentials not set!");
        return false;
    }
    
    Serial.println("[WebSocketServer] Connecting to WiFi...");
    WiFi.begin(ssid, password);
    
    // Wait up to 15 seconds for connection
    for(int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.print(".");
        delay(1000);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("[WebSocketServer] WiFi connected");
        Serial.print("[WebSocketServer] IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("");
        Serial.println("[WebSocketServer] WiFi connection failed!");
        return false;
    }
}

String WebSocketServer::getIPAddress() {
    return WiFi.localIP().toString();
}

void WebSocketServer::begin() {
    server.listen(port);
    Serial.print("[WebSocketServer] Listening on port ");
    Serial.println(port);
    Serial.print("[WebSocketServer] Server live? ");
    Serial.println(server.available() ? "Yes" : "No");
}

void WebSocketServer::loop() {
    if (!clientConnected) {
        client = server.accept();
        if (client.available()) {
            clientConnected = true;
            Serial.println("[WebSocketServer] Client connected");
        }
    } else {
        if (!client.available()) {
            clientConnected = false;
            Serial.println("[WebSocketServer] Client disconnected");
        } else {
            // Poll for incoming messages (non-blocking)
            if (client.poll()) {
                // Message available, read it
                while (client.available()) {
                    WebsocketsMessage msg = client.readBlocking();
                    if (msg.data().length() > 0) {
                        Serial.print("[WebSocketServer] Got Message: ");
                        Serial.println(msg.data());
                    }
                }
            }
        }
    }
}

bool WebSocketServer::hasConnectedClients() {
    return clientConnected && client.available();
}

void WebSocketServer::sendUnlockCommand(int userId) {
    if (clientConnected && client.available()) {
        String message = "{\"command\":\"unlock\",\"userId\":" + String(userId) + "}";
        client.send(message);
        Serial.print("[WebSocketServer] Sent unlock command for user: ");
        Serial.println(userId);
    }
}

void WebSocketServer::sendAuthenticationResult(bool success, int userId) {
    if (clientConnected && client.available()) {
        String message = "{\"command\":\"authResult\",\"success\":" + String(success ? "true" : "false") + ",\"userId\":" + String(userId) + "}";
        client.send(message);
        Serial.print("[WebSocketServer] Sent auth result: ");
        Serial.println(success ? "success" : "failed");
    }
}

void WebSocketServer::sendMessage(const String& message) {
    if (clientConnected && client.available()) {
        client.send(message);
        Serial.print("[WebSocketServer] Sent message: ");
        Serial.println(message);
    } else {
        Serial.println("[WebSocketServer] No client connected, cannot send message");
    }
}
