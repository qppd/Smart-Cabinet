
#include "WebSocketServer.h"

WebSocketServer::WebSocketServer(uint16_t port_) : port(port_) {}

void WebSocketServer::begin() {
    server.listen(port);
    Serial.print("[WebSocketServer] Listening on port ");
    Serial.println(port);
}

void WebSocketServer::loop() {
    WebsocketsClient client = server.accept();
    if(client.available()) {
        WebsocketsMessage msg = client.readBlocking();
        Serial.print("[WebSocketServer] Got Message: ");
        Serial.println(msg.data());
        // Echo back
        client.send("Echo: " + msg.data());
        client.close();
    }
}
