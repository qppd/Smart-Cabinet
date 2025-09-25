
#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <ArduinoWebsockets.h>
#include <WiFi.h>

using namespace websockets;

class WebSocketServer {
private:
    WebsocketsServer server;
    uint16_t port;
public:
    WebSocketServer(uint16_t port = 80);
    void begin();
    void loop();
};

#endif
