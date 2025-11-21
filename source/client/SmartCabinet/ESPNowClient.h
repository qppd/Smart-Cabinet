#ifndef ESPNOW_CLIENT_H
#define ESPNOW_CLIENT_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Data structure for ESP-NOW messages (must match host)
typedef struct {
    char command[32];      // Command type: "unlock", "authResult", "status"
    int userId;            // User ID from fingerprint
    bool success;          // Success/failure flag
    char data[64];         // Additional data
} ESPNowMessage;

// Callback function type
typedef void (*MessageCallback)(const char* command, int userId, bool success, const char* data);

class ESPNowClient {
public:
    ESPNowClient();
    void begin();
    void setPeerAddress(uint8_t* macAddress);
    void sendStatus(const char* status);
    void sendMotionDetected(bool detected);
    void sendDoorState(bool isOpen);
    void setMessageCallback(MessageCallback callback);
    bool hasConnectedPeer();
    
private:
    uint8_t peerMAC[6];
    bool peerSet;
    static MessageCallback messageCallback;
    static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
    static void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);
};

#endif // ESPNOW_CLIENT_H
