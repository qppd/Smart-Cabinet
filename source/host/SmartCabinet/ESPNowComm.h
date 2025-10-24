#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Data structure for ESP-NOW messages
typedef struct {
    char command[32];      // Command type: "unlock", "authResult", "status"
    int userId;            // User ID from fingerprint
    bool success;          // Success/failure flag
    char data[64];         // Additional data
} ESPNowMessage;

class ESPNowComm {
public:
    ESPNowComm();
    void begin();
    void setPeerAddress(uint8_t* macAddress);
    void sendUnlockCommand(int userId);
    void sendAuthenticationResult(bool success, int userId);
    void sendMessage(const char* command, const char* data = "");
    bool hasConnectedPeer();
    
private:
    uint8_t peerMAC[6];
    bool peerSet;
    static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
    static void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);
};

#endif // ESPNOW_COMM_H
