#include "ESPNowComm.h"

ESPNowComm::ESPNowComm() : peerSet(false) {
    memset(peerMAC, 0, sizeof(peerMAC));
}

void ESPNowComm::begin() {
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    
    Serial.print("[ESPNowComm] MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNowComm] Error initializing ESP-NOW");
        return;
    }
    
    Serial.println("[ESPNowComm] ESP-NOW initialized successfully");
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
}

void ESPNowComm::setPeerAddress(uint8_t* macAddress) {
    memcpy(peerMAC, macAddress, 6);
    
    // Add peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, peerMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    // Remove peer if already exists
    if (esp_now_is_peer_exist(peerMAC)) {
        esp_now_del_peer(peerMAC);
    }
    
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESPNowComm] Failed to add peer");
        peerSet = false;
        return;
    }
    
    Serial.print("[ESPNowComm] Peer added: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", peerMAC[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    peerSet = true;
}

void ESPNowComm::sendUnlockCommand(int userId) {
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strcpy(msg.command, "unlock");
    msg.userId = userId;
    msg.success = true;
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowComm] Sent unlock command for user: ");
        Serial.println(userId);
    } else {
        Serial.println("[ESPNowComm] Error sending unlock command");
    }
}

void ESPNowComm::sendAuthenticationResult(bool success, int userId) {
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strcpy(msg.command, "authResult");
    msg.userId = userId;
    msg.success = success;
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowComm] Sent auth result: ");
        Serial.println(success ? "success" : "failed");
    } else {
        Serial.println("[ESPNowComm] Error sending auth result");
    }
}

void ESPNowComm::sendMessage(const char* command, const char* data) {
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strncpy(msg.command, command, sizeof(msg.command) - 1);
    if (data) {
        strncpy(msg.data, data, sizeof(msg.data) - 1);
    }
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowComm] Message sent: ");
        Serial.println(command);
    } else {
        Serial.println("[ESPNowComm] Error sending message");
    }
}

bool ESPNowComm::hasConnectedPeer() {
    return peerSet;
}

void ESPNowComm::onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.print("[ESPNowComm] Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void ESPNowComm::onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    ESPNowMessage* msg = (ESPNowMessage*)data;
    Serial.print("[ESPNowComm] Received command: ");
    Serial.println(msg->command);
}
