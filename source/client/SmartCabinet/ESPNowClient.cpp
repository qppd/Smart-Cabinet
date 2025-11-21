#include "ESPNowClient.h"

MessageCallback ESPNowClient::messageCallback = nullptr;

ESPNowClient::ESPNowClient() : peerSet(false) {
    memset(peerMAC, 0, sizeof(peerMAC));
}

void ESPNowClient::begin() {
    // Set device as a Wi-Fi Station (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    
    Serial.print("[ESPNowClient] MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNowClient] Error initializing ESP-NOW");
        return;
    }
    
    Serial.println("[ESPNowClient] ESP-NOW initialized successfully");
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
}

void ESPNowClient::setPeerAddress(uint8_t* macAddress) {
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
        Serial.println("[ESPNowClient] Failed to add peer");
        peerSet = false;
        return;
    }
    
    Serial.print("[ESPNowClient] Peer added: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", peerMAC[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    peerSet = true;
}

void ESPNowClient::sendStatus(const char* status) {
    if (!peerSet) return;
    
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strcpy(msg.command, "status");
    strncpy(msg.data, status, sizeof(msg.data) - 1);
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowClient] Sent status: ");
        Serial.println(status);
    } else {
        Serial.println("[ESPNowClient] Error sending status");
    }
}

void ESPNowClient::sendMotionDetected(bool detected) {
    if (!peerSet) return;
    
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strcpy(msg.command, "motion");
    msg.success = detected;
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowClient] Motion: ");
        Serial.println(detected ? "detected" : "cleared");
    }
}

void ESPNowClient::sendDoorState(bool isOpen) {
    if (!peerSet) return;
    
    ESPNowMessage msg;
    memset(&msg, 0, sizeof(msg));
    strcpy(msg.command, "doorState");
    msg.success = isOpen;
    
    esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    
    if (result == ESP_OK) {
        Serial.print("[ESPNowClient] Door: ");
        Serial.println(isOpen ? "open" : "closed");
    }
}

void ESPNowClient::setMessageCallback(MessageCallback callback) {
    messageCallback = callback;
}

bool ESPNowClient::hasConnectedPeer() {
    return peerSet;
}

void ESPNowClient::onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Silent acknowledgment - can add logging if needed
}

void ESPNowClient::onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    ESPNowMessage* msg = (ESPNowMessage*)data;
    
    Serial.print("[ESPNowClient] Received: ");
    Serial.print(msg->command);
    Serial.print(" | UserID: ");
    Serial.print(msg->userId);
    Serial.print(" | Success: ");
    Serial.println(msg->success);
    
    if (messageCallback) {
        messageCallback(msg->command, msg->userId, msg->success, msg->data);
    }
}
