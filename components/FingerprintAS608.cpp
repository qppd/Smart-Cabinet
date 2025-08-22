#include "FingerprintAS608.h"

FingerprintAS608::FingerprintAS608(HardwareSerial &serial, uint32_t baud)
  : _serial(serial), _finger(&serial), _baud(baud) {}

void FingerprintAS608::begin() {
  _serial.begin(_baud);
  delay(100);
  _state = IDLE;
  _searchCb = nullptr;
}

bool FingerprintAS608::verifySensor() {
  return _finger.verifyPassword();
}

int FingerprintAS608::getTemplateCount() {
  return _finger.getTemplateCount();
}

bool FingerprintAS608::enroll(uint16_t id) {
  // Block until finger enrolled or failure
  if (id == 0) return false;
  int p = _finger.getImage();
  // Simple flow: call library examples for full implementation
  for (int i = 0; i < 3; ++i) {
    // wait for finger
    while (_finger.getImage() != FINGERPRINT_OK) delay(100);
    if (_finger.image2Tz(i+1) != FINGERPRINT_OK) return false;
    if (i == 0) {
      // ask for second reading
      // ...example simplified
    }
  }
  if (_finger.createModel() != FINGERPRINT_OK) return false;
  if (_finger.storeModel(id) != FINGERPRINT_OK) return false;
  return true;
}

int FingerprintAS608::search() {
  int p = _finger.getImage();
  if (p != FINGERPRINT_OK) return -1;
  if (_finger.image2Tz() != FINGERPRINT_OK) return -1;
  int res = _finger.fingerSearch();
  if (res == FINGERPRINT_OK) return _finger.fingerID;
  if (res == FINGERPRINT_NOTFOUND) return 0;
  return -1;
}

bool FingerprintAS608::startSearch(SearchCallback cb) {
  if (_state != IDLE) return false;
  _searchCb = cb;
  _state = SEARCHING;
  return true;
}

void FingerprintAS608::update() {
  if (_state == IDLE) return;
  // Try to get image; return if not ready yet
  int p = _finger.getImage();
  if (p != FINGERPRINT_OK) return; // still waiting
  if (_finger.image2Tz() != FINGERPRINT_OK) {
    if (_searchCb) _searchCb(-1);
    _state = IDLE;
    return;
  }
  int res = _finger.fingerSearch();
  if (res == FINGERPRINT_OK) {
    if (_searchCb) _searchCb(_finger.fingerID);
  } else if (res == FINGERPRINT_NOTFOUND) {
    if (_searchCb) _searchCb(0);
  } else {
    if (_searchCb) _searchCb(-1);
  }
  _state = IDLE;
}
