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

bool FingerprintAS608::isFingerDetected() {
  uint8_t p = _finger.getImage();
  // Only return true if we successfully got an image
  // FINGERPRINT_NOFINGER means no finger detected
  return (p == FINGERPRINT_OK);
}

int FingerprintAS608::authenticate() {
  // Step 1: Get fingerprint image
  uint8_t p = _finger.getImage();
  if (p != FINGERPRINT_OK) {
    // Failed to get image (no finger, communication error, etc.)
    // Return -1 for no finger detected (silent failure)
    return -1;
  }
  
  // Step 2: Convert image to template
  p = _finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    // Failed to convert (messy image, feature extraction failed, etc.)
    Serial.println("[FingerprintAS608] Image conversion failed");
    return -2; // Image quality issue or wrong finger
  }
  
  // Step 3: Search for match in database
  p = _finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    // Found a match! Return the fingerprint ID
    Serial.print("[FingerprintAS608] Match found! ID: ");
    Serial.print(_finger.fingerID);
    Serial.print(" Confidence: ");
    Serial.println(_finger.confidence);
    return _finger.fingerID;
  } else if (p == FINGERPRINT_NOTFOUND) {
    // No match found in database
    Serial.println("[FingerprintAS608] No match found");
    return -2; // Fingerprint scanned but not in database
  } else {
    // Communication or other error
    Serial.println("[FingerprintAS608] Search error");
    return -1;
  }
}

bool FingerprintAS608::enroll(uint16_t id) {
  // ID #0 not allowed
  if (id == 0) {
    Serial.println("[FingerprintAS608] ID #0 not allowed");
    return false;
  }
  
  int p = -1;
  
  // Step 1: Get first fingerprint image
  Serial.print("[FingerprintAS608] Waiting for finger to enroll as ID #");
  Serial.println(id);
  while (p != FINGERPRINT_OK) {
    p = _finger.getImage();
    if (p == FINGERPRINT_OK) {
      Serial.println("[FingerprintAS608] Image taken");
    } else if (p == FINGERPRINT_NOFINGER) {
      // Still waiting for finger
      delay(100);
    } else {
      Serial.println("[FingerprintAS608] Error capturing image");
      return false;
    }
  }
  
  // Convert first image to template 1
  p = _finger.image2Tz(1);
  if (p != FINGERPRINT_OK) {
    Serial.println("[FingerprintAS608] Failed to convert first image");
    return false;
  }
  Serial.println("[FingerprintAS608] First image converted");
  
  // Step 2: Wait for finger removal
  Serial.println("[FingerprintAS608] Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = _finger.getImage();
  }
  
  // Step 3: Get second fingerprint image
  Serial.println("[FingerprintAS608] Place same finger again");
  p = -1;
  while (p != FINGERPRINT_OK) {
    p = _finger.getImage();
    if (p == FINGERPRINT_OK) {
      Serial.println("[FingerprintAS608] Image taken");
    } else if (p == FINGERPRINT_NOFINGER) {
      // Still waiting for finger
      delay(100);
    } else {
      Serial.println("[FingerprintAS608] Error capturing second image");
      return false;
    }
  }
  
  // Convert second image to template 2
  p = _finger.image2Tz(2);
  if (p != FINGERPRINT_OK) {
    Serial.println("[FingerprintAS608] Failed to convert second image");
    return false;
  }
  Serial.println("[FingerprintAS608] Second image converted");
  
  // Step 4: Create model from both templates
  Serial.print("[FingerprintAS608] Creating model for ID #");
  Serial.println(id);
  p = _finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("[FingerprintAS608] Prints matched!");
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("[FingerprintAS608] Fingerprints did not match");
    return false;
  } else {
    Serial.println("[FingerprintAS608] Error creating model");
    return false;
  }
  
  // Step 5: Store the model
  p = _finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.print("[FingerprintAS608] Stored! ID #");
    Serial.println(id);
    return true;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("[FingerprintAS608] Could not store in that location");
    return false;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("[FingerprintAS608] Error writing to flash");
    return false;
  } else {
    Serial.println("[FingerprintAS608] Unknown storage error");
    return false;
  }
}

bool FingerprintAS608::deleteFingerprint(uint16_t id) {
  // ID #0 not allowed
  if (id == 0) {
    Serial.println("[FingerprintAS608] ID #0 not allowed");
    return false;
  }
  
  Serial.print("[FingerprintAS608] Deleting ID #");
  Serial.println(id);
  
  uint8_t p = _finger.deleteModel(id);
  
  if (p == FINGERPRINT_OK) {
    Serial.println("[FingerprintAS608] Deleted!");
    return true;
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("[FingerprintAS608] Communication error");
    return false;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("[FingerprintAS608] Could not delete in that location");
    return false;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("[FingerprintAS608] Error writing to flash");
    return false;
  } else {
    Serial.print("[FingerprintAS608] Unknown error: 0x");
    Serial.println(p, HEX);
    return false;
  }
}

bool FingerprintAS608::emptyDatabase() {
  Serial.println("[FingerprintAS608] Emptying database...");
  
  uint8_t p = _finger.emptyDatabase();
  
  if (p == FINGERPRINT_OK) {
    Serial.println("[FingerprintAS608] Database emptied!");
    return true;
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("[FingerprintAS608] Communication error");
    return false;
  } else if (p == FINGERPRINT_DBCLEARFAIL) {
    Serial.println("[FingerprintAS608] Database clear failed");
    return false;
  } else {
    Serial.print("[FingerprintAS608] Unknown error: 0x");
    Serial.println(p, HEX);
    return false;
  }
}

uint8_t FingerprintAS608::loadModel(uint16_t id) {
  // Try to load the model from flash to buffer
  // Returns FINGERPRINT_OK if the model exists at this ID
  // Returns error code if model doesn't exist or other error
  return _finger.loadModel(id);
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
