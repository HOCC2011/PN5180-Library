#include <Debug.h>
#include <PN5180.h>
#include <PN5180ISO14443.h>

#define PN5180_RST  4
#define PN5180_NSS  5
#define PN5180_BUSY 6

PN5180ISO14443 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST); // 3 arguments

void setup() {
  Serial.begin(115200);
  nfc.begin();

  Serial.println(F("PN5180 Hard-Reset..."));
  nfc.reset();

  Serial.println(F("Reading product version..."));
  uint8_t productVersion[2];
  nfc.readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));
  Serial.print(F("Product version="));
  Serial.print(productVersion[1]);
  Serial.print(F("."));
  Serial.println(productVersion[0]);

  if (0xff == productVersion[1]) { // if product version 255, the initialization failed
    Serial.println(F("Initialization failed!?"));
    Serial.println(F("Press reset to restart..."));
    Serial.flush();
    rgbLedWrite(21, 0, RGB_BRIGHTNESS, 0); 
    exit(-1); // halt
  }

  Serial.println(F("Reading firmware version..."));
  uint8_t firmwareVersion[2];
  nfc.readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  Serial.print(F("Firmware version="));
  Serial.print(firmwareVersion[1]);
  Serial.print(F("."));
  Serial.println(firmwareVersion[0]);

  Serial.println(F("Reading EEPROM version..."));
  uint8_t eepromVersion[2];
  nfc.readEEprom(EEPROM_VERSION, eepromVersion, sizeof(eepromVersion));
  Serial.print(F("EEPROM version="));
  Serial.print(eepromVersion[1]);
  Serial.print(F("."));
  Serial.println(eepromVersion[0]);

  Serial.println(F("Enabling RF field..."));
  nfc.setupRF();
  rgbLedWrite(21, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS); 
}

void loop() {
  uint8_t uid[10];
  if (nfc.isCardPresent()) {
    Serial.println(F("ISO14443 card found"));
    
    uint8_t uidLength = nfc.readCardSerial(uid);
    
    if (uidLength > 0) {
      Serial.print(F("ISO14443 card found, UID="));
      for (int i=0; i<uidLength; i++) {
        Serial.print(uid[i] < 0x10 ? " 0" : " ");
        Serial.print(uid[i], HEX);
      }
      Serial.println();
    }
  } 
  delay(1000);
}