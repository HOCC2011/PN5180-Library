#include <Debug.h>
#include <PN5180.h>
#include <PN5180ISO14443.h>

#define PN5180_RST  4
#define PN5180_NSS  5
#define PN5180_BUSY 6

PN5180ISO14443 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);

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

bool errorFlag = false;

void loop() {
  if (errorFlag) {
    uint32_t irqStatus = nfc.getIRQStatus();
    showIRQStatus(irqStatus);

    if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) { // no card detected
      Serial.println(F("*** No card detected!"));
    }

    nfc.reset();
    nfc.setupRF();

    errorFlag = false;
    delay(10);
  }
  uint8_t uid[10];
  if (nfc.isIsoDepCardPresent()) {
    Serial.println(F("ISO14443 card found"));
    Serial.println(F("IsoDep started successfully."));

    uint8_t selectCommand[20] = {0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00}; //PPSE select command
    uint8_t response[256];
    uint16_t responseLength;
    // Send the command and automatically get the length and data
    responseLength = nfc.exchangeApdu(selectCommand, sizeof(selectCommand), response, sizeof(response), 10);
    if (responseLength > 0) {
      Serial.print(F("Response data: "));
      Serial.print(bytesToHex(response, responseLength));
      Serial.println(F(""));
    } else {
      Serial.println(F("responseLength is < 0"));
    }
  } 
  delay(1000);
}

String bytesToHex(unsigned char* data, unsigned int len) {
  String hexString = "";
  for (unsigned int i = 0; i < len; i++) {
    char temp[3]; // 2 chars for hex, 1 for null terminator
    sprintf(temp, "%02X", data[i]);
    hexString += temp;
  }
  return hexString;
}

void showIRQStatus(uint32_t irqStatus) {
  Serial.print(F("IRQ-Status 0x"));
  Serial.print(irqStatus, HEX);
  Serial.print(": [ ");
  if (irqStatus & (1<< 0)) Serial.print(F("RQ "));
  if (irqStatus & (1<< 1)) Serial.print(F("TX "));
  if (irqStatus & (1<< 2)) Serial.print(F("IDLE "));
  if (irqStatus & (1<< 3)) Serial.print(F("MODE_DETECTED "));
  if (irqStatus & (1<< 4)) Serial.print(F("CARD_ACTIVATED "));
  if (irqStatus & (1<< 5)) Serial.print(F("STATE_CHANGE "));
  if (irqStatus & (1<< 6)) Serial.print(F("RFOFF_DET "));
  if (irqStatus & (1<< 7)) Serial.print(F("RFON_DET "));
  if (irqStatus & (1<< 8)) Serial.print(F("TX_RFOFF "));
  if (irqStatus & (1<< 9)) Serial.print(F("TX_RFON "));
  if (irqStatus & (1<<10)) Serial.print(F("RF_ACTIVE_ERROR "));
  if (irqStatus & (1<<11)) Serial.print(F("TIMER0 "));
  if (irqStatus & (1<<12)) Serial.print(F("TIMER1 "));
  if (irqStatus & (1<<13)) Serial.print(F("TIMER2 "));
  if (irqStatus & (1<<14)) Serial.print(F("RX_SOF_DET "));
  if (irqStatus & (1<<15)) Serial.print(F("RX_SC_DET "));
  if (irqStatus & (1<<16)) Serial.print(F("TEMPSENS_ERROR "));
  if (irqStatus & (1<<17)) Serial.print(F("GENERAL_ERROR "));
  if (irqStatus & (1<<18)) Serial.print(F("HV_ERROR "));
  if (irqStatus & (1<<19)) Serial.print(F("LPCD "));
  Serial.println("]");
}