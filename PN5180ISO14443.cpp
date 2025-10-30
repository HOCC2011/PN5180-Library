// #define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO14443.h"
#include <PN5180.h>
#include "Debug.h"

PN5180ISO14443::PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin) 
              : PN5180(SSpin, BUSYpin, RSTpin) {
}

bool PN5180ISO14443::setupRF() {
  PN5180DEBUG(F("Loading RF-Configuration...\n"));
  if (loadRFConfig(0x00, 0x80)) {  // ISO14443 parameters
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  PN5180DEBUG(F("Turning ON RF field...\n"));
  if (setRF_on()) {
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  return true;
}

uint16_t PN5180ISO14443::rxBytesReceived() {
	uint32_t rxStatus;
	uint16_t len = 0;
	readRegister(RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}
/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQA
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*
* return value: the uid length:
* -	zero if no tag was recognized
* -	single Size UID (4 byte)
* -	double Size UID (7 byte)
* -	triple Size UID (10 byte) - not yet supported
*/
uint8_t PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind, bool switchToIsoDep) {
    uint8_t cmd[7];
    uint8_t uidLength = 0;

    // ... (All your initialization and anti-collision code remains the same) ...

    // --- End of SAK Anti-Collision 1 ---
    //Read 1 byte SAK into buffer[2]
    if (!readData(1, buffer+2))
      return 0;

    // Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
    // If Bit 3 is 0 it is 4 Byte UID
    if ((buffer[2] & 0x04) == 0) {
       // Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
       for (int i = 0; i < 4; i++) buffer[3+i] = cmd[2 + i];
       uidLength = 4;
       // *** SAK is final SAK (from Select Anti-collision 1) ***
       this->lastSak = buffer[2];
    }
    else {
       // Take First 3 bytes of UID, Ignore first byte 88(CT)
       if (cmd[2] != 0x88)
         return 0;
       for (int i = 0; i < 3; i++) buffer[3+i] = cmd[3 + i];
       // Clear RX CRC
       if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))
          return 0;
       // Clear TX CRC
       if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))
          return 0;
       // Do anti collision 2
       cmd[0] = 0x95;
       cmd[1] = 0x20;
       if (!sendData(cmd, 2, 0x00))
          return 0;
       //Read 5 bytes. we will store at offset 2 for later use
       if (!readData(5, cmd+2))
          return 0;
       // first 4 bytes belongs to last 4 UID bytes, we keep it.
       for (int i = 0; i < 4; i++) {
         buffer[6 + i] = cmd[2+i];
       }
       //Enable RX CRC calculation
       if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01))
          return 0;
       //Enable TX CRC calculation
       if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01))
          return 0;
       //Send Select anti collision 2
       cmd[0] = 0x95;
       cmd[1] = 0x70;
       if (!sendData(cmd, 7, 0x00))
          return 0;
       //Read 1 byte SAK into buffer[2]
       if (!readData(1, buffer + 2))
          return 0;
       uidLength = 7;
       // *** SAK is final SAK (from Select Anti-collision 2) ***
       this->lastSak = buffer[2];
    }

    // Check if the code requests to communicate with the PICC in ISO-DEP (ISO 14443-4)
    if (switchToIsoDep) {
        PN5180DEBUG(F("Code requested to communicate in ISO-DEP\n"));
        // SAK Bit 6 (0x20) indicates ISO/IEC 14443-4 support
        if ((this->lastSak & 0x20) != 0) {
            PN5180DEBUG(F("PICC supports IDO-DEP!\n"));
            // NOTE: You need to pass appropriate parameters for ATS storage to StartIsoDep().
            // This example assumes StartIsoDep is updated to accept an ATS buffer.
            if (!startIsoDep(buffer + 3 + uidLength, 20)) {
                // RATS failed, return error or handle appropriately.
                // Since this is a critical failure if requested, returning 0 (failure) is reasonable.
                return 0;
            }
        } else {
             // Card does not support ISO-DEP (Bit 6 = 0)
             PN5180DEBUG(F("PICC doesn't support IDO-DEP!\n"));
        }
    }

    return uidLength;
}

bool PN5180ISO14443::startIsoDep(uint8_t *atsBuffer, uint8_t maxAtsLength) {
    // NOTE: The SAK check (this->lastSak & 0x20) is now handled in activateTypeA()
    // before calling this function, so we skip it here.

    // --- 1. Prepare and Send the RATS Command ---
    // RATS command: | 0xE0 | FSDI_CID |
    // FSDI = 0x08 (max FSD=2048 bytes) and CID = 0x00 -> FSDI_CID = 0x80
    uint8_t ratsCmd[2] = {0xE0, 0x80};

    // The RATS command must be sent with CRC, which is enabled at the end of anti-collision.

    // Send RATS command (2 bytes, 8 bits in last byte of the first frame, no trailing bits)
    if (!sendData(ratsCmd, 2, 0x00))
      return false;

    // --- 2. Read the ATS Response ---
    // The maximum size the card can send for ATS is 20 bytes (TL=0x14).
    // The 'maxAtsLength' parameter passed here defines the size of 'atsBuffer'.
    // We will use the passed 'maxAtsLength' as the maximum to read.

    // **FIX 1: Removed local declaration of maxAtsLength to avoid shadowing.**
    // **We directly use the function parameter 'maxAtsLength'.**

    // Use the provided atsBuffer and maxAtsLength parameter.
    // The max ATS size is 20 bytes. The passed maxAtsLength should ideally be >= 20.
    if (!readData(maxAtsLength, atsBuffer))
      return false;

    // --- 3. Process/Store Parameters and Return ---

    // **FIX 2: Need to return the received length.** // Since 'readData' doesn't return the length, we rely on 'rxBytesReceived()'
    // and store the length in a local variable.

    uint8_t actualAtsLength = rxBytesReceived();

    // The first byte of the response (atsBuffer[0]) is TL, the actual length of the ATS data.
    // If the received length (actualAtsLength) is valid, we can update the caller's knowledge
    // of the ATS length *by convention* or by using a member variable.

    // Since the original code was trying to write to *atsLength, which is no longer a pointer:
    // A common library practice is to store the actual received length in the class instance.
    // Let's assume you have a class member: this->lastAtsLength
    if (actualAtsLength == 0) return false;

    this->lastAtsLength = actualAtsLength; // Store the actual length for later use

    // Basic validation of the ATS length
    // TL (atsBuffer[0]) should be between 2 and 20. Actual length should be TL.
    if (atsBuffer[0] < 2 || atsBuffer[0] > 20) {
        // Invalid TL byte received
        return false;
    }

    // Success: The ATS is stored in atsBuffer, and its length is in this->lastAtsLength.
    return true;
}

uint16_t PN5180ISO14443::exchangeApdu(uint8_t *apduCommand, uint16_t commandLen, uint8_t *responseBuffer, uint16_t maxResponseLen) {
    uint16_t receivedLen = 0;

    // 1. Send the Command APDU
    // CRC is handled by the registers set during activation. PCB is assumed to be handled by the driver/firmware.
    // The last parameter (0x00) indicates no trailing bits.
    if (!sendData(apduCommand, commandLen, 0x00)) {
      // Communication failure during transmission
      return 0;
    }

    // 2. Wait for the PICC (card) to process and respond
    // The required time here depends on the card and command complexity.
    // This is the FWT (Frame Waiting Time) derived from the ATS.
    // 5ms is often a reasonable starting point for many standard commands.
    delay(5);

    // 3. Check how many bytes were received from the PICC
    receivedLen = rxBytesReceived();

    // 4. Validate the received length
    // A Response APDU must contain at least the 2-byte Status Word (SW1 SW2).
    if (receivedLen < 2) {
      // Too short for a valid APDU response, or no response received.
      return 0;
    }

    // Also ensure the received length does not exceed the provided buffer size
    if (receivedLen > maxResponseLen) {
      // If the card sends more data than the buffer can hold, this indicates an issue
      // (e.g., failed chaining or buffer too small). We should still read 'maxResponseLen'
      // to clear the FIFO, but report failure.
      if (readData(maxResponseLen, responseBuffer)) {
          // Read successful, but length was clipped. Return 0 for failure to signal buffer overflow/issue.
          return 0;
      }
      return 0; // Read failed
    }

    // 5. Read the valid data into the response buffer
    if (!readData(receivedLen, responseBuffer)) {
      // Read failed even though length was reported
      return 0;
    }

    // Success: return the actual length of the received Response APDU.
    return receivedLen;
}

bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer) {
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	if (!sendData(cmd, 2, 0x00))
	  return false;
	//Check if we have received any data from the tag
	delay(5);
	len = rxBytesReceived();
	if (len == 16) {
		// READ 16 bytes into  buffer
		if (readData(16, buffer))
		  success = true;
	}
	return success;
}

uint8_t PN5180ISO14443::mifareBlockWrite16(uint8_t blockno, uint8_t *buffer) {
	uint8_t cmd[1];
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	readData(1, cmd);

	// Mifare write part 2
	sendData(buffer,16, 0x00);
	delay(10);

	// Read ACK/NAK
	readData(1, cmd);

	//Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}

bool PN5180ISO14443::mifareHalt() {
	uint8_t cmd[1];
	//mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	sendData(cmd, 2, 0x00);	
	return true;
}

uint8_t PN5180ISO14443::readCardSerial(uint8_t *buffer) {
  
    uint8_t response[10];
	uint8_t uidLength;
	// Always return 10 bytes
    // Offset 0..1 is ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
    for (int i = 0; i < 10; i++) response[i] = 0;
    uidLength = activateTypeA(response, 1, false);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
	  return 0;
	// check for valid uid
	if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00))
	  return 0;
	if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF))
	  return 0;
    for (int i = 0; i < 7; i++) buffer[i] = response[i+3];
	mifareHalt();
	return uidLength;  
}

bool PN5180ISO14443::isCardPresent() {
    uint8_t buffer[10];
	return (readCardSerial(buffer) >=4);
}

