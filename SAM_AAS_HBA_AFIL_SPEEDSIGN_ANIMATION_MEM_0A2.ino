//Sketch by Nico1080, "Modified by Zettainfo" @ds5_ups.and.mods


//This sketch will make the AAS (parking sensor) and SAM (blind spot monitoring) toogle key on NAC work (BSI telecoding for AAS is optional)
//It also allow physical button and LED to work again. Check the connection yourself. Beware that LED are in +12v logic
//This skecth was tested on a DS4, and should also work on any C4 gen 2 (hatcback, sedan, etc)
//HBA (High Beam Assist) do not work on DS4/C4 (car ignore CAN message from cluster), but it should be possible to make it work on DS5
//Many thank to Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration and to Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


//ZETTAINFO Added HBA led frame, AFIL and HBA variables and codes, activate pull_up for all pins, add speedsign's Keryan code, and create some variables to change screen with the bitRead function on MEM button, now everything work with just one arduino.
//e-mail: rodrigo @zettainfo.com.br
//+55-47-98874-9877
//@ds5_ups.and.mods


/////////////////////
//    Libraries    //
/////////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ // Switch to 16MHZ if you have a 16Mhz module


// LED and button input
const int AASBUTTON_PIN = A3; // Arduino pin connected to AAS button's pin
const int AASLED_PIN    = 3; // Arduino pin connected to AAS LED's pin
const int SAMBUTTON_PIN = A4; // Arduino pin connected to SAM button's pin
const int SAMLED_PIN    = 4; // Arduino pin connected to SAM LED's pin
const int HBABUTTON_PIN = A5; // Arduino pin connected to HBA button's pin
const int HBALED_PIN    = 5; // Arduino pin connected to HBA LED's pin


////////////////////
// Initialization //
////////////////////

MCP2515 CAN0(CS_PIN_CAN0);

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;
bool SerialEnabled = true;
int Animation = 0x00;
int modes = 0x00;
long lastmode;

// timer for physical switch
long lastSAM;
long lastSAM_NAC;
long lastAFIL;
long lastAFIL_NAC;
long lastHBA;
long lastHBA_NAC;
long lastAAS;

//Bool for avoiding duplicate push
bool SAMsend = false;
bool SAMpushrelease = true;
bool AFILsend = false;
bool HBAsend = false;
bool HBApushrelease = true;
bool AASsendNAC = false;
bool AASsendCLUSTER = false;
bool AASpushrelease = true;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;
char tmp[4];

void setup() {
  pinMode(HBALED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(SAMLED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(AASLED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(HBABUTTON_PIN, INPUT_PULLUP);
  pinMode(SAMBUTTON_PIN, INPUT_PULLUP);
  pinMode(AASBUTTON_PIN, INPUT_PULLUP);
  digitalWrite(HBALED_PIN, LOW); // Initial state of HBA LED
  digitalWrite(SAMLED_PIN, LOW); // Initial state of SAM LED
  digitalWrite(AASLED_PIN, LOW); // Initial state of AAS LED



  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN0");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can ok");
  }
}

void loop() {
  // Receive CAN messages from the car
  if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN0 ) {
      Serial.print("FRAME:ID=");
      Serial.print(id);
      Serial.print(":LEN=");
      Serial.print(len);

      char tmp[3];
      for (int i = 0; i < len; i++) {
        Serial.print(":");

        snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

        Serial.print(tmp);
      }

      Serial.println();
    }

    if (id == 0x1A9) {
      if (bitRead(canMsgRcv.data[3] , 5 ) == 1 ) {
        if (millis() - lastSAM_NAC > 300) {
          SAMsend = true;
          lastSAM_NAC = millis();
        }
      }
      if (AASsendNAC) {
        canMsgSnd = canMsgRcv;
        bitWrite(canMsgSnd.data[3] , 2, 1 );
        AASsendNAC = false;
        CAN0.sendMessage( & canMsgSnd);
      }
    }
    if (id == 0x2D1) {
      if (bitRead(canMsgRcv.data[0] , 2 ) == 0 ) {
        digitalWrite(SAMLED_PIN, HIGH);
      }
      else {
        digitalWrite(SAMLED_PIN, LOW);
      }
      if (id == 0x1A9) {
        if (bitRead(canMsgRcv.data[3] , 3 ) == 1 ) {
          if (millis() - lastHBA_NAC > 300) {
            HBAsend = true;
            lastHBA_NAC = millis();
          }
        }
      }
      if (id == 0x227) {
        if (bitRead(canMsgRcv.data[3] , 4 ) == 0 ) {
          digitalWrite(HBALED_PIN, LOW);
        }
        else {
          digitalWrite(HBALED_PIN, HIGH);
        }
      }
    }
    if (id == 0x227) {
      if (bitRead(canMsgRcv.data[0] , 6 ) == 1 ) {
        digitalWrite(AASLED_PIN, HIGH);
      }
      else  {
        digitalWrite(AASLED_PIN, LOW);
      }
    }

    if (millis() - lastSAM > 500) {
      if (digitalRead(SAMBUTTON_PIN) == LOW && SAMpushrelease) {
        SAMsend = true;
        lastSAM = millis();
        SAMpushrelease = false;
      }
      else if (digitalRead(SAMBUTTON_PIN) == HIGH) {
        SAMpushrelease = true;
      }
    }
    if (millis() - lastAAS > 500) {
      if (digitalRead(AASBUTTON_PIN) == LOW && AASpushrelease) {
        AASsendNAC = true;
        AASsendCLUSTER = true;
        lastAAS = millis();
        AASpushrelease = false;
      }
      else if (digitalRead(AASBUTTON_PIN) == HIGH) {
        AASpushrelease = true;
      }
    }
    if (id == 0x217 && (SAMsend || AASsendCLUSTER )) {
      canMsgSnd = canMsgRcv;
      if (SAMsend) {
        bitWrite(canMsgSnd.data[3] , 3, 1 );
        SAMsend = false;
      }
      CAN0.sendMessage( & canMsgSnd);
    }
    if (id == 0x1A9) {
      if (bitRead(canMsgRcv.data[3] , 3 ) == 1 ) {
        if (millis() - lastHBA_NAC > 300) {
          HBAsend = true;
          lastHBA_NAC = millis();
        }
      }
    }
    if (id == 0x227) {
      if (bitRead(canMsgRcv.data[3] , 4 ) == 0 ) {
        digitalWrite(HBALED_PIN, LOW);
      }
      else {
        digitalWrite(HBALED_PIN, HIGH);
      }
    }
    if (millis() - lastHBA > 500) {
      if (digitalRead(HBABUTTON_PIN) == LOW && HBApushrelease) {
        HBAsend = true;
        lastHBA = millis();
        HBApushrelease = false;
      }
      else if (digitalRead(HBABUTTON_PIN) == HIGH) {
        HBApushrelease = true;
      }
    }
    if (id == 0x217 && (HBAsend || AASsendCLUSTER )) {
      canMsgSnd = canMsgRcv;
      if (HBAsend) {
        bitWrite(canMsgSnd.data[3] , 7, 1 );
        HBAsend = false;
      }
      if (AASsendCLUSTER) {
        bitWrite(canMsgSnd.data[2] , 7, 1 );
        AASsendCLUSTER = false;
      }
      CAN0.sendMessage( & canMsgSnd);
    }
    if (id == 0x1A9) {
      if (bitRead(canMsgRcv.data[3] , 4 ) == 1 ) {
        if (millis() - lastAFIL_NAC > 300) {
          AFILsend = true;
          lastAFIL_NAC = millis();
        }
      }
    }
    if (id == 0x217 && (AFILsend)) {
      canMsgSnd = canMsgRcv;
      if (AFILsend) {
        bitWrite(canMsgSnd.data[3] , 6, 1 );
        AFILsend = false;
      }
      CAN0.sendMessage( & canMsgSnd);
    }

    if (id == 0x1E8 && bitRead(canMsgRcv.data[2] , 5 ) == 1 ) {
      CAN0.sendMessage( & canMsgRcv);
      if (millis() - lastmode > 500) {
        modes += 1;
        lastmode = millis();
      }
    }
    else if (id == 0x0A2) {
      canMsgSnd.data[0] = canMsgRcv.data[0];
      canMsgSnd.data[1] = canMsgRcv.data[1];
      canMsgSnd.data[2] = canMsgRcv.data[2];
      canMsgSnd.data[3] = modes;
      canMsgSnd.data[4] = canMsgRcv.data[4];
      canMsgSnd.data[5] = canMsgRcv.data[5];
      canMsgSnd.can_id = 0x0A2;
      canMsgSnd.can_dlc = 6;
      CAN0.sendMessage( & canMsgSnd);
    }
    if (id == 0x1E9 && bitRead(canMsgRcv.data[2], 3) == 1) {
      canMsgSnd.data[0] = canMsgRcv.data[1];
      canMsgSnd.data[1] = 0x10;
      canMsgSnd.data[2] = 0x00;
      canMsgSnd.data[3] = 0x00;
      canMsgSnd.data[4] = 0x00;
      canMsgSnd.data[5] = 0x00;
      canMsgSnd.data[6] = 0x00;
      canMsgSnd.data[7] = 0x00;
      canMsgSnd.can_id = 0x268;
      canMsgSnd.can_dlc = 8;
      CAN0.sendMessage( & canMsgSnd);
    }
    if (id == 0x236) {
      if (Animation == 0x00 && millis() > 2000) {
        canMsgSnd.data[5] = bitWrite(canMsgRcv.data[5], 6, 1);
        canMsgSnd.can_id = 0x236;
        canMsgSnd.can_dlc = 8;
        CAN0.sendMessage( & canMsgSnd);
        Animation = 0x01;
      }
    }
  }
}
