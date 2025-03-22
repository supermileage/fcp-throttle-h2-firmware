#include <Arduino.h>
#include "mcp2515_can.h"
#include <SPI.h>

#define DEBUG_SERIAL 0

//Pins
#define THROTTLE_IN A0
#define THROTTLE_OUT 10
#define THROTTLE_ENABLE 8
#define THROTTLE_MIN 0 // Minimum normalized output value
#define THROTTLE_MAX 128.0 // Maximum normalized output value
#define RAW_THROTTLE_MIN 0.83
#define RAW_THROTTLE_MAX 4.37
#define DIGIPOT_STEPS 128
#define H2_OUT 5
#define SPI_CS 6
#define CAN_SPEED CAN_500KBPS // speed of CAN network
#define CAN_CONTROLLER_SPEED MCP_8MHz 
#define CAN_H2_ID 0x256

#define THROTTLE_ENABLE_TIME 1000 //(ms)

#define H2_THRESHOLD 1.0 // % hydrogen concentration

mcp2515_can can(SPI_CS); // creating CAN object

static float currentH2Percent = 100;

// This struct contains all the components of a CAN message. dataLength must be <= 8, 
// and the first [dataLength] positions of data[] must contain valid data
typedef uint8_t CanBuffer[8];
	struct CanMessage {
			uint32_t id;
			uint8_t dataLength;
			CanBuffer data;
	};

void throttleHandle();
void canHandle();
void digiPotWrite(pin_size_t cs, uint8_t step);

int static a = 0;

String getErrorDescription(int errorCode){
  switch(errorCode){
      case CAN_OK: 
          return "CAN OK";
          break;
      case CAN_FAILINIT:
          return "CAN FAIL INIT";
          break;
      case CAN_FAILTX:
          return "CAN FAIL TX";
          break;
      case CAN_MSGAVAIL:
          return "CAN MSG AVAIL";
          break;
      case CAN_NOMSG:
          return "CAN NO MSG";
          break;
      case CAN_CTRLERROR:
          return "CAN CTRL ERROR";
          break;
      case CAN_GETTXBFTIMEOUT:
          return "CAN TX BF TIMEOUT";
          break;
      case CAN_SENDMSGTIMEOUT:    
          return "CAN SEND MSG TIMEOUT";
          break;
      default:
          return "CAN FAIL";
          break;
  }
}

void setup() {
  if(DEBUG_SERIAL){
  Serial.begin(9600);
  }
  // Pins
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  pinMode(THROTTLE_ENABLE, OUTPUT);
  pinMode(H2_OUT, OUTPUT);
  pinMode(SPI_CS, OUTPUT);

	// CAN begin
	int error = can.begin(CAN_SPEED, CAN_CONTROLLER_SPEED); // checks if it can communicate with mcp
  if(DEBUG_SERIAL){
	  Serial.println("CAN Init Status: " + getErrorDescription(error));
  }
  // SPI
  SPI.begin();

  //Throttle enable
  delay(THROTTLE_ENABLE_TIME);
  digitalWrite(THROTTLE_ENABLE, HIGH);
}

void loop() {
  throttleHandle();
  canHandle();
}

void canHandle(){
  float newH2Percent = currentH2Percent;
  if (can.checkReceive() == CAN_MSGAVAIL) {
    CanMessage message;
    message.id = 0;
    message.dataLength = 8;
    can.readMsgBuf(&message.dataLength, message.data); 
    // Polling based read of CAN message
    message.id = can.getCanId();
    if(DEBUG_SERIAL){
      Serial.print("ID: ");
      Serial.println(message.id);
    }
    if (message.id == CAN_H2_ID) {
      // Read H2 values in CAN frame
      uint16_t h2Int = 0;
      h2Int = message.data[1]<<8 | message.data[2];
      newH2Percent = h2Int * 0.01; // 0.00 - 100.00
      if(DEBUG_SERIAL){
        Serial.print("newH2Percent: ");
        Serial.println(newH2Percent);
      }
    } else{
      newH2Percent = 100.0;
      if(DEBUG_SERIAL){
        Serial.println("Uh oh");
      }
    }
  }

  if(newH2Percent == currentH2Percent){ return; } // No change, no need to write

  // Write
  if(newH2Percent >= H2_THRESHOLD){
    //Shut down
    digiPotWrite(H2_OUT, 128);
  } else{
    // We're good
    digiPotWrite(H2_OUT, 0);
  }
}

void throttleHandle(){
  int throttleRead = analogRead(THROTTLE_IN);
  float voltageValue = 5.0 / 1023.0 * throttleRead;
  int throttleNormalized = (((voltageValue) - RAW_THROTTLE_MIN) / (RAW_THROTTLE_MAX-RAW_THROTTLE_MIN)) * DIGIPOT_STEPS;
  int throttleScaled = throttleNormalized; // Option to add scaling factor here

  if(throttleScaled > THROTTLE_MAX){
    digiPotWrite(THROTTLE_OUT, THROTTLE_MAX);
  } else if(throttleScaled < THROTTLE_MIN){
    digiPotWrite(THROTTLE_OUT, THROTTLE_MIN);
  } else{
    digiPotWrite(THROTTLE_OUT,throttleScaled);
  }
}

// Change a digitpot to a certain step
// cs: which cs to enable
// step: scale the resistance from 0 - 128
void digiPotWrite(pin_size_t cs, uint8_t step){
  digitalWrite(cs, LOW);

  SPI.transfer(0x00);
  SPI.transfer(step);

  digitalWrite(cs, HIGH);
}
