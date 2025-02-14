#include <Arduino.h>

#include <mcp2515.h>
#include <SPI.h>

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

#define THROTTLE_ENABLE_TIME 1000 //(ms)

#define H2_THRESHOLD 1.0

struct can_frame canMsg;
MCP2515 mcp2515(SPI_CS);

void throttleHandle();
void canHandle();
void digiPotWrite(pin_size_t cs, uint8_t step);

void setup() {
  // Pins
  Serial.begin(9600);
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  pinMode(THROTTLE_ENABLE, OUTPUT);
  pinMode(H2_OUT, OUTPUT);

  // mcp2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
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
  // Polling based read of CAN message
  float h2Percent = 0;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Read H2 values in CAN frame
    uint16_t h2Int = 0;
    h2Int = canMsg.data[1]<<8 | canMsg.data[2];
    h2Percent = h2Int * 0.01; // 0.00 - 100.00
  }

  // Write
  if(h2Percent >= H2_THRESHOLD){
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
  int throttleScaled = throttleNormalized;

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
