#include <Arduino.h>

#include <mcp2515.h>
#include <SPI.h>

//Pins
#define THROTTLE_IN
#define H2_IN
#define THROTTLE_OUT D10
#define THROTTLE_ENABLE D8
#define H2_OUT D5
#define SPI_CS D6

#define THROTTLE_ENABLE_TIME 1000 //(ms)

#define H2_THRESHOLD 1.0

struct can_frame canMsg;

MCP2515 mcp2515(SPI_CS);

void setup() {
  // Pins
  Serial.begin(9600);
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  pinMode(THROTTLE_ENABLE, OUTPUT);
  pinMode(H2_IN, INPUT);
  pinMode(H2_OUT, OUTPUT);

  // mcp2525
  mcp2525.reset();
  mcp2525.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2525.setNormalMode();
  
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
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Read H2 values in CAN frame
    uint16_t h2Int = 0;
    h2Int = data[1]<<8 | data[2];
    float h2Percent = h2Int * 0.01 // 0.00 - 100.00
  }
  // Write
  if(h2Percent >= H2_THRESHOLD){
    
  } else{

  }
}

void throttleHandle(){


}