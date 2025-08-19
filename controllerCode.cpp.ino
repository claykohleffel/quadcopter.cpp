// CONTROLLER CODE
#include <RF24.h>

//nRF24 can only hold 32 bytes

//CE = D9 , CSN = D10
RF24 radio(9,10); 


//address for TX-RX
const uint64_t address = 0x4F8D2A1C9ELL;
//count amt of packets held
uint8_t seqctr = 0;


struct ctrlpkt {
  uint8_t seq; 
  uint8_t flag; 
  uint16_t thr, pit, yaw, roll; 
  uint8_t arm; 
};


//initialize pins
const int armPin = 2;
const int thrPin = A0; 
const int pitPin = A2; 
const int yawPin = A3;
const int rollPin = A1; 


//joystick pin array
const int jPins[4] = {A0, A2, A3, A1};

/*function to map the 0-1023 for joysticks to 0-1000
use long so mult doesn't overflow using large numbers*/
inline uint16_t joy1000(int v) {
  return (uint16_t)(((long)v * 1000L) / 1023L);
}


void setup() {
/*
INITIALIZE: Serial!, Joystick Pins, 
nRF24 Radio, Packet State
*/
pinMode(armPin , INPUT_PULLUP); 


Serial.begin(115200); 
radio.begin(); 
radio.setAddressWidth(5); 
//when setting channel, 0-125 are valid
radio.setChannel(108);
radio.setDataRate(RF24_250KBPS); 
/*Setting lowest power amplifier level for testing; increase later on
- will control range but with a tradeoff of amt of current drawn and possible transmission interference*/
radio.setPALevel(RF24_PA_MIN); 
//using 16 bits for cyclic redundancy check to ensure good data being transmitted in high traffic area
radio.setCRCLength(RF24_CRC_16); 
radio.setAutoAck(true); 
radio.setRetries(3, 5); // 3 = 750 microsec delay, 5 = retry sending packet 5 times

radio.openWritingPipe(address); 
radio.stopListening(); // puts controller in TX mode


}



void loop() {
  //initialize variable pkt for struct
  ctrlpkt p = {0}; 
  seqctr++; 

  p.seq = seqctr; 
//work on arming drone and failsafes. 

  bool armed = digitalRead(armPin) == HIGH; 

  if (!(armed)) {
    p.arm = false; 
    p.thr = 0;
    p.pit = 0;
    p.yaw = 0;
    p.roll = 0;  
    radio.write(&p, sizeof(p));
    return; 
  }
  

  for (int i = 0; i < 4; i++) {
    int raw = analogRead(jPins[i]); 
    uint16_t fix = joy1000(raw); 
    if (i == 0) {p.thr = fix;}
    if (i == 1) {p.pit = fix;} 
    if (i == 2) {p.yaw = fix;}
    if (i == 3) {p.roll = fix;}
    
  }

  radio.write(&p, sizeof(p)); 
}
