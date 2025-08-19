//DRONE CODE 

#include <RF24.h> 

const uint64_t address = 0x4F8D2A1C9ELL;

// 11,12,13 for SPI and 1 for Serial
// ESC pins are motor pins
RF24 radio(9,10);
const int mtr1 = 3;
const int mtr2 = 4;
const int mtr3 = 5; 
const int mtr4 = 6; 

// use same struct as the controller so that data will be properly stored
struct ctrlpkt {
  uint8_t seq; 
  uint16_t thr, pit, yaw, roll; 
  uint8_t arm;
};

void setup() {


Serial.begin(115200); 
//start radio
radio.begin(); 
radio.setPayloadSize(sizeof(ctrlpkt)); 
radio.setAddressWidth(5); 

//set radio data
radio.setChannel(108); 
radio.setDataRate(RF24_250KBPS); 
radio.setPALevel(RF24_PA_MIN); 
radio.setCRCLength(RF24_CRC_16); 
radio.setAutoAck(true); 

// openReadingPipe takes two arguments, pipe number and address
radio.openReadingPipe(0, address); 
radio.startListening(); 

// set pinmodes , ESC's
pinMode(mtr1, OUTPUT); 
pinMode(mtr2, OUTPUT); 
pinMode(mtr3, OUTPUT); 
pinMode(mtr4, OUTPUT); 

}


void loop() {
// Check for availiable data
if (radio.available()) {
  char pktinfo[6]; 
// store info in new array, declare size
  radio.read(&pktinfo, sizeof(ctrlpkt));
  Serial.print("Recieved");
}




}
