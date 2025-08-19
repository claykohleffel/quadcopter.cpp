//DRONE CODE 

#include <RF24.h> 
#include <Servo.h> 

const uint64_t address = 0x4F8D2A1C9ELL;
bool armed = false; 
bool linkAlive = false;
unsigned long lastCheck = 0;
// 11,12,13 for SPI and 1 for Serial
// ESC pins are motor pins
RF24 radio(7,8);

Servo mtr1, mtr2, mtr3, mtr4; 

// use same struct as the controller so that data will be properly stored
struct ctrlpkt {
  uint8_t seq; 
  uint16_t thr, pit, yaw, roll; 
  uint8_t arm;
};

ctrlpkt latestpkt = {0, 1000, 500, 500, 500, 0}; 


void setup() {

mtr1.attach(3); 
mtr2.attach(5); 
mtr3.attach(6); 
mtr4.attach(9); 

for (int i = 0; i < 100; i++) {
  mtr1.writeMicroseconds(1000);
  mtr2.writeMicroseconds(1000);
  mtr3.writeMicroseconds(1000);
  mtr4.writeMicroseconds(1000);
  delay(20); 
}

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
radio.setRetries(5, 15); 
// openReadingPipe takes two arguments, pipe number and address
radio.openReadingPipe(0, address); 
radio.startListening(); 


}


void loop() {
ctrlpkt p; 
// Check for availiable data
if (radio.available()) {  
// store info in new array, declare size
  radio.read(&p, sizeof(ctrlpkt));
  Serial.println("Received");
  latestpkt = p;
  // check if link is alive; difference between pkts < 200 ms
  lastCheck = millis();
  linkAlive = true; 
  armed = (p.arm == 1);
}
 

// Scale motors controls to throttle so that 2000 = 1 and 1000 = 0 for smooth descents
float scale = float(latestpkt.thr - 1000) / 1000; 
// subtract 500 from values so that joystick values will provide neg values for controls
float newYaw = (latestpkt.yaw - 500) * scale; 
float newPit = (latestpkt.pit - 500) * scale; 
float newRoll = (latestpkt.roll - 500) * scale; 

int m1 = latestpkt.thr - newPit + newYaw - newRoll; 
int m2 = latestpkt.thr - newPit - newYaw + newRoll; 
int m3 = latestpkt.thr + newPit - newYaw - newRoll; 
int m4 = latestpkt.thr + newPit + newYaw + newRoll; 

m1 = constrain(m1, 1000, 2000); 
m2 = constrain(m2, 1000, 2000); 
m3 = constrain(m3, 1000, 2000); 
m4 = constrain(m4, 1000, 2000); 

// write power to the motors
if (armed) {
mtr1.writeMicroseconds(m1); 
mtr2.writeMicroseconds(m2); 
mtr3.writeMicroseconds(m3); 
mtr4.writeMicroseconds(m4);
}
else {
mtr1.writeMicroseconds(1000); 
mtr2.writeMicroseconds(1000); 
mtr3.writeMicroseconds(1000); 
mtr4.writeMicroseconds(1000);
}


//create boolean
linkAlive = (millis() - lastCheck) <= 200;
if (!(linkAlive)) {
  armed = false;  
  latestpkt.thr = 1000;
}

}
